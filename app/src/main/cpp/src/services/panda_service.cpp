#include "services/panda_service.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "utils/adas_topics.h"
#include "utils/logger.h"
#include "utils/protobuf_utils.h"
#include "volkswagen/values.h"

namespace {
constexpr int64_t kHcaCmdTimeoutMs = 250;
}

PandaService::PandaService(Config config)
  : config_(std::move(config)), usb_fd_(config_.usb_fd), dbc_path_(config_.dbc_path)
{
}

void PandaService::configure()
{
  try {
    if (!dbc_path_.empty()) {
      dbc_ = std::make_unique<DBSParser>(dbc_path_);
      decoder_.setDbc(dbc_.get());
      LOGI("Loaded DBC: %s (%zu messages)", dbc_path_.c_str(), dbc_->getAllMessages().size());
    } else {
      LOGW("No DBC path — CarState decode disabled");
    }

    initializePanda();

    subscribe<ai::flow::adas::ZMQMessage>(adas::topics::kSteerCommand,
                                          [this](const ai::flow::adas::ZMQMessage& m) { steerCommandCallback(m); });
    subscribe<ai::flow::adas::ZMQMessage>(adas::topics::kLongPlan,
                                          [this](const ai::flow::adas::ZMQMessage& m) { longPlanCallback(m); });

    scheduleTimer(
        10, [this] { pandaRxCallback(); }, "rx");
    scheduleTimer(
        100, [this] { pandaStateCallback(); }, "state");
    scheduleTimer(
        10, [this] { carControllerCallback(); }, "tx");

    LOGI("PandaService configured (cruise_buttons=%s)", config_.cruise_buttons_enabled ? "ON" : "off");
  } catch (const std::exception& e) {
    LOGE("Failed to configure PandaService: %s", e.what());
    throw;
  }
}

void PandaService::initializePanda()
{
  LOGI("Initializing Panda device...");
  panda_ = std::make_shared<Panda>(usb_fd_, 0);
  LOGI("Panda instance created successfully");
}

void PandaService::steerCommandCallback(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_steer_command())
    return;
  const auto& cmd = msg.steer_command();
  using P = volkswagen::CarControllerParams;
  const int torque = std::clamp(cmd.torque_cnm(), -P::STEER_MAX, P::STEER_MAX);
  hca_cmd_steer_ = torque;
  hca_cmd_enabled_ = cmd.enabled() && (torque != 0);
  hca_cmd_ts_ms_ = utils::getCurrentTimestamp();
}

void PandaService::longPlanCallback(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_long_plan())
    return;
  const auto& lp = msg.long_plan();
  long_v_target_ = lp.v_target();
  have_long_plan_ = true;
  long_plan_ts_ms_ = utils::getCurrentTimestamp();
}

volkswagen::CruiseButtonCmd PandaService::computeCruiseButtons(const volkswagen::CarStateView& cs)
{
  volkswagen::CruiseButtonCmd cmd;
  if (!config_.cruise_buttons_enabled)
    return cmd;

  constexpr int64_t kLongPlanTimeoutMs = 500;
  const int64_t now = utils::getCurrentTimestamp();
  const bool plan_fresh = have_long_plan_ && (now - long_plan_ts_ms_) <= kLongPlanTimeoutMs;

  if (cs.cruiseEngaged && !cruise_was_engaged_) {
    cruise_v_set_ = std::max(0.0, static_cast<double>(cs.vEgo));
    cruise_hold_tip_up_ = cruise_hold_tip_down_ = false;
    cruise_cooldown_until_ms_ = 0;
    LOGI("cruise engage: latch v_set=%.1f km/h", cruise_v_set_ * 3.6);
  }
  if (!cs.cruiseEngaged) {
    cruise_was_engaged_ = false;
    cruise_hold_tip_up_ = cruise_hold_tip_down_ = false;
    return cmd;
  }
  cruise_was_engaged_ = true;

  if (cs.gasPressed || cs.brakePressed || !cs.graStock.valid) {
    cruise_hold_tip_up_ = cruise_hold_tip_down_ = false;
    return cmd;
  }

  if (cruise_hold_tip_up_ || cruise_hold_tip_down_) {
    cmd.send = true;
    cmd.tip_up = cruise_hold_tip_up_;
    cmd.tip_down = cruise_hold_tip_down_;
    if (cs.graStock.counter() != cruise_gra_cnt_at_arm_) {
      cruise_hold_tip_up_ = cruise_hold_tip_down_ = false;
      cruise_cooldown_until_ms_ = now + config_.cruise_tip_cooldown_ms;
    }
    return cmd;
  }

  if (now < cruise_cooldown_until_ms_ || !plan_fresh)
    return cmd;

  const double err = long_v_target_ - cruise_v_set_;
  if (std::abs(err) < config_.cruise_deadband_ms)
    return cmd;

  cruise_gra_cnt_at_arm_ = cs.graStock.counter();
  cmd.send = true;
  if (err > 0) {
    cruise_hold_tip_up_ = true;
    cmd.tip_up = true;
    cruise_v_set_ += config_.cruise_tip_step_ms;
  } else {
    cruise_hold_tip_down_ = true;
    cmd.tip_down = true;
    cruise_v_set_ = std::max(0.0, cruise_v_set_ - config_.cruise_tip_step_ms);
  }
  return cmd;
}

void PandaService::carControllerCallback()
{
  if (!panda_ || !panda_->connected() || !panda_->comms_healthy())
    return;
  using C = volkswagen::MqbSafetyConstants;
  if (safety_.lastSafetyMode() != C::kVolkswagen || !safety_.lastIgnition())
    return;

  const int64_t now = utils::getCurrentTimestamp();
  const bool cmd_fresh = hca_cmd_ts_ms_ > 0 && (now - hca_cmd_ts_ms_) <= kHcaCmdTimeoutMs;

  const auto cs = decoder_.toCarStateView();

  volkswagen::CarControl cc;
  cc.latActive = cmd_fresh && hca_cmd_enabled_ && safety_.lastControlsAllowed();
  if (cmd_fresh)
    cc.actuators.steerTorqueCNm = hca_cmd_steer_;
  cc.hud.leftLaneVisible = true;
  cc.hud.rightLaneVisible = true;
  cc.cruise = computeCruiseButtons(cs);

  auto frames = car_controller_.update(cc, cs);
  if (!frames.empty())
    panda_->can_send(frames);
}

PandaService::~PandaService()
{
  if (panda_ && panda_->connected()) {
    try {
      panda_->set_safety_model(volkswagen::MqbSafetyConstants::kNoOutput, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
      LOGE("Failed to set safe mode on shutdown");
    }
  }
}

void PandaService::reset() {}

void PandaService::publishCarState()
{
  if (!decoder_.consumeDirty())
    return;
  decoder_.state().set_timestamp(utils::getCurrentTimestamp());
  publish(adas::topics::kVehicleState, utils::createCarStateMessage(decoder_.state()));
}

void PandaService::pandaRxCallback()
{
  if (!panda_ || !panda_->connected())
    return;

  try {
    std::vector<can_frame> raw;
    if (!panda_->can_receive(raw) || raw.empty())
      return;

    std::vector<can_frame> filtered;
    for (const auto& frame : raw) {
      if (!volkswagen::isAllowedMqbRxAddress(frame.address))
        continue;
      filtered.push_back(frame);
      decoder_.updateFromFrame(frame);
    }

    if (!filtered.empty())
      publish(adas::topics::kCanRx, utils::createCANMessage(filtered));
    publishCarState();
  } catch (const std::exception& e) {
    LOGE("Exception in pandaRxCallback(): %s", e.what());
  }
}

void PandaService::pandaStateCallback()
{
  if (!panda_ || !panda_->connected())
    return;

  try {
    auto health = panda_->get_state();
    if (!health)
      return;

    volkswagen::SafetyLogContext log;
    log.last_tsk_status = decoder_.lastTskStatus();
    log.cruise_engaged = decoder_.state().cruise_engaged();
    log.brake_pressed = decoder_.state().brake_pressed();
    log.gas_pressed = decoder_.state().gas_pressed();
    log.eps_hca_status = decoder_.epsHcaStatus();
    log.apply_steer_last = car_controller_.applySteerLast();
    log.hca_cmd_steer = hca_cmd_steer_;
    log.lat_cmd_active = hca_cmd_enabled_ && safety_.lastControlsAllowed();
    log.ldw_valid = decoder_.ldwStock().valid;

    auto out = safety_.tick(*panda_, *health, utils::getCurrentTimestamp(), log);
    if (out)
      publish(adas::topics::kPandaHealth, utils::createHealthMessage(*out));
  } catch (const std::exception& e) {
    LOGE("Exception in pandaStateCallback(): %s", e.what());
  }
}
