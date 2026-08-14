#include "adas/services/platform.h"

#include <chrono>
#include <thread>

#include "adas/utils/adas_topics.h"
#include "adas/utils/logger.h"
#include "adas/utils/proto_convert.h"
#include "adas/platform/volkswagen/values.h"
#include "adas/platform/volkswagen/mqb_car_state_decoder.h"

namespace adas {
namespace services {
Platform::Platform(Config config) : config_(std::move(config)), ci_({config_.dbc_path, config_.speed_filter}) {}

void Platform::configure()
{
  using C = volkswagen::MqbSafetyConstants;
  safety_.setAlternativeExperience(C::kAltExpDisableDisengageOnGas | C::kAltExpAlka);

  panda_ = std::make_shared<::Panda>(config_.usb_fd, 0);
  ci_.init();

  subscribe<adas::proto::SteerCommand>(topics::kSteerCommand,
                                       [this](const adas::proto::SteerCommand& cmd) { cmd_ = cmd; });

  scheduleTimer(
      10, [this] { rxCallback(); }, "rx");
  scheduleTimer(
      10, [this] { txCallback(); }, "tx");
  scheduleTimer(
      100, [this] { stateCallback(); }, "state");

  LOGI("Platform: panda fd=%d, %s → %s / %s / %s", config_.usb_fd, topics::kSteerCommand, topics::kVehicleState,
       topics::kCanRx, topics::kPandaHealth);
}

void Platform::rxCallback()
{
  if (!panda_ || !panda_->connected())
    return;
  try {
    std::vector<can_frame> raw;
    if (!panda_->can_receive(raw) || raw.empty())
      return;

    std::vector<can_frame> filtered;
    filtered.reserve(raw.size());
    for (const auto& frame : raw) {
      if (volkswagen::isAllowedMqbRxAddress(frame.address))
        filtered.push_back(frame);
    }
    if (filtered.empty())
      return;

    const int64_t now_ms = nowMs();
    auto can_msg = utils::createCANMessage(filtered, now_ms);
    publish(topics::kCanRx, can_msg);
    if (ci_.update(can_msg, now_ms))
      publish(topics::kVehicleState, utils::createCarStateMessage(ci_.carState()));
  } catch (const std::exception& e) {
    LOGE("Platform rx: %s", e.what());
  }
}

void Platform::stateCallback()
{
  if (!panda_ || !panda_->connected())
    return;
  try {
    auto health = panda_->get_state();
    if (!health)
      return;

    volkswagen::SafetyLogContext log;
    log.last_tsk_status = ci_.lastTskStatus();
    log.eps_hca_status = ci_.epsHcaStatus();
    log.apply_steer_last = ci_.applySteerLast();
    log.cruise_engaged = ci_.carState().cruise_engaged();
    log.brake_pressed = ci_.carState().brake_pressed();
    log.gas_pressed = ci_.carState().gas_pressed();
    log.hca_cmd_steer = cmd_.torque_cnm();
    log.lat_cmd_active = cmd_.enabled();

    const int64_t now_ms = nowMs();
    auto out = safety_.tick(*panda_, *health, now_ms, log);
    if (out) {
      publish(topics::kPandaHealth, utils::createHealthMessage(*out, now_ms, safety_.lastIgnition(),
                                                               ci_.actuationAllowed(safety_.lastControlsAllowed())));
    }
  } catch (const std::exception& e) {
    LOGE("Platform state: %s", e.what());
  }
}

void Platform::txCallback()
{
  using C = volkswagen::MqbSafetyConstants;
  if (!panda_ || !panda_->connected())
    return;
  if (!safety_.lastIgnition() || !ci_.safetyModelOk(safety_.lastSafetyMode()))
    return;

  const auto cs = ci_.carStateView();
  volkswagen::CarControl cc;
  cc.latActive = cmd_.enabled() && cmd_.torque_cnm() != 0 && ci_.actuationAllowed(safety_.lastControlsAllowed());
  cc.actuators.steerTorqueCNm = cmd_.torque_cnm();
  cc.hud.leftLaneVisible = cmd_.hud_left_lane_visible();
  cc.hud.rightLaneVisible = cmd_.hud_right_lane_visible();
  cc.cruise = cruiseButtons(cs);

  try {
    auto frames = ci_.apply(cc, cs);
    if (!frames.empty())
      panda_->can_send(frames);
  } catch (const std::exception& e) {
    LOGE("Platform tx: %s", e.what());
  }
}

volkswagen::CruiseButtonCmd Platform::cruiseButtons(const volkswagen::CarStateView& cs)
{
  volkswagen::CruiseButtonCmd cmd;
  if (!config_.cruise_buttons_enabled)
    return cmd;

  const int64_t now = nowMs();
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

  if (now < cruise_cooldown_until_ms_ || !cs.graStock.valid || cmd_.cruise_intent() == 0)
    return cmd;

  cruise_gra_cnt_at_arm_ = cs.graStock.counter();
  cmd.send = true;
  cruise_hold_tip_up_ = cmd_.cruise_intent() == 1;
  cruise_hold_tip_down_ = cmd_.cruise_intent() == 2;
  cmd.tip_up = cruise_hold_tip_up_;
  cmd.tip_down = cruise_hold_tip_down_;
  return cmd;
}

Platform::~Platform()
{
  if (panda_ && panda_->connected()) {
    try {
      panda_->set_safety_model(volkswagen::MqbSafetyConstants::kNoOutput, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
      LOGE("Platform: failed to put the panda into safe mode on shutdown");
    }
  }
}

}  // namespace services
}  // namespace adas
