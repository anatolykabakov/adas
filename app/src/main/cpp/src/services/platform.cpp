#include "adas/services/platform.h"

#include <utility>

#include "adas/utils/adas_topics.h"
#include "adas/utils/logger.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
Platform::Platform(Config config) : config_(std::move(config)) {}

void Platform::configure()
{
  adas::platform::CarPlatformOptions opts;
  opts.dbc_path = config_.dbc_path;
  opts.speed_filter = config_.speed_filter;
  opts.cruise_buttons_enabled = config_.cruise_buttons_enabled;
  opts.cruise_tip_cooldown_ms = config_.cruise_tip_cooldown_ms;

  car_ = adas::platform::makeCarPlatform(config_.car_name, opts);
  if (!car_) {
    LOGE("Platform: no car platform for '%s' — the bus stays silent", config_.car_name.c_str());
    return;
  }
  car_->configureSafety();

  panda_ = std::make_shared<::Panda>(config_.usb_fd, 0);
  car_->init();

  subscribe<adas::proto::SteerCommand>(topics::kSteerCommand,
                                       [this](const adas::proto::SteerCommand& cmd) { cmd_ = cmd; });

  scheduleTimer(
      10, [this] { rxCallback(); }, "rx");
  scheduleTimer(
      10, [this] { txCallback(); }, "tx");
  scheduleTimer(
      100, [this] { stateCallback(); }, "state");

  LOGI("Platform: car=%s panda fd=%d, %s → %s / %s / %s", car_->name(), config_.usb_fd, topics::kSteerCommand,
       topics::kVehicleState, topics::kCanRx, topics::kPandaHealth);
}

void Platform::rxCallback()
{
  if (!car_ || !panda_ || !panda_->connected())
    return;
  try {
    std::vector<can_frame> raw;
    if (!panda_->can_receive(raw) || raw.empty())
      return;

    std::vector<can_frame> filtered;
    filtered.reserve(raw.size());
    for (const auto& frame : raw) {
      if (car_->isAllowedRxAddress(frame.address))
        filtered.push_back(frame);
    }
    if (filtered.empty())
      return;

    const int64_t now_ms = nowMs();
    auto can_msg = utils::createCANMessage(filtered, now_ms);
    publish(topics::kCanRx, can_msg);
    if (car_->update(can_msg, now_ms))
      publish(topics::kVehicleState, utils::createCarStateMessage(car_->carState()));
  } catch (const std::exception& e) {
    LOGE("Platform rx: %s", e.what());
  }
}

void Platform::stateCallback()
{
  if (!car_ || !panda_ || !panda_->connected())
    return;
  try {
    auto health = panda_->get_state();
    if (!health)
      return;

    const int64_t now_ms = nowMs();
    car_->setLastCommand(cmd_.torque_cnm(), cmd_.enabled());
    auto out = car_->safetyTick(*panda_, *health, now_ms);
    if (out)
      publish(topics::kPandaHealth,
              utils::createHealthMessage(*out, now_ms, car_->ignition(), car_->lateralActuationAllowed()));
  } catch (const std::exception& e) {
    LOGE("Platform state: %s", e.what());
  }
}

void Platform::txCallback()
{
  if (!car_ || !panda_ || !panda_->connected())
    return;
  if (!car_->ignition() || !car_->safetyModelOk())
    return;

  adas::platform::CarControl cc;
  cc.latActive = cmd_.enabled() && cmd_.torque_cnm() != 0 && car_->lateralActuationAllowed();
  cc.actuators.steerTorqueCNm = cmd_.torque_cnm();
  cc.hud.leftLaneVisible = cmd_.hud_left_lane_visible();
  cc.hud.rightLaneVisible = cmd_.hud_right_lane_visible();

  car_->setCruiseIntent(cmd_.cruise_intent(), nowMs());

  try {
    auto frames = car_->apply(cc);
    if (!frames.empty())
      panda_->can_send(frames);
  } catch (const std::exception& e) {
    LOGE("Platform tx: %s", e.what());
  }
}

Platform::~Platform()
{
  if (car_ && panda_ && panda_->connected())
    car_->enterSafeMode(*panda_);
}

}  // namespace services
}  // namespace adas
