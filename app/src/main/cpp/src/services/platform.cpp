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

  scheduleTimer(10, [this] { rxCallback(); }, "rx");
  scheduleTimer(10, [this] { txCallback(); }, "tx");
  scheduleTimer(100, [this] { stateCallback(); }, "state");

  LOGI("Platform: car=%s panda fd=%d, %s → %s / %s / %s", car_->name(), config_.usb_fd, topics::kSteerCommand,
       topics::kVehicleState, topics::kCanRx, topics::kPandaHealth);
}

void Platform::reseatPanda(int usb_fd)
{
  if (usb_fd < 0) {
    LOGW("Platform: reseat asked with fd=%d — ignored", usb_fd);
    return;
  }
  const int replaced = pending_fd_.exchange(usb_fd, std::memory_order_release);
  if (replaced >= 0 && replaced != usb_fd)
    LOGW("Platform: reseat fd=%d arrived before fd=%d was seated — the older one is dropped", usb_fd, replaced);
  LOGI("Platform: panda reseat queued, fd=%d", usb_fd);
}

void Platform::applyPendingPanda()
{
  const int fd = pending_fd_.exchange(-1, std::memory_order_acquire);
  if (fd < 0)
    return;
  if (!car_) {
    LOGE("Platform: reseat fd=%d ignored — there is no car platform to drive", fd);
    return;
  }

  // The old handle is dropped first, and without a safe-mode write: it speaks through a descriptor the
  // host has already closed, so the write would fail, and its failure must not cost us the reseat.
  // Dropping it before building the new one also keeps the two libusb contexts from overlapping.
  const int64_t started_ms = nowMs();
  panda_.reset();
  try {
    // This blocks: the constructor asks the board its type and health version, and each control
    // transfer retries five times with a 100 ms timeout. A board that answers nothing can hold this
    // thread for a second or so — acceptable, because the alternative is a board we cannot use at all.
    auto fresh = std::make_shared<::Panda>(fd, 0);
    if (!fresh->connected()) {
      LOGE("Platform: reseat fd=%d did not connect — staying without a panda", fd);
      return;
    }
    panda_ = std::move(fresh);
  } catch (const std::exception& e) {
    LOGE("Platform: reseat fd=%d threw (%s) — staying without a panda", fd, e.what());
    return;
  } catch (...) {
    LOGE("Platform: reseat fd=%d threw something unknown — staying without a panda", fd);
    return;
  }

  // A board we have never configured: power saving, alternative experience and safety model have to be
  // set again, and only the supervisor knows the order they need. The car's decode is untouched — that
  // is the whole reason for going this way instead of restarting.
  car_->resetPandaState();
  car_->configureSafety();
  config_.usb_fd = fd;
  const int count = reseats_.fetch_add(1, std::memory_order_relaxed) + 1;
  LOGI("Platform: panda seated on fd=%d in %lld ms (reseat #%d) — car state, filters and learners kept", fd,
       static_cast<long long>(nowMs() - started_ms), count);
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
  // Before the guards below, not after: the reseat exists for the case where `panda_` is dead or gone,
  // and a check that returns early would never let the new descriptor in.
  applyPendingPanda();
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
