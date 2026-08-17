#include "adas/services/traffic_sign.h"

#include "adas/utils/proto_convert.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/logger.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace services {
void TrafficSign::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, config_.steer_ratio));
  });
  subscribe<adas::proto::TrafficDetections>(topics::kTrafficDetections,
                                            [this](const adas::proto::TrafficDetections& m) { onDets(m); });
  scheduleTimer(
      100, [this] { tick(); }, "tick");
  LOGI("TrafficSign: %s + chassis → %s (speed limit / TFL / overspeed)", topics::kTrafficDetections,
       topics::kTrafficVision);
}

void TrafficSign::reset()
{
  have_chassis_ = false;
  chassis_ = {};
  state_.speed_limit_kmh = 0;
  state_.speed_limit_label.clear();
  state_.speed_limit_ts_ms = 0;
  state_.tfl_color = adas::proto::TFL_UNKNOWN;
  state_.tfl_conf = 0.f;
  state_.tfl_ts_ms = 0;
  state_.n_dets = 0;
  state_.status = "reset";
}

void TrafficSign::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void TrafficSign::onDets(const adas::proto::TrafficDetections& payload)
{
  const int64_t now = payload.timestamp() > 0 ? payload.timestamp() : nowMs();
  state_.n_dets = payload.dets_size();
  state_.status = state_.n_dets > 0 ? "ok" : "no_dets";

  int best_limit = 0;
  float best_limit_conf = 0.f;
  std::string best_limit_label;
  adas::proto::TrafficLightColor best_tfl = adas::proto::TFL_UNKNOWN;
  float best_tfl_conf = 0.f;

  for (const auto& d : payload.dets()) {
    if (d.speed_limit_kmh() > 0 && d.conf() >= config_.min_sign_conf && d.conf() >= best_limit_conf) {
      best_limit = d.speed_limit_kmh();
      best_limit_conf = d.conf();
      best_limit_label = d.label();
    }
    if (d.tfl_color() != adas::proto::TFL_UNKNOWN && d.tfl_color() != adas::proto::TFL_OFF &&
        d.conf() >= config_.min_tfl_conf && d.conf() >= best_tfl_conf) {
      best_tfl = d.tfl_color();
      best_tfl_conf = d.conf();
    }
  }

  if (best_limit > 0) {
    state_.speed_limit_kmh = best_limit;
    state_.speed_limit_label = best_limit_label;
    state_.speed_limit_ts_ms = now;
  }
  if (best_tfl != adas::proto::TFL_UNKNOWN) {
    state_.tfl_color = best_tfl;
    state_.tfl_conf = best_tfl_conf;
    state_.tfl_ts_ms = now;
  }
  publishTraffic(now);
}

void TrafficSign::tick()
{
  if (!have_chassis_)
    return;
  publishTraffic(nowMs());
}

void TrafficSign::publishTraffic(int64_t now_ms)
{
  const traffic::Config cfg{config_.speed_limit_hold_ms, config_.tfl_hold_ms, config_.overspeed_margin_kmh};
  traffic::expire(state_, now_ms, cfg);
  const auto assessment = traffic::assess(state_, have_chassis_ ? chassis_.speed_mps : 0.0, have_chassis_, now_ms, cfg);
  publish(topics::kTrafficVision, createTrafficVision(state_, assessment, now_ms));
}

}  // namespace services
}  // namespace adas
