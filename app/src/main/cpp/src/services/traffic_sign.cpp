#include "adas/services/traffic_sign.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/logger.h"
#include "adas/utils/protobuf_utils.h"

namespace adas {
namespace services {

void TrafficSign::configure()
{
  subscribe<ai::flow::adas::ZMQMessage>(topics::kTrafficDetections,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onDets(m); });
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  scheduleTimer(
      100, [this] { tick(); }, "tick");
  LOGI("TrafficSign: %s + chassis → %s (speed limit / TFL / overspeed)", topics::kTrafficDetections,
       topics::kTrafficVision);
}

void TrafficSign::reset()
{
  have_chassis_ = false;
  chassis_ = {};
  speed_limit_kmh_ = 0;
  speed_limit_label_.clear();
  speed_limit_ts_ms_ = 0;
  tfl_color_ = ai::flow::adas::TFL_UNKNOWN;
  tfl_conf_ = 0.f;
  tfl_ts_ms_ = 0;
  n_dets_ = 0;
  status_ = "reset";
}

void TrafficSign::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void TrafficSign::onDets(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_traffic_detections())
    return;
  const auto& td = msg.traffic_detections();
  const int64_t now = td.timestamp() > 0 ? td.timestamp() : utils::getCurrentTimestamp();
  n_dets_ = td.dets_size();
  status_ = n_dets_ > 0 ? "ok" : "no_dets";

  // Prefer highest-conf speed-limit sign and traffic light this frame.
  int best_limit = 0;
  float best_limit_conf = 0.f;
  std::string best_limit_label;
  ai::flow::adas::TrafficLightColor best_tfl = ai::flow::adas::TFL_UNKNOWN;
  float best_tfl_conf = 0.f;

  for (const auto& d : td.dets()) {
    if (d.speed_limit_kmh() > 0 && d.conf() >= config_.min_sign_conf && d.conf() >= best_limit_conf) {
      best_limit = d.speed_limit_kmh();
      best_limit_conf = d.conf();
      best_limit_label = d.label();
    }
    if (d.tfl_color() != ai::flow::adas::TFL_UNKNOWN && d.tfl_color() != ai::flow::adas::TFL_OFF &&
        d.conf() >= config_.min_tfl_conf && d.conf() >= best_tfl_conf) {
      best_tfl = d.tfl_color();
      best_tfl_conf = d.conf();
    }
  }

  if (best_limit > 0) {
    speed_limit_kmh_ = best_limit;
    speed_limit_label_ = best_limit_label;
    speed_limit_ts_ms_ = now;
  }
  if (best_tfl != ai::flow::adas::TFL_UNKNOWN) {
    tfl_color_ = best_tfl;
    tfl_conf_ = best_tfl_conf;
    tfl_ts_ms_ = now;
  }

  publishState(now);
}

void TrafficSign::tick()
{
  if (!have_chassis_)
    return;
  publishState(utils::getCurrentTimestamp());
}

void TrafficSign::publishState(int64_t now_ms)
{
  if (speed_limit_kmh_ > 0 && speed_limit_ts_ms_ > 0 && (now_ms - speed_limit_ts_ms_) > config_.speed_limit_hold_ms) {
    speed_limit_kmh_ = 0;
    speed_limit_label_.clear();
  }
  if (tfl_ts_ms_ > 0 && (now_ms - tfl_ts_ms_) > config_.tfl_hold_ms) {
    tfl_color_ = ai::flow::adas::TFL_UNKNOWN;
    tfl_conf_ = 0.f;
  }

  const double v_kmh = have_chassis_ ? chassis_.speed_mps * 3.6 : 0.0;
  bool overspeed = false;
  float over_by = 0.f;
  if (speed_limit_kmh_ > 0 && v_kmh > speed_limit_kmh_ + config_.overspeed_margin_kmh) {
    overspeed = true;
    over_by = static_cast<float>(v_kmh - speed_limit_kmh_);
  }

  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(now_ms);
  zmq.set_topic(topics::kTrafficVision);
  auto* s = zmq.mutable_traffic_vision();
  s->set_timestamp(now_ms);
  s->set_speed_limit_kmh(speed_limit_kmh_);
  s->set_speed_limit_age_ms(speed_limit_ts_ms_ > 0 ? static_cast<int>(now_ms - speed_limit_ts_ms_) : -1);
  s->set_v_ego_kmh(static_cast<float>(v_kmh));
  s->set_overspeed(overspeed);
  s->set_overspeed_kmh(over_by);
  s->set_tfl_color(tfl_color_);
  s->set_tfl_conf(tfl_conf_);
  s->set_tfl_age_ms(tfl_ts_ms_ > 0 ? static_cast<int>(now_ms - tfl_ts_ms_) : -1);
  s->set_status(status_);
  s->set_speed_limit_label(speed_limit_label_);
  s->set_n_dets(n_dets_);
  publish(topics::kTrafficVision, zmq);
}

}  // namespace services
}  // namespace adas
