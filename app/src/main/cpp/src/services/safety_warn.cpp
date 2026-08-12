#include "adas/services/safety_warn.h"

#include <algorithm>

#include "adas/utils/logger.h"
#include "adas/utils/math_utils.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
void SafetyWarn::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, config_.steer_ratio));
  });
  subscribe<adas::proto::LanePath>(
      topics::kVisionPath, [this](const adas::proto::LanePath& payload) { onPath(lanePathFromProto(payload)); });
  subscribe<adas::proto::ModelLongPlan>(topics::kVisionModelLong,
                                        [this](const adas::proto::ModelLongPlan& m) { onModelLong(m); });
  subscribe<adas::proto::SteerCommand>(topics::kSteerCommand,
                                       [this](const adas::proto::SteerCommand& m) { onSteer(m); });
  scheduleTimer(
      50, [this] { tick(); }, "tick");
  LOGI("SafetyWarn: path + model_long + chassis → %s (FCW/AEB/LDW, no actuation)", topics::kSafetyWarn);
}

void SafetyWarn::rebuildLatches()
{
  const int set_frames = std::max(1, config_.warn_set_frames);
  const int hold_frames = std::max(0, config_.warn_hold_frames);
  fcw_latch_ = safety::WarningLatch(set_frames, hold_frames);
  aeb_latch_ = safety::WarningLatch(set_frames, hold_frames);
  lldw_latch_ = safety::WarningLatch(set_frames, hold_frames);
  rldw_latch_ = safety::WarningLatch(set_frames, hold_frames);
}

void SafetyWarn::reset()
{
  have_chassis_ = false;
  have_lateral_ = false;
  have_model_ = false;
  chassis_ = {};
  lateral_ = {};
  model_ = {};
  cte_rate_ms_ = 0.0;
  lane_anchored_ = false;
  last_cte_m_ = 0.0;
  last_path_ts_us_ = 0;
  lat_active_ts_us_ = 0;
  rebuildLatches();
}

void SafetyWarn::onSteer(const adas::proto::SteerCommand& payload)
{
  if (payload.enabled())
    lat_active_ts_us_ = static_cast<int64_t>(now());
}

void SafetyWarn::onPath(const LanePathMsg& msg)
{
  const PathLateralState lat = estimatePathLateralState(msg.polyline);
  if (lat.valid && have_lateral_ && last_path_ts_us_ > 0 && msg.timestamp_us > last_path_ts_us_) {
    const double dt = static_cast<double>(msg.timestamp_us - last_path_ts_us_) * 1e-6;
    if (dt > 1e-3 && dt < 1.0) {
      constexpr double kAlpha = 0.3;
      const double raw = (lat.cte_m - last_cte_m_) / dt;
      cte_rate_ms_ = kAlpha * raw + (1.0 - kAlpha) * cte_rate_ms_;
    }
  }
  if (lat.valid) {
    last_cte_m_ = lat.cte_m;
    last_path_ts_us_ = msg.timestamp_us;
  }
  lateral_ = lat;
  have_lateral_ = lat.valid;
  lane_anchored_ = msg.lane_anchored;
}

void SafetyWarn::onModelLong(const adas::proto::ModelLongPlan& payload)
{
  model_ = payload;
  have_model_ = true;
}

void SafetyWarn::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void SafetyWarn::tick()
{
  if (!have_chassis_)
    return;

  const auto& cfg = config_.planner;
  safety::PlannerInput in;
  in.ego_speed_ms = std::max(0.0, chassis_.speed_mps);
  in.driver_steering = chassis_.steering_pressed;
  constexpr int64_t kLatActiveTimeoutUs = 250'000;
  in.lat_active = lat_active_ts_us_ > 0 && (static_cast<int64_t>(now()) - lat_active_ts_us_) <= kLatActiveTimeoutUs;
  in.left_blinker = chassis_.left_blinker;
  in.right_blinker = chassis_.right_blinker;
  if (have_lateral_) {
    in.lateral.cte_m = lateral_.cte_m;
    in.lateral.epsi_rad = lateral_.epsi_rad;
    in.lateral.kappa = lateral_.kappa;
    in.lateral.cte_rate_ms = cte_rate_ms_;
    in.lateral.lane_anchored = lane_anchored_;
    in.lateral.valid = true;
  }

  double lead_d = 0.0;
  double lead_v = 0.0;
  double lead_prob = 0.0;
  bool has_lead = false;
  if (have_model_) {
    const auto& lead = model_.lead0();
    lead_prob = lead.prob();
    lead_d = lead.d_rel() > 0 ? lead.d_rel() : (lead.x_size() > 0 ? lead.x(0) : 0.0);
    lead_v = lead.v_lead() != 0 ? lead.v_lead() : (lead.v_size() > 0 ? lead.v(0) : 0.0);
    if (lead_prob >= cfg.lead_prob_thresh && lead_d > 1.0 && lead_d < 150.0) {
      has_lead = true;
      in.cipo.present = true;
      in.cipo.speed_ms = lead_v;
      in.cipo.gap_m = std::max(0.5, lead_d - cfg.front_bumper_offset_m);
      in.cipo.offset_m = lead.y_rel();
    }
  }

  const safety::SafetyPlan plan = safety::computeSafetyPlan(cfg, in);

  bool raw_fcw = false, raw_aeb = false, raw_lldw = false, raw_rldw = false;
  for (const auto w : plan.warnings) {
    switch (w) {
      case safety::Warning::FCW:
        raw_fcw = true;
        break;
      case safety::Warning::AEB:
        raw_aeb = true;
        break;
      case safety::Warning::LLDW:
        raw_lldw = true;
        break;
      case safety::Warning::RLDW:
        raw_rldw = true;
        break;
      default:
        break;
    }
  }

  const bool fcw = fcw_latch_.update(raw_fcw);
  const bool aeb = aeb_latch_.update(raw_aeb);
  const bool lldw = lldw_latch_.update(raw_lldw);
  const bool rldw = rldw_latch_.update(raw_rldw);

  const int64_t ms = nowMs();
  publish(topics::kSafetyWarn,
          createSafetyWarn(
              in, plan, {lead_d, lead_v, lead_prob, has_lead, fcw, aeb, lldw, rldw, have_lateral_ ? "ok" : "no_path"},
              nowMs()));
}

}  // namespace services
}  // namespace adas
