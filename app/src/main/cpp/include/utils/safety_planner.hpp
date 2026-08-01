#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace adas {
namespace safety {

enum class Warning : std::uint8_t {
  None = 0,
  FCW = 1,
  AEB = 2,
  LLDW = 3,
  RLDW = 4,
};

struct LongitudinalConfig {
  double speed_limit_ms = 27.778;
  double max_accel_ms2 = 1.5;
  double comfortable_decel_ms2 = 3.0;
  double time_headway_s = 1.5;
  double min_gap_m = 2.0;
  double free_road_exponent = 4.0;
  double friction_mu = 0.5;
  double gravity_ms2 = 9.81;
};

struct SafetyPlannerConfig {
  double speed_limit_ms = 27.778;
  double free_road_gap_m = 9999.0;
  double front_bumper_offset_m = 1.5;

  double fcw_ttc_s = 2.5;
  double aeb_ttc_s = 1.4;
  double fcw_decel_ms2 = 3.5;
  double aeb_decel_ms2 = 5.5;
  double warn_min_speed_ms = 3.0;
  double min_closing_speed_ms = 0.5;
  double lead_prob_thresh = 0.5;
  double lead_max_offset_m = 2.0;
  double standstill_gap_m = 2.0;

  double cte_ldw_threshold_m = 0.5;
  double cte_ldw_hard_m = 0.8;
  double ldw_min_speed_ms = 12.5;
  double ldw_min_outward_rate_ms = 0.05;
  bool ldw_suppress_on_driver_steer = true;
  bool ldw_suppress_on_blinker = true;
  bool ldw_require_lane_lines = true;

  LongitudinalConfig longitudinal{};
};

struct LateralState {
  double cte_m = 0.0;
  double epsi_rad = 0.0;
  double kappa = 0.0;
  double cte_rate_ms = 0.0;
  bool valid = false;
  bool lane_anchored = false;
};

struct CipoState {
  bool present = false;
  double speed_ms = 0.0;
  double gap_m = 0.0;
  double offset_m = 0.0;
};

struct PlannerInput {
  LateralState lateral{};
  double ego_speed_ms = 0.0;
  CipoState cipo{};
  bool driver_steering = false;
  bool left_blinker = false;
  bool right_blinker = false;
};

struct ThreatState {
  bool valid = false;
  double ttc_s = 0.0;
  double a_req_ms2 = 0.0;
  double closing_ms = 0.0;
};

struct SafetyPlan {
  double acceleration_ms2 = 0.0;
  ThreatState threat{};
  std::vector<Warning> warnings{};
};

inline double computeIdmAccel(const LongitudinalConfig& cfg, double kappa, double ego_speed_ms, bool has_cipo,
                              double cipo_speed_ms, double cipo_gap_m)
{
  const double abs_kappa = std::abs(kappa);
  const double curv_speed_limit_ms =
      abs_kappa > 1e-6 ? std::sqrt(cfg.friction_mu * cfg.gravity_ms2 / abs_kappa) : cfg.speed_limit_ms;
  const double speed_limit_ms = std::min(cfg.speed_limit_ms, curv_speed_limit_ms);

  const double delta_v = ego_speed_ms - cipo_speed_ms;
  const double dynamic_term =
      (ego_speed_ms * delta_v) / (2.0 * std::sqrt(cfg.max_accel_ms2 * cfg.comfortable_decel_ms2));
  const double s_star = cfg.min_gap_m + std::max(0.0, ego_speed_ms * cfg.time_headway_s + dynamic_term);

  const double gap_m = std::max(0.5, cipo_gap_m);
  const double free_road_term = std::pow(ego_speed_ms / std::max(0.1, speed_limit_ms), cfg.free_road_exponent);
  const double interaction_term = has_cipo ? std::pow(s_star / gap_m, 2.0) : 0.0;

  return cfg.max_accel_ms2 * (1.0 - free_road_term - interaction_term);
}

inline bool leadInPath(const SafetyPlannerConfig& cfg, const LateralState& lateral, double gap_m, double lead_offset_m)
{
  const double d = std::max(0.0, gap_m);
  const double path_y = lateral.valid ? (lateral.cte_m + 0.5 * lateral.kappa * d * d) : 0.0;
  return std::abs(lead_offset_m - path_y) <= cfg.lead_max_offset_m;
}

inline ThreatState computeThreat(const SafetyPlannerConfig& cfg, const PlannerInput& input)
{
  ThreatState t;
  if (!input.cipo.present || input.ego_speed_ms < cfg.warn_min_speed_ms)
    return t;
  if (!leadInPath(cfg, input.lateral, input.cipo.gap_m, input.cipo.offset_m))
    return t;

  const double closing = input.ego_speed_ms - input.cipo.speed_ms;
  if (closing < cfg.min_closing_speed_ms)
    return t;

  const double gap = std::max(0.1, input.cipo.gap_m - cfg.standstill_gap_m);
  t.valid = true;
  t.closing_ms = closing;
  t.ttc_s = gap / closing;
  t.a_req_ms2 = closing * closing / (2.0 * gap);
  return t;
}

inline SafetyPlan computeSafetyPlan(const SafetyPlannerConfig& cfg, const PlannerInput& input)
{
  LongitudinalConfig long_cfg = cfg.longitudinal;
  long_cfg.speed_limit_ms = cfg.speed_limit_ms;

  const bool has_cipo = input.cipo.present;
  const double cipo_speed_ms = has_cipo ? input.cipo.speed_ms : cfg.speed_limit_ms;
  const double cipo_gap_m = has_cipo ? input.cipo.gap_m : cfg.free_road_gap_m;
  const double kappa = input.lateral.valid ? input.lateral.kappa : 0.0;

  SafetyPlan plan;
  plan.acceleration_ms2 = computeIdmAccel(long_cfg, kappa, input.ego_speed_ms, has_cipo, cipo_speed_ms, cipo_gap_m);
  plan.threat = computeThreat(cfg, input);

  const bool ldw_allowed = input.lateral.valid && input.ego_speed_ms >= cfg.ldw_min_speed_ms &&
                           !(cfg.ldw_suppress_on_driver_steer && input.driver_steering) &&
                           (!cfg.ldw_require_lane_lines || input.lateral.lane_anchored);
  if (ldw_allowed) {
    const double cte = input.lateral.cte_m;
    const double rate = input.lateral.cte_rate_ms;
    const bool drifting_left = cte < -cfg.cte_ldw_threshold_m && rate < -cfg.ldw_min_outward_rate_ms;
    const bool drifting_right = cte > cfg.cte_ldw_threshold_m && rate > cfg.ldw_min_outward_rate_ms;
    const bool left_ok = !(cfg.ldw_suppress_on_blinker && input.left_blinker);
    const bool right_ok = !(cfg.ldw_suppress_on_blinker && input.right_blinker);
    if (left_ok && (drifting_left || cte < -cfg.cte_ldw_hard_m))
      plan.warnings.push_back(Warning::LLDW);
    if (right_ok && (drifting_right || cte > cfg.cte_ldw_hard_m))
      plan.warnings.push_back(Warning::RLDW);
  }

  if (plan.threat.valid) {
    const bool aeb = plan.threat.ttc_s <= cfg.aeb_ttc_s || plan.threat.a_req_ms2 >= cfg.aeb_decel_ms2;
    const bool fcw = plan.threat.ttc_s <= cfg.fcw_ttc_s || plan.threat.a_req_ms2 >= cfg.fcw_decel_ms2;
    if (aeb)
      plan.warnings.push_back(Warning::AEB);
    else if (fcw)
      plan.warnings.push_back(Warning::FCW);
  }

  return plan;
}

class WarningLatch {
public:
  WarningLatch(int set_frames = 3, int hold_frames = 10) : set_frames_(set_frames), hold_frames_(hold_frames) {}

  bool update(bool raw)
  {
    if (raw) {
      quiet_ = 0;
      if (++asked_ >= set_frames_)
        active_ = true;
    } else {
      asked_ = 0;
      if (active_ && ++quiet_ >= hold_frames_)
        active_ = false;
    }
    return active_;
  }

  bool active() const { return active_; }

  void reset()
  {
    asked_ = 0;
    quiet_ = 0;
    active_ = false;
  }

private:
  int set_frames_;
  int hold_frames_;
  int asked_ = 0;
  int quiet_ = 0;
  bool active_ = false;
};

}  // namespace safety
}  // namespace adas
