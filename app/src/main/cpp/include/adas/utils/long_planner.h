#pragma once

#include "adas/utils/lane_path.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "adas/utils/curvature_preview.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace longplan {
/** Longitudinal plan for a car that cannot brake.
 *
 *  A Golf 7 Highline has plain cruise control: no radar, no ACC, therefore no brake-by-wire. The
 *  only actuator is the cruise set speed (`Panda::computeCruiseButtons` taps GRA +/−), and
 *  lowering it closes the throttle and nothing more. Every number here follows from that: the plan
 *  may ask for as much acceleration as the engine gives, but only as much deceleration as coasting
 *  gives, and anything beyond that has to become a message to the driver instead of a command.
 *
 *  Split out of `LongPlan` so the rules can be tested without a middleware instance, the
 *  same way `safety_planner.hpp` is split out of `SafetyWarn`. */
struct Config {
  double lead_prob_thresh = 0.5;  ///< Model confidence needed before a lead is believed.
  double t_follow = 1.5;          ///< Desired time gap to the lead [s].
  double min_gap_m = 4.0;         ///< Standstill gap [m].
  double a_max = 1.2;             ///< Acceleration the plan may request [m/s^2].
  double a_min = -2.5;            ///< Deceleration the plan may request [m/s^2].
  double kp_gap = 0.35;           ///< Gain on gap error.
  double kp_v = 0.4;              ///< Gain on speed error.

  double curv_a_lat_max = 1.8;     ///< Lateral acceleration allowed in a curve [m/s^2]; caps speed before it.
  double curv_preview_s = 4.0;     ///< How far ahead the curve is looked for [s].
  double curv_min_speed_ms = 8.0;  ///< Below this speed curvature does not limit the plan [m/s].
  double curv_v_floor_ms = 8.0;    ///< Floor the curve limit will not push speed below [m/s].

  double a_coast_ms2 = -0.30;  ///< Deceleration from lifting off alone [m/s^2] — all this car offers without brakes.

  double lead_max_offset_m = 2.0;  ///< Lateral offset beyond which a lead is in another lane [m].

  double lead_min_speed_ms = 2.0;  ///< Below this the lead is treated as stationary [m/s].

  bool plan_v_enabled = false;  ///< Follow the model's own speed plan instead of the rule-based one.

  LanePathConfig lane_path{};  ///< How lane lines become a path, for the curve preview.
  double steer_ratio = 15.7;   ///< Steering ratio, to turn the wheel angle into a road-wheel angle.
};

struct LeadState {
  double prob = 0.0;
  double d_rel = 0.0;
  double y_rel = 0.0;
  double v_lead = 0.0;
};

struct PlanVState {
  bool valid = false;
  double v_plan = 0.0;
  bool pose_valid = false;
  double pose_vx = 0.0;
};

struct Input {
  double v_ego = 0.0;
  LeadState lead{};
  PlanVState plan_v{};
  const std::vector<Vec2>* path = nullptr;
};

struct Plan {
  double v_target = 0.0;
  double a_target = 0.0;
  double v_curv = 0.0;
  double kappa_ahead = 0.0;
  bool has_lead = false;
  bool lead_in_lane = true;
  std::string source = "hold";
  std::string status = "ok";
};

inline constexpr double kCoastHorizonS = 3.0;

/** Lateral position of our own path at `x_m`, linearly interpolated. Outside the polyline we hold
 *  the nearest end rather than extrapolate: a fit run past its data is what produced the phantom
 *  offsets the arc work spent two runs chasing. */
inline double pathYAt(const std::vector<Vec2>& path, double x_m)
{
  if (path.empty())
    return 0.0;
  if (x_m <= path.front().x())
    return path.front().y();
  if (x_m >= path.back().x())
    return path.back().y();
  for (size_t i = 1; i < path.size(); ++i) {
    const double x0 = path[i - 1].x();
    const double x1 = path[i].x();
    if (x_m <= x1) {
      const double span = x1 - x0;
      const double t = span > 1e-6 ? (x_m - x0) / span : 0.0;
      return path[i - 1].y() + t * (path[i].y() - path[i - 1].y());
    }
  }
  return path.back().y();
}

inline Plan compute(const Config& cfg, const Input& in)
{
  Plan out;
  const double v_ego = std::max(0.0, in.v_ego);
  out.v_target = v_ego;
  out.a_target = 0.0;

  const bool have_path = in.path != nullptr && in.path->size() >= 3;
  const auto& lead = in.lead;

  if (lead.prob >= cfg.lead_prob_thresh && have_path)
    out.lead_in_lane = std::abs(lead.y_rel - pathYAt(*in.path, lead.d_rel)) <= cfg.lead_max_offset_m;

  if (lead.prob >= cfg.lead_prob_thresh && lead.d_rel > 1.0 && lead.d_rel < 120.0 && out.lead_in_lane &&
      lead.v_lead >= cfg.lead_min_speed_ms) {
    out.has_lead = true;
    out.source = "lead";
    const double gap_des = std::max(cfg.min_gap_m, cfg.t_follow * v_ego);
    const double gap_err = lead.d_rel - gap_des;
    const double v_rel = lead.v_lead - v_ego;
    out.a_target = cfg.kp_gap * gap_err + 0.5 * v_rel;
    out.v_target = std::max(0.0, v_ego + out.a_target * kCoastHorizonS);
    if (out.a_target < 0.0)
      out.v_target = std::max(out.v_target, lead.v_lead);
  } else if (cfg.plan_v_enabled && in.plan_v.valid) {
    out.source = "plan_v";
    const double v_ref = in.plan_v.pose_valid ? 0.5 * (in.plan_v.v_plan + in.plan_v.pose_vx) : in.plan_v.v_plan;
    out.a_target = cfg.kp_v * (v_ref - v_ego);
    out.v_target = std::max(0.0, v_ref);
  } else {
    out.source = "hold";
  }

  if (have_path && v_ego >= cfg.curv_min_speed_ms) {
    const double from_m = std::max(5.0, 0.5 * v_ego);
    const double to_m = std::max(from_m + 15.0, cfg.curv_preview_s * v_ego);
    out.kappa_ahead = maxCurvatureAhead(*in.path, from_m, to_m);
    out.v_curv = std::max(cfg.curv_v_floor_ms, curvatureSpeedLimit(out.kappa_ahead, cfg.curv_a_lat_max, v_ego));
    if (out.v_curv < out.v_target - 0.1) {
      out.v_target = out.v_curv;
      out.a_target = std::min(out.a_target, cfg.kp_v * (out.v_curv - v_ego));
      out.source += "+curv";
    }
  }

  if (out.a_target < cfg.a_coast_ms2 - 1e-9) {
    out.status = "brake_needed";
    out.v_target = std::max(out.v_target, v_ego + cfg.a_coast_ms2 * kCoastHorizonS);
  }
  out.a_target = std::clamp(out.a_target, std::max(cfg.a_min, cfg.a_coast_ms2), cfg.a_max);
  return out;
}

}  // namespace longplan
}  // namespace adas
