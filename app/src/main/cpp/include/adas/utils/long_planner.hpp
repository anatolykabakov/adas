#pragma once

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
  double lead_prob_thresh = 0.5;
  double t_follow = 1.5;
  double min_gap_m = 4.0;
  double a_max = 1.2;
  double a_min = -2.5;
  double kp_gap = 0.35;
  double kp_v = 0.4;

  bool curv_enabled = true;
  double curv_a_lat_max = 1.8;
  double curv_preview_s = 4.0;
  double curv_min_speed_ms = 8.0;
  double curv_v_floor_ms = 8.0;

  /** Strongest deceleration this car can actually produce without the brake pedal.
   *
   *  Measured on this car, not assumed: `bag_coast_decel.py` over the 2026-08-04 runs (four bags,
   *  125 intervals with both pedals up and speed falling monotonically) gives median −0.28 m/s²,
   *  p10 −0.51, strongest −0.89, nearly flat with speed (−0.25 at 0–5 m/s, −0.26 at 14–20, −0.44
   *  at 20–30). Flat with speed at that magnitude is road load, not engine drag: the DSG is
   *  sailing. The signal that would change that — `ACC_Freilauf_Anf`, "request DSG sailing" in
   *  `vw_mqb_2010.dbc` — lives in `ACC_07`, which only the radar-equipped ACC sends.
   *
   *  −0.30 is the median rounded toward zero: promise what the car delivers most of the time, not
   *  its best case on a downhill. */
  double a_coast_ms2 = -0.30;

  /** Require the lead to be in our lane, comparing its lateral position with our own path the way
   *  `SafetyPlanner` does. Without it the plan follows parked cars: on run 2026_08_06_00_36_42 a
   *  lead was used in 38 % of ticks and 31 % of those reported it nearly stationary, which drove
   *  `v_target` toward a stop and, with `cruise_buttons` on, chattered the set speed 715 times down
   *  and 690 up in 28 minutes. */
  double lead_max_offset_m = 2.0;

  /** A lead slower than this is an obstacle, not a car to follow. We cannot brake, so there is
   *  nothing useful to do about a stationary object except warn; matching it as a speed target only
   *  guarantees the set speed collapses. */
  double lead_min_speed_ms = 2.0;

  /** Use the model's planned speed as an absolute speed target. **Off, and measured off.**
   *
   *  `plan_v0 / v_ego` = 0.678 median over 18 916 frames of run 2026_08_06_00_36_42 (0.756 at
   *  5–10 m/s falling to 0.669 at 20–30): the model's plan velocity reads about a third low in our
   *  metric scale, worse than the camera odometry's own 0.888. Fed in as an absolute target it made
   *  the plan demand roughly 5 m/s below the current speed at all times, and it is not a bias one
   *  constant fixes — the ratio moves with speed.
   *
   *  With this off, free flow means "hold the current speed", and the only reasons to slow are a
   *  real in-lane lead and curvature ahead. */
  bool plan_v_enabled = false;
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
  const std::vector<Vec2>* path = nullptr;  // our own path in ego frame; null when unavailable
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

/** How far ahead `v_target` may run from the current speed once the plan wants more braking than
 *  coasting delivers.
 *
 *  Without this the plan publishes a speed the car will never reach and the cruise-button actuator
 *  reads the gap as "keep tipping down" — 715 down-tips in 28 minutes on run 2026_08_06_00_36_42.
 *  Three seconds is the reachable step: at the measured −0.30 m/s² it is 0.9 m/s, just past the
 *  0.7 m/s actuation deadband, so the target stays ahead of the car without running away. */
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

  // In our lane? Compare against our own path at the lead's distance, not against the vehicle axis
  // — otherwise our own lead car leaves the corridor as soon as the road bends.
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
    // Derive the target speed from the acceleration rather than jumping to the lead's speed. A lead
    // 100 m ahead and 5 m/s slower needs no action yet, but `v_target = v_lead` announced the full
    // deficit immediately, and the button actuator reads any deficit past its 0.7 m/s deadband as
    // "tip down" — which is how run 2026_08_06_00_36_42 collected 73 tips per minute. Flooring at
    // the lead's speed keeps us from aiming below the car we are following.
    out.v_target = std::max(0.0, v_ego + out.a_target * kCoastHorizonS);
    if (out.a_target < 0.0)
      out.v_target = std::max(out.v_target, lead.v_lead);
  } else if (cfg.plan_v_enabled && in.plan_v.valid) {
    out.source = "plan_v";
    const double v_ref = in.plan_v.pose_valid ? 0.5 * (in.plan_v.v_plan + in.plan_v.pose_vx) : in.plan_v.v_plan;
    out.a_target = cfg.kp_v * (v_ref - v_ego);
    out.v_target = std::max(0.0, v_ref);
  } else {
    // Free flow: hold the current speed. Nothing else is justified — see `plan_v_enabled`.
    out.source = "hold";
  }

  if (cfg.curv_enabled && have_path && v_ego >= cfg.curv_min_speed_ms) {
    const double from_m = std::max(5.0, 0.5 * v_ego);
    const double to_m = std::max(from_m + 15.0, cfg.curv_preview_s * v_ego);
    out.kappa_ahead = maxCurvatureAhead(*in.path, from_m, to_m);
    // Cap against `v_ego`, not against the already-reduced target. Passing the target made `v_curv`
    // a copy of it, so the published field claimed the limiter was active in 79 % of ticks of run
    // 2026_08_06_00_36_42 while it had actually changed the target in 0.5 %.
    out.v_curv = std::max(cfg.curv_v_floor_ms, curvatureSpeedLimit(out.kappa_ahead, cfg.curv_a_lat_max, v_ego));
    if (out.v_curv < out.v_target - 0.1) {
      out.v_target = out.v_curv;
      out.a_target = std::min(out.a_target, cfg.kp_v * (out.v_curv - v_ego));
      out.source += "+curv";
    }
  }

  // What the plan wants versus what the car can do. Anything past the measured coast envelope is
  // not a command, it is a request for the driver.
  if (out.a_target < cfg.a_coast_ms2 - 1e-9) {
    out.status = "brake_needed";
    out.v_target = std::max(out.v_target, v_ego + cfg.a_coast_ms2 * kCoastHorizonS);
  }
  out.a_target = std::clamp(out.a_target, std::max(cfg.a_min, cfg.a_coast_ms2), cfg.a_max);
  return out;
}

}  // namespace longplan
}  // namespace adas
