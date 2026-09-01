#include "adas/longitudinal/long_planner.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/curvature_preview.h"

namespace adas {
namespace longitudinal {
namespace {
constexpr double kDegToRad = 0.017453292519943295;

double interpVec(double x, const std::vector<double>& xp, const std::vector<double>& fp)
{
  return interp(x, xp.data(), fp.data(), static_cast<int>(std::min(xp.size(), fp.size())));
}

/// Model path lateral position at x, linearly interpolated, held at the ends.
double pathYAt(const std::vector<Vec2>& path, double x_m)
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

}  // namespace

Planner::Planner(PlannerConfig cfg) : cfg_(std::move(cfg)), mpc_(cfg_.mpc) {}

void Planner::reset(double v_ego, double a_ego)
{
  kappa_filt_ = 0.0;
  v_desired_ = std::max(0.0, v_ego);
  a_desired_ = std::clamp(a_ego, -3.5, 2.0);
  mpc_.reset();
  initialized_ = true;
}

double Planner::maxAccel(double v_ego) const { return interpVec(v_ego, cfg_.a_cruise_max_bp, cfg_.a_cruise_max_v); }

std::array<double, 2> Planner::limitAccelInTurns(double v_ego, double steering_angle_deg,
                                                 std::array<double, 2> limits) const
{
  // The lateral acceleration the current wheel angle produces eats into a total budget that grows with
  // speed; whatever is left is the most the plan may ask for longitudinally.
  const double a_total_max = interpVec(v_ego, cfg_.a_total_max_bp, cfg_.a_total_max_v);
  const double a_y = v_ego * v_ego * steering_angle_deg * kDegToRad / (cfg_.steer_ratio * cfg_.wheelbase_m);
  const double a_x_allowed = std::sqrt(std::max(a_total_max * a_total_max - a_y * a_y, 0.0));
  return {limits[0], std::min(limits[1], a_x_allowed)};
}

std::optional<LeadTrajectory> Planner::processLead(const Lead& lead, double v_ego) const
{
  double x, v, a, tau;
  if (lead.valid) {
    x = lead.d_rel;
    v = lead.v_lead;
    a = lead.a_lead;
    tau = cfg_.lead_accel_tau;
  } else {
    // No lead: a fast one far away, so the MPC keeps the same structure and the cruise obstacle wins.
    x = 50.0;
    v = v_ego + 10.0;
    a = 0.0;
    tau = cfg_.lead_accel_tau;
  }
  // A lead closer than what a −3.5 m/s² stop would need is inside the plant's reach; clip it out.
  const double min_x_lead = ((v_ego + v) / 2.0) * (v_ego - v) / (3.5 * 2.0);
  x = std::max(x, min_x_lead);
  v = std::max(0.0, v);
  a = std::clamp(a, -10.0, 5.0);
  return extrapolateLead(x, v, a, tau);
}

PlannerOutput Planner::update(const PlannerInput& in)
{
  PlannerOutput out;
  out.lead_source = cfg_.lead_source;
  const double v_ego = std::max(0.0, in.v_ego);
  const auto& T = modelTimes();
  const auto& Tm = mpcTimes();

  if (!initialized_ || in.reset_state)
    reset(v_ego, in.a_ego);

  // The filtered desired speed follows the car with a 2 s time constant; the integration below moves it.
  const double alpha = cfg_.dt_s / (cfg_.v_filter_rc_s + cfg_.dt_s);
  v_desired_ = std::max(0.0, (1.0 - alpha) * v_desired_ + alpha * v_ego);

  out.v_cruise = in.v_cruise_mps;
  if (in.v_cruise_mps <= 0.0) {
    out.valid = false;
    out.status = "no_cruise";
    out.v_target = v_ego;
    a_desired_ = 0.0;
    mpc_.reset();
    return out;
  }

  // Curvature preview: the curve ahead is a speed the set speed must not exceed.
  const bool have_path = in.path != nullptr && in.path->size() >= 3;
  double v_cruise = std::clamp(in.v_cruise_mps, 0.0, cfg_.v_cruise_max_mps);
  if (cfg_.curv_limit_enabled && have_path && v_ego >= cfg_.curv_min_speed_ms) {
    const double from_m = std::max(5.0, 0.5 * v_ego);
    const double to_m = std::max(from_m + 15.0, cfg_.curv_preview_s * v_ego);
    // One long window and the median, not the 80th percentile of short ones: a quadratic through
    // 25 m of path with 0.15 m of scatter at 20 m reads as R ≈ 200 m on a straight; through 40 m it
    // reads as R ≈ 700 m and stays under the deadband.
    const double kappa_raw = maxCurvatureAhead(*in.path, from_m, to_m, 0.9 * (to_m - from_m), 0.5);
    // Low-passed: the fit through a jittery path throws spurious curvature every frame, and a cap that
    // flickers is a plan that brakes and accelerates for nothing.
    const double alpha_k = cfg_.dt_s / std::max(cfg_.dt_s, cfg_.curv_filter_tau_s);
    kappa_filt_ += alpha_k * (kappa_raw - kappa_filt_);
    out.kappa_ahead = kappa_filt_;
    if (kappa_filt_ > cfg_.curv_deadband) {
      // The cap is against the set speed, not the current one: a curve ahead lowers what we aim for,
      // it never turns the current speed into the ceiling.
      out.v_curv = std::max(cfg_.curv_v_floor_ms, curvatureSpeedLimit(kappa_filt_, cfg_.curv_a_lat_max, v_cruise));
      if (out.v_curv < v_cruise - 0.1)
        v_cruise = out.v_curv;
      else
        out.v_curv = 0.0;
    }
  } else {
    kappa_filt_ *= 0.9;
  }
  out.v_cruise = v_cruise;

  // Acceleration bounds: by speed, by the turn, and never excluding the acceleration we are already at.
  std::array<double, 2> limits{cfg_.a_cruise_min, maxAccel(v_ego)};
  limits = limitAccelInTurns(v_ego, in.steering_angle_deg, limits);
  limits[0] = std::min(limits[0], a_desired_ + 0.05);
  limits[1] = std::max(limits[1], a_desired_ - 0.05);

  // Leads: believed above the probability threshold and only when on our path.
  Lead l0 = in.lead0;
  Lead l1 = in.lead1;
  auto gate = [&](Lead& l) {
    if (cfg_.lead_source == "none") {
      l.valid = false;
      return;
    }
    if (!l.valid || l.prob < cfg_.lead_prob_thresh || l.d_rel <= 0.0 || l.d_rel > 150.0) {
      l.valid = false;
      return;
    }
    if (have_path && std::abs(l.y_rel - pathYAt(*in.path, l.d_rel)) > cfg_.lead_max_offset_m) {
      l.valid = false;
      out.lead_in_lane = false;
    }
  };
  gate(l0);
  gate(l1);
  out.has_lead = l0.valid;

  const bool prev_accel_constraint = !(in.reset_state || in.standstill);
  mpc_.setWeights(prev_accel_constraint);
  mpc_.setAccelLimits(limits[0], limits[1]);
  mpc_.setCurState(v_desired_, a_desired_);
  const bool ok = mpc_.update(processLead(l0, v_ego), processLead(l1, v_ego), v_cruise,
                              l0.valid && l0.prob > cfg_.lead_fcw_prob);
  out.solver_ok = ok;
  out.solver_iters = mpc_.iterations();
  out.solver_cost = mpc_.cost();
  if (!ok) {
    // The reset plan: hold the current speed. The next tick starts the solver from scratch.
    out.valid = true;
    out.status = "solver_reset";
    out.speeds.fill(v_desired_);
    out.accels.fill(0.0);
    out.jerks.fill(0.0);
    out.v_target = v_desired_;
    a_desired_ = 0.0;
    return out;
  }

  // Resample the MPC grid onto the model grid the control law reads.
  for (int i = 0; i < kControlN; ++i) {
    out.speeds[i] = interp(T[i], Tm.data(), mpc_.vSolution().data(), kMpcNodes);
    out.accels[i] = interp(T[i], Tm.data(), mpc_.aSolution().data(), kMpcNodes);
    out.jerks[i] = interp(T[i], Tm.data(), mpc_.jSolution().data(), kMpcN);
  }

  // Integrate the desired state forward by one tick, trapezoidal in acceleration.
  const double a_prev = a_desired_;
  a_desired_ = interp(cfg_.dt_s, T.data(), out.accels.data(), kControlN);
  v_desired_ += cfg_.dt_s * (a_desired_ + a_prev) / 2.0;

  out.valid = true;
  out.v_target = out.speeds[0];
  out.a_target = a_desired_;
  out.source = mpc_.source();
  if (out.v_curv > 0.0 && out.source == "cruise")
    out.source += "+curv";
  out.fcw = mpc_.crashCount() > 2 && !in.standstill;
  out.status = "ok";
  return out;
}

void CruiseSetpoint::tip(bool held, bool was_held, Press& p, int direction, const PlannerConfig& cfg, double now_s)
{
  if (v_cruise_ <= 0.0)
    return;
  const double lo = cfg.v_cruise_min_mps;
  const double hi = cfg.v_cruise_max_mps;
  auto clamp_kph = [&](double kph) { return std::clamp(kph / 3.6, lo, hi); };
  const double kph = v_cruise_ * 3.6;

  if (held && !was_held) {
    p.since_s = now_s;
    p.last_long_s = now_s;
    p.long_fired = false;
    return;
  }
  if (held) {
    // Held past the threshold: a long step now and every threshold after, each landing on the next
    // multiple of the long step in the direction of the press.
    if (now_s - p.since_s >= cfg.cruise_long_press_s && now_s - p.last_long_s >= cfg.cruise_long_press_s) {
      const double step = std::max(1.0, cfg.cruise_long_step_kph);
      const double target = direction > 0 ? (std::floor(kph / step + 1e-6) + 1.0) * step
                                          : (std::ceil(kph / step - 1e-6) - 1.0) * step;
      v_cruise_ = clamp_kph(target);
      p.last_long_s = now_s;
      p.long_fired = true;
    }
    return;
  }
  if (was_held && !p.long_fired)
    v_cruise_ = clamp_kph(kph + direction * cfg.cruise_short_step_kph);
}

double CruiseSetpoint::update(const Buttons& now, double v_ego, bool available, const PlannerConfig& cfg, double now_s)
{
  auto pressed = [&](bool cur, bool prev) { return cur && !prev; };
  const double lo = cfg.v_cruise_min_mps;
  const double hi = cfg.v_cruise_max_mps;
  auto round_kph = [](double v) { return std::round(v * 3.6) / 3.6; };

  if (!available) {
    if (v_cruise_ > 0.0)
      v_last_ = v_cruise_;
    v_cruise_ = 0.0;
  } else {
    if (pressed(now.set, prev_.set))
      v_cruise_ = std::clamp(round_kph(v_ego), lo, hi);
    else if (pressed(now.resume, prev_.resume))
      v_cruise_ = v_cruise_ > 0.0 ? v_cruise_ : (v_last_ > 0.0 ? v_last_ : std::clamp(round_kph(v_ego), lo, hi));
    tip(now.accel, prev_.accel, accel_, +1, cfg, now_s);
    tip(now.decel, prev_.decel, decel_, -1, cfg, now_s);
    // Cancel keeps the set speed for Resume; the panda is what stops the actuation.
  }
  prev_ = now;
  return v_cruise_;
}

}  // namespace longitudinal
}  // namespace adas
