#include "adas/lateral/vp_planner.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/path_lateral_state.h"

namespace adas {
namespace lateral {
void VpPlanner::reset()
{
  kappa_rate_.reset();
  kappa_ema_init_ = false;
  epsi_ema_init_ = false;
  cte_ema_init_ = false;
  have_prev_ = false;
  last_steer_rad_ = 0.0;
}

double VpPlanner::emaAlpha(double alpha_at_nominal, double frame_dt_s) const
{
  return lateral::emaAlpha(alpha_at_nominal, cfg_.vision_nominal_dt_s, frame_dt_s);
}

Output VpPlanner::update(const Input& in)
{
  const auto& poly = in.polyline_ego;
  Output out;
  out.controller = "mpc";
  out.dbg.speed_mps = in.speed_mps;
  out.dbg.n_points = static_cast<int>(poly.size());

  const auto lat = estimatePathLateralState(poly);
  out.cte_m = lat.cte_m;
  out.epsi_rad = lat.epsi_rad;
  out.curvature = lat.kappa;
  out.dbg.mpc_kappa_path = lat.kappa;
  out.target_x = lat.cte_m;
  out.target_y = lat.epsi_rad;

  if (!lat.valid) {
    out.status = lat.n_points < 5 ? "no_polyline" : "bad_fit";
    return out;
  }

  const double Lf = cfg_.Lf > 1e-3 ? cfg_.Lf : cfg_.vehicle.wheelbase_m;

  double cte = lat.cte_m;
  double epsi = lat.epsi_rad;
  const double cte_alpha = emaAlpha(cfg_.cte_ema_alpha, in.frame_dt_s);
  if (cte_alpha < 1.0 - 1e-6) {
    cte_ema_ = cte_ema_init_ ? (cte_alpha * cte + (1.0 - cte_alpha) * cte_ema_) : cte;
    cte_ema_init_ = true;
    cte = cte_ema_;
  }
  const double epsi_alpha = emaAlpha(cfg_.epsi_ema_alpha, in.frame_dt_s);
  if (epsi_alpha < 1.0 - 1e-6) {
    epsi_ema_ = epsi_ema_init_ ? (epsi_alpha * epsi + (1.0 - epsi_alpha) * epsi_ema_) : epsi;
    epsi_ema_init_ = true;
    epsi = epsi_ema_;
  }
  out.cte_m = cte;
  out.epsi_rad = epsi;
  out.target_x = cte;
  out.target_y = epsi;

  double kappa = lat.kappa;
  const double alpha = cfg_.kappa_yaw_blend;
  if (alpha > 1e-6 && in.have_chassis && in.speed_mps > cfg_.kappa_yaw_min_speed) {
    const double kappa_yaw = in.yaw_rate / in.speed_mps;
    out.dbg.mpc_kappa_yaw = kappa_yaw;
    kappa = (1.0 - alpha) * lat.kappa + alpha * kappa_yaw;
  }
  const double kappa_alpha = emaAlpha(cfg_.kappa_ema_alpha, in.frame_dt_s);
  if (kappa_alpha < 1.0 - 1e-6) {
    kappa_ema_ = kappa_ema_init_ ? (kappa_alpha * kappa + (1.0 - kappa_alpha) * kappa_ema_) : kappa;
    kappa_ema_init_ = true;
    kappa = kappa_ema_;
  }
  out.curvature = kappa;
  out.dbg.mpc_kappa_used = kappa;

  const double dkappa_ds = kappa_rate_.update(kappa, in.speed_mps, in.frame_dt_s);
  out.dbg.mpc_dkappa_ds = dkappa_ds;
  const auto kappa_sched = visionpilot::build_kappa_schedule(Lf, epsi, kappa, dkappa_ds, cfg_.solver.N);

  Eigen::VectorXd v_sched(static_cast<int>(cfg_.solver.N));
  for (int i = 0; i < static_cast<int>(cfg_.solver.N); ++i)
    v_sched[i] = in.speed_mps;

  Eigen::VectorXd state(3);
  state << cte, epsi, kappa;
  const auto deltas = mpc_.compute_steering(Lf, state, v_sched, kappa_sched);

  double delta_vp = 0.0;
  if (deltas.size() > 1)
    delta_vp = deltas[1];
  else if (!deltas.empty())
    delta_vp = deltas[0];

  out.steer_rad = -delta_vp;
  applySteerLimit(out, in.speed_mps, cfg_.limits);

  if (cfg_.rate_limit_deg > 1e-9 && have_prev_) {
    const double v_eff = std::max(in.speed_mps, cfg_.rate_min_speed);
    const double dkappa_max = cfg_.max_lateral_jerk / (v_eff * v_eff) * in.frame_dt_s;
    const double rate_ceil = cfg_.rate_limit_deg * M_PI / 180.0;
    const double rate = std::min(dkappa_max * Lf, rate_ceil);
    const double d = std::clamp(out.steer_rad - last_steer_rad_, -rate, rate);
    out.steer_rad = last_steer_rad_ + d;
  }
  last_steer_rad_ = out.steer_rad;
  have_prev_ = true;

  out.steer_norm = out.max_steer_rad > 1e-6 ? out.steer_rad / out.max_steer_rad : 0.0;
  out.has_target = true;
  out.status = "ok";
  return out;
}

}  // namespace lateral
}  // namespace adas
