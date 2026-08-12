#include "adas/lateral/fp_planner.hpp"

#include <algorithm>
#include <cmath>

#include "adas/utils/path_lateral_state.h"

namespace adas {
namespace lateral {
FpPlanner::FpPlanner(Config config) : cfg_(std::move(config))
{
  KappaSolverConfig k;
  k.wheelbase_m = cfg_.wheelbase_m;
  k.max_lateral_jerk = cfg_.max_lateral_jerk;
  k.steering_rate_weight = cfg_.steering_rate_weight;
  k.steer_delay_s = cfg_.steer_delay_s;
  kappa_ = makeKappaSolver(cfg_.solver, k);
}

const char* FpPlanner::solverName() const { return kappa_->name(); }

void FpPlanner::reset()
{
  kappa_->reset();
  last_steer_rad_ = 0.0;
  have_prev_ = false;
}

double FpPlanner::steerRaw(double kappa, double speed_mps, const VehicleParams& vehicle) const
{
  const double Lf = cfg_.Lf > 1e-3 ? cfg_.Lf : cfg_.wheelbase_m;
  return -steerFromCurvature(curvatureWithRoll(kappa, speed_mps, vehicle, cfg_.roll_compensation), speed_mps, Lf,
                             slipFactorOrZero(vehicle));
}

Output FpPlanner::update(const Input& in)
{
  Output out;
  out.controller = "fp";
  const auto& poly = in.polyline_ego;
  out.dbg.speed_mps = in.speed_mps;
  out.dbg.n_points = static_cast<int>(poly.size());

  const auto lat = estimatePathLateralState(poly);
  out.cte_m = lat.cte_m;
  out.epsi_rad = lat.epsi_rad;
  out.curvature = lat.kappa;
  out.dbg.mpc_kappa_path = lat.kappa;
  out.target_x = lat.cte_m;
  out.target_y = lat.epsi_rad;

  if (poly.size() < 4) {
    out.status = "no_polyline";
    return out;
  }

  const double Lf = cfg_.Lf > 1e-3 ? cfg_.Lf : cfg_.wheelbase_m;
  const double yaw = in.have_chassis ? in.yaw_rate : 0.0;

  double kappa_cmd = 0.0;
  double kappa_rate = 0.0;
  if (!kappa_->solve(in, poly, yaw, Lf, kappa_cmd, kappa_rate)) {
    out.status = "bad_fit";
    return out;
  }

  out.curvature = kappa_cmd;
  out.dbg.mpc_kappa_used = kappa_cmd;
  out.dbg.mpc_dkappa_ds = kappa_rate;

  out.steer_rad = steerRaw(kappa_cmd, in.speed_mps, in.vehicle);
  out.dbg.mpc_delta_vp_rad = out.steer_rad;
  const double lim = maxSteerRad(in.speed_mps, cfg_.limits);
  out.max_steer_rad = lim;
  out.dbg.mpc_max_steer_rad = lim;
  if (lim > 1e-6)
    out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
  out.dbg.mpc_delta_clamped_rad = out.steer_rad;

  if (cfg_.steer_slew_limit_deg > 1e-9 && have_prev_) {
    const double ceil = cfg_.steer_slew_limit_deg * M_PI / 180.0;
    const double d = std::clamp(out.steer_rad - last_steer_rad_, -ceil, ceil);
    out.steer_rad = last_steer_rad_ + d;
  }
  last_steer_rad_ = out.steer_rad;
  have_prev_ = true;

  out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
  out.has_target = true;
  out.status = "ok";
  return out;
}

std::optional<FpPlanner::Recompute> FpPlanner::recomputeSteer(double speed_mps, double frame_dt_s,
                                                              const VehicleParams& vehicle)
{
  const auto k = kappa_->curvatureAtSpeed(speed_mps, frame_dt_s);
  if (!k)
    return std::nullopt;

  double steer = steerRaw(*k, speed_mps, vehicle);
  const double lim = maxSteerRad(speed_mps, cfg_.limits);
  if (lim > 1e-6)
    steer = std::clamp(steer, -lim, lim);

  if (cfg_.steer_slew_limit_deg > 1e-9 && have_prev_) {
    const double ceil = cfg_.steer_slew_limit_deg * M_PI / 180.0;
    steer = last_steer_rad_ + std::clamp(steer - last_steer_rad_, -ceil, ceil);
  }
  last_steer_rad_ = steer;
  have_prev_ = true;

  Recompute r;
  r.steer_rad = steer;
  r.curvature = *k;
  r.max_steer_rad = lim;
  return r;
}

}  // namespace lateral
}  // namespace adas
