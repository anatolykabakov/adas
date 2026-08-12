#pragma once

#include <algorithm>
#include <cmath>

#include "adas/lateral/types.hpp"
#include "adas/utils/vehicle_model.h"

namespace adas {
namespace lateral {
struct SteerLimits {
  double mpc_max_steer_deg = 25.0;
  double low_speed_steer_deg = 8.0;
  double steer_deg_per_mps = 0.5;
};

inline double slipFactorOrZero(const VehicleParams& v)
{
  if (!v.use_vehicle_model)
    return 0.0;
  VehicleModelParams p;
  p.wheelbase_m = v.wheelbase_m;
  p.mass_kg = v.mass_kg;
  p.center_to_front_frac = v.center_to_front_frac;
  p.tire_stiffness_factor = v.tire_stiffness_factor;
  return slipFactor(p);
}

inline double curvatureWithRoll(double kappa, double speed_mps, const VehicleParams& v, bool roll_compensation)
{
  if (!roll_compensation || !v.road_roll_valid)
    return kappa;
  return kappa - rollCompensationCurvature(v.road_roll_rad, speed_mps, slipFactorOrZero(v));
}

/** The steering ceiling grows with speed: at a standstill the rack fights tyre scrub, and at speed
 *  a large angle is both unnecessary and dangerous. */
inline double maxSteerRad(double speed_mps, const SteerLimits& lim)
{
  const double ceil_deg = std::min(lim.mpc_max_steer_deg, 25.0);
  const double lo_deg = std::clamp(lim.low_speed_steer_deg, 1.0, ceil_deg);
  const double slope = std::max(lim.steer_deg_per_mps, 0.0);
  const double lim_deg = std::clamp(lo_deg + slope * std::max(0.0, speed_mps), lo_deg, ceil_deg);
  return lim_deg * M_PI / 180.0;
}

/** The filter time constant is given as a fraction of a frame at the nominal vision rate; at a
 *  different rate the same coefficient would mean different smoothing. */
inline double emaAlpha(double alpha_at_nominal, double nominal_dt_s, double frame_dt_s)
{
  if (!(alpha_at_nominal < 1.0 - 1e-9) || alpha_at_nominal <= 0.0)
    return alpha_at_nominal;
  const double tau = -std::max(nominal_dt_s, 1e-3) / std::log(1.0 - alpha_at_nominal);
  return std::clamp(1.0 - std::exp(-frame_dt_s / tau), 1e-3, 1.0);
}

}  // namespace lateral
}  // namespace adas
