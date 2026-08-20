#pragma once

#include <algorithm>
#include <cmath>

#include "adas/lateral/types.h"
#include "adas/utils/vehicle_model.h"

namespace adas {
namespace lateral {
/** Speed-dependent ceiling on the steering angle. */
struct SteerLimits {
  double mpc_max_steer_deg = 25.0;   ///< Absolute ceiling on the angle [deg].
  double low_speed_steer_deg = 8.0;  ///< Angle allowed at a standstill [deg]; the limit ramps up from here.
  double steer_deg_per_mps = 0.5;    ///< Slope of that ramp [deg per m/s].
};

/// \return Slip factor, or 0 when the vehicle model is disabled.
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

/// \return \p kappa corrected for the road bank at \p speed_mps.
inline double curvatureWithRoll(double kappa, double speed_mps, const VehicleParams& v)
{
  if (!v.road_roll_valid)
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

/**
 * \brief Clamp an angle to the speed-dependent ceiling.
 * \param[in] steer_rad Requested road-wheel angle [rad].
 * \param[in] speed_mps Ego speed [m/s].
 * \param[in] lim Ramp that defines the ceiling.
 * \param[out] ceiling_out The ceiling that was applied; ignored when null.
 * \return The angle, clamped. A non-positive ceiling is treated as "no limit configured" and lets the angle through
 * rather than silently forcing it to zero.
 */
inline double clampToSteerLimit(double steer_rad, double speed_mps, const SteerLimits& lim,
                                double* ceiling_out = nullptr)
{
  const double ceiling = maxSteerRad(speed_mps, lim);
  if (ceiling_out)
    *ceiling_out = ceiling;
  return ceiling > 1e-6 ? std::clamp(steer_rad, -ceiling, ceiling) : steer_rad;
}

/**
 * \brief Apply the speed-dependent steering ceiling and record what it was.
 * \param[in,out] out Output whose `steer_rad` is clamped; `max_steer_rad` and the two debug fields are set.
 * \param[in] speed_mps Ego speed [m/s].
 * \param[in] lim Ramp that defines the ceiling.
 */
inline void applySteerLimit(Output& out, double speed_mps, const SteerLimits& lim)
{
  out.dbg.mpc_delta_vp_rad = out.steer_rad;
  out.steer_rad = clampToSteerLimit(out.steer_rad, speed_mps, lim, &out.max_steer_rad);
  out.dbg.mpc_max_steer_rad = out.max_steer_rad;
  out.dbg.mpc_delta_clamped_rad = out.steer_rad;
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
