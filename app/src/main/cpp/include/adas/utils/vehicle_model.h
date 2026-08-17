#pragma once

#include <algorithm>
#include <cmath>

namespace adas {
struct VehicleModelParams {
  double wheelbase_m = 2.636;           ///< Wheelbase [m].
  double center_to_front_frac = 0.45;   ///< Distance to the front axle as a fraction of the wheelbase.
  double mass_kg = 1533.0;              ///< Kerb mass plus a driver [kg].
  double tire_stiffness_factor = 0.64;  ///< Scale on the reference car's stiffness; 1.0 is the reference itself.
};

/**
 * \brief The reference car every stiffness is expressed against.
 *
 * \details Cornering stiffness is not measured per car; it is scaled from a known one, and upstream
 * that car is the Civic. The controller's slip factor and the parameter learner's base stiffness both
 * scale from these numbers, so they live here once — two copies would let the learner estimate against
 * one car while the controller steers by another, and the disagreement would read as estimator drift.
 */
struct ReferenceCar {
  double mass_kg = 1326.0 + 136.0;      ///< Kerb mass plus a driver [kg].
  double wheelbase_m = 2.70;            ///< Wheelbase [m].
  double center_to_front_frac = 0.4;    ///< Distance to the front axle as a fraction of the wheelbase.
  double stiff_front_n_rad = 192150.0;  ///< Front axle cornering stiffness [N/rad].
  double stiff_rear_n_rad = 202500.0;   ///< Rear axle cornering stiffness [N/rad].

  double centerToFront() const { return wheelbase_m * center_to_front_frac; }
  double centerToRear() const { return wheelbase_m - centerToFront(); }
};

inline constexpr ReferenceCar kReferenceCar{};

/**
 * \brief Axle cornering stiffnesses for `p`, scaled from the reference car.
 *
 * \param[in] p Vehicle whose geometry and mass set the scaling.
 * \param[in] stiffness_factor Scale on the reference stiffness; 1.0 gives the unscaled base.
 * \param[out] cF,cR Front and rear cornering stiffness [N/rad].
 */
inline void referenceStiffness(const VehicleModelParams& p, double stiffness_factor, double& cF, double& cR)
{
  const ReferenceCar& r = kReferenceCar;
  const double wb = std::max(p.wheelbase_m, 1e-3);
  const double aF = wb * p.center_to_front_frac;
  const double aR = wb - aF;
  cF = r.stiff_front_n_rad * stiffness_factor * p.mass_kg / r.mass_kg * (aR / wb) / (r.centerToRear() / r.wheelbase_m);
  cR = r.stiff_rear_n_rad * stiffness_factor * p.mass_kg / r.mass_kg * (aF / wb) / (r.centerToFront() / r.wheelbase_m);
}

inline double slipFactor(const VehicleModelParams& p)
{
  const double wb = std::max(p.wheelbase_m, 1e-3);
  const double aF = wb * p.center_to_front_frac;
  const double aR = wb - aF;
  double cF = 0.0, cR = 0.0;
  referenceStiffness(p, p.tire_stiffness_factor, cF, cR);
  if (cF <= 0.0 || cR <= 0.0)
    return 0.0;
  return p.mass_kg * (cF * aF - cR * aR) / (wb * wb * cF * cR);
}

inline double rollCompensationCurvature(double roll_rad, double v_mps, double slip_factor)
{
  constexpr double kG = 9.81;
  if (std::abs(slip_factor) < 1e-6)
    return 0.0;
  const double denom = 1.0 / slip_factor - v_mps * v_mps;
  if (std::abs(denom) < 1e-6)
    return 0.0;
  return kG * roll_rad / denom;
}

inline double steerFromCurvature(double kappa, double v_mps, double wheelbase_m, double slip_factor)
{
  const double v = std::max(v_mps, 0.0);
  const double factor = std::max(1.0 - slip_factor * v * v, 0.2);
  return std::atan(kappa * wheelbase_m * factor);
}

inline double curvatureFromSteer(double steer_rad, double v_mps, double wheelbase_m, double slip_factor)
{
  const double v = std::max(v_mps, 0.0);
  const double factor = std::max(1.0 - slip_factor * v * v, 0.2);
  return std::tan(steer_rad) / std::max(wheelbase_m, 1e-3) / factor;
}

}  // namespace adas
