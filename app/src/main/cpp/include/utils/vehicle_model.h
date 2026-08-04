#pragma once

#include <algorithm>
#include <cmath>

namespace adas {

struct VehicleModelParams {
  double wheelbase_m = 2.636;
  double center_to_front_frac = 0.45;
  double mass_kg = 1533.0;
  double tire_stiffness_factor = 0.64;
};

inline double slipFactor(const VehicleModelParams& p)
{
  constexpr double kCivicMass = 1326.0 + 136.0;
  constexpr double kCivicWheelbase = 2.70;
  constexpr double kCivicC2F = kCivicWheelbase * 0.4;
  constexpr double kCivicC2R = kCivicWheelbase - kCivicC2F;
  constexpr double kCivicStiffFront = 192150.0;
  constexpr double kCivicStiffRear = 202500.0;

  const double wb = std::max(p.wheelbase_m, 1e-3);
  const double aF = wb * p.center_to_front_frac;
  const double aR = wb - aF;
  const double cF =
      kCivicStiffFront * p.tire_stiffness_factor * p.mass_kg / kCivicMass * (aR / wb) / (kCivicC2R / kCivicWheelbase);
  const double cR =
      kCivicStiffRear * p.tire_stiffness_factor * p.mass_kg / kCivicMass * (aF / wb) / (kCivicC2F / kCivicWheelbase);
  if (cF <= 0.0 || cR <= 0.0)
    return 0.0;
  return p.mass_kg * (cF * aF - cR * aR) / (wb * wb * cF * cR);
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
