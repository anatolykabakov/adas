#pragma once

#include "adas/lateral/flowpilot_mpc.h"
#include "adas/lateral/kappa_solver.h"

namespace adas {
namespace lateral {
/// Gradient descent over the plan horizon. Available everywhere; keeps a curvature profile.
class GradKappaSolver final : public KappaSolver {
public:
  const char* name() const override { return "grad"; }

  void reset() override { mpc_.reset(); }

  bool solve(const Input& in, const std::vector<Vec2>& poly, double yaw, double Lf, double& kappa,
             double& kappa_rate) override;

  std::optional<double> curvatureAtSpeed(double speed_mps, double dt_s) override
  {
    return mpc_.curvatureAtSpeed(speed_mps, dt_s);
  }

private:
  flowpilot::LateralMpc mpc_;
};

}  // namespace lateral
}  // namespace adas
