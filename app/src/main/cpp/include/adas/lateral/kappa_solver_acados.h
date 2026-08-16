#pragma once

#include "adas/lateral/acados_lat_mpc.h"
#include "adas/lateral/kappa_solver.h"

namespace adas {
namespace lateral {
/// The same problem solved by the acados interior-point method.
class AcadosKappaSolver final : public KappaSolver {
public:
  const char* name() const override { return "acados"; }

  bool available() const override { return mpc_.available(); }

  void reset() override { mpc_.reset(); }

  bool solve(const Input& in, const std::vector<Vec2>& poly, double yaw, double Lf, double& kappa,
             double& kappa_rate) override;

private:
  flowpilot::AcadosLatMpc mpc_;
};

}  // namespace lateral
}  // namespace adas
