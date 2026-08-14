#include "adas/lateral/kappa_solver.h"

#include <algorithm>

#include "adas/lateral/kappa_solver_acados.hpp"
#include "adas/lateral/kappa_solver_grad.hpp"
#include "adas/utils/logger.h"

namespace adas {
namespace lateral {
bool GradKappaSolver::solve(const Input& in, const std::vector<Vec2>& poly, double yaw, double Lf, double& kappa,
                            double& kappa_rate)
{
  flowpilot::LatMpcConfig fcfg;
  fcfg.max_lateral_jerk = cfg_.max_lateral_jerk;
  fcfg.steering_rate_weight = std::max(0.0, cfg_.steering_rate_weight);
  fcfg.steer_delay_s = std::max(0.05, cfg_.steer_delay_s);
  fcfg.rotation_radius = std::max(0.0, 0.5 * cfg_.wheelbase_m);
  mpc_.setConfig(fcfg);

  const auto sol = mpc_.update(in.speed_mps, yaw, Lf, poly, in.plan_poly, in.plan_yaw, in.plan_yaw_rate, in.frame_dt_s);
  if (!sol.ok)
    return false;
  kappa = sol.desired_curvature;
  kappa_rate = sol.desired_curvature_rate;
  return true;
}

bool AcadosKappaSolver::solve(const Input& in, const std::vector<Vec2>& poly, double yaw, double Lf, double& kappa,
                              double& kappa_rate)
{
  (void)Lf;
  std::vector<double> y_ref, psi_ref, r_ref;
  if (!flowpilot::LateralMpc::sampleRefs(poly, in.speed_mps, in.plan_poly, in.plan_yaw, in.plan_yaw_rate, y_ref,
                                         psi_ref, r_ref))
    return false;

  mpc_.setWeights({1.0, 0.11, 0.0, 0.05, std::max(0.0, cfg_.steering_rate_weight)});
  const auto a = mpc_.solve(in.speed_mps, std::max(0.0, 0.5 * cfg_.wheelbase_m), yaw, y_ref, psi_ref, r_ref,
                            std::max(0.05, cfg_.steer_delay_s));
  if (!a.ok)
    return false;
  kappa = a.desired_curvature;
  kappa_rate = a.desired_curvature_rate;
  return true;
}

std::unique_ptr<KappaSolver> makeKappaSolver(const std::string& name, const KappaSolverConfig& cfg)
{
  std::unique_ptr<KappaSolver> solver;
  if (name == "acados") {
    auto a = std::make_unique<AcadosKappaSolver>();
    if (a->available())
      solver = std::move(a);
    else
      LOGW("Planner: fp_solver=acados unavailable, falling back to grad");
  }
  if (!solver)
    solver = std::make_unique<GradKappaSolver>();
  solver->setConfig(cfg);
  return solver;
}

}  // namespace lateral
}  // namespace adas
