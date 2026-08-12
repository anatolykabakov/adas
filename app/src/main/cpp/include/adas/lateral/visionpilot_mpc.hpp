#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace visionpilot {
/**
 * @brief Solver weights and gains.
 *
 * These used to be globals with setters: whoever called one changed the behaviour of every instance
 * at once, and a drive configuration could not tell you which weights it drove with, because the
 * last setter call won. They now belong to the instance and come from the planner configuration.
 */
struct Params {
  std::size_t N = 20;

  double cte_weight_base = 20.0;
  double cte_quartic_scale = 5.0;

  double epsi_gain = 0.5;
  double ff_scale = 2.0;
  double cte_gain_base = 0.6;
  double cte_gain_floor = 0.0;
};

class LateralPlanner {
public:
  explicit LateralPlanner(Params params = {});
  ~LateralPlanner();

  const Params& params() const { return params_; }

  std::vector<double> compute_steering(double Lf, const Eigen::VectorXd& state, const Eigen::VectorXd& v_schedule,
                                       const Eigen::VectorXd& kappa_schedule);

private:
  Params params_;
};

Eigen::VectorXd build_kappa_schedule(double Lf, double epsi, double kappa, double dkappa_ds, std::size_t N);

class KappaRateFilter {
public:
  double update(double kappa, double ego_v, double dt = 0.05);
  void reset();

private:
  double prev_kappa_ = 0.0;
  double dkappa_filt_ = 0.0;
  bool has_prev_ = false;
};

}  // namespace visionpilot
