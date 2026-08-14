#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace visionpilot {
/**
 * \brief Solver weights and gains.
 *
 * These used to be globals with setters: whoever called one changed the behaviour of every instance
 * at once, and a drive configuration could not tell you which weights it drove with, because the
 * last setter call won. They now belong to the instance and come from the planner configuration.
 */
struct Params {
  std::size_t N = 20;  ///< Horizon length in steps.

  double cte_weight_base = 20.0;   ///< Weight on cross-track error at small errors.
  double cte_quartic_scale = 5.0;  ///< How much faster that weight grows on large errors.

  double epsi_gain = 0.5;       ///< Gain on heading error.
  double ff_scale = 2.0;        ///< Scale on the curvature feedforward.
  double cte_gain_base = 0.6;   ///< Gain on the cross-track correction.
  double cte_gain_floor = 0.0;  ///< Lower clamp on that gain.
};

/**
 * \brief Visionpilot's lateral solver: a steering sequence over a curvature schedule.
 *
 * \details Takes the state and the speed and curvature schedules and returns the steering plan. Weights
 * live in `Params` on the instance rather than in globals, so two instances can differ and a recorded
 * configuration means something.
 */
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

/// Rate limit on the commanded curvature, so a new solution cannot step the wheel.
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
