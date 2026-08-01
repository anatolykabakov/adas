#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace visionpilot {

extern std::size_t N;

extern double g_cte_weight_base;
extern double g_cte_quartic_scale;
void set_cost_weights(double cte_weight_base, double cte_quartic_scale);

extern double g_epsi_gain;
extern double g_ff_scale;
void set_warm_start_gains(double epsi_gain, double ff_scale);

extern double g_cte_gain_base;
void set_cte_gain_base(double base);

extern double g_cte_gain_floor;
void set_cte_gain_floor(double floor);

class LateralPlanner {
public:
  LateralPlanner();
  ~LateralPlanner();

  std::vector<double> compute_steering(double Lf, const Eigen::VectorXd& state, const Eigen::VectorXd& v_schedule,
                                       const Eigen::VectorXd& kappa_schedule);
};

Eigen::VectorXd build_kappa_schedule(double Lf, double epsi, double kappa, double dkappa_ds);

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
