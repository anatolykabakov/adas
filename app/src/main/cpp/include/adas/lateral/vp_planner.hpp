#pragma once

#include "adas/lateral/limits.hpp"
#include "adas/lateral/planner.hpp"
#include "adas/lateral/types.hpp"
#include "adas/lateral/visionpilot_mpc.hpp"

namespace adas {
namespace lateral {
/**
 * @brief Gradient descent over a preview horizon: steering from offset, heading and curvature.
 *
 * The planner owns the solver and all state between frames. That state used to live in the
 * service: nine LaneKeep fields existed for one of three planners and were cleared in its branch.
 */
class VpPlanner final : public IPlanner {
public:
  struct Config {
    double Lf = 0.0;

    double cte_ema_alpha = 1.0;
    double epsi_ema_alpha = 1.0;
    double kappa_ema_alpha = 1.0;
    double vision_nominal_dt_s = 0.05;

    double kappa_yaw_blend = 0.0;
    double kappa_yaw_min_speed = 0.0;

    double rate_limit_deg = 0.0;
    double rate_min_speed = 1.0;
    double max_lateral_jerk = 5.0;

    visionpilot::Params solver{};
    VehicleParams vehicle{};
    SteerLimits limits{};
  };

  explicit VpPlanner(Config config) : cfg_(std::move(config)), mpc_(cfg_.solver) {}

  const char* name() const override { return "mpc"; }

  void reset() override;

  Output update(const Input& in) override;

  const Config& config() const { return cfg_; }

private:
  /// Sample weight rescaled from the nominal frame rate to the actual one.
  double emaAlpha(double alpha_at_nominal, double frame_dt_s) const;

  Config cfg_;
  visionpilot::LateralPlanner mpc_;
  visionpilot::KappaRateFilter kappa_rate_;

  double cte_ema_ = 0.0;
  bool cte_ema_init_ = false;
  double epsi_ema_ = 0.0;
  bool epsi_ema_init_ = false;
  double kappa_ema_ = 0.0;
  bool kappa_ema_init_ = false;

  double last_steer_rad_ = 0.0;
  bool have_prev_ = false;
};

}  // namespace lateral
}  // namespace adas
