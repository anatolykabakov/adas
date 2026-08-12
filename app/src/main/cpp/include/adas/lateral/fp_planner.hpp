#pragma once

#include <memory>
#include <optional>
#include <string>

#include "adas/lateral/kappa_solver.h"
#include "adas/lateral/limits.hpp"
#include "adas/lateral/planner.hpp"
#include "adas/lateral/types.hpp"

namespace adas {
namespace lateral {
/**
 * @brief Solve for curvature over the model plan horizon, then convert curvature to steering.
 *
 * The numerical method is picked by name at construction, so the choice lives here, not in the
 * service. The planner owns the solver and the memory between frames, which is why it also answers
 * the between-frame setpoint recompute: the curvature comes from the same solution as the frame did
 * and the rate limit sees the same history, so the 100 Hz path and the per-frame path cannot
 * command from different starting points.
 */
class FpPlanner final : public IPlanner {
public:
  struct Config {
    double Lf = 2.67;
    double wheelbase_m = 2.636;

    double max_lateral_jerk = 5.0;
    double steering_rate_weight = 1.0;
    double steer_delay_s = 0.2;

    double steer_slew_limit_deg = 0.0;

    bool roll_compensation = false;

    std::string solver = "grad";

    SteerLimits limits{};
  };

  explicit FpPlanner(Config config);

  const char* name() const override { return "fp"; }
  const char* solverName() const override;

  void reset() override;

  Output update(const Input& in) override;

  /// The angle plus the numbers by which it is read back from a recorded drive.
  struct Recompute {
    double steer_rad = 0.0;
    double curvature = 0.0;
    double max_steer_rad = 0.0;
  };

  /** Setpoint at the current speed between vision frames; empty when there is nothing to redo.
   *
   *  The rate limit applies per call rather than per frame, as it did before the planner was split
   *  out. */
  std::optional<Recompute> recomputeSteer(double speed_mps, double frame_dt_s, const VehicleParams& vehicle);

  const Config& config() const { return cfg_; }

private:
  /// Steering angle from curvature including roll compensation, before the speed clamp.
  double steerRaw(double kappa, double speed_mps, const VehicleParams& vehicle) const;

  Config cfg_;
  std::unique_ptr<KappaSolver> kappa_;

  double last_steer_rad_ = 0.0;
  bool have_prev_ = false;
};

}  // namespace lateral
}  // namespace adas
