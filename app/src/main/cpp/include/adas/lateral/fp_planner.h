#pragma once

#include <memory>
#include <optional>
#include <string>

#include "adas/lateral/kappa_solver.h"
#include "adas/lateral/limits.h"
#include "adas/lateral/planner.h"
#include "adas/lateral/types.h"

namespace adas {
namespace lateral {
/** Solve for curvature over the model plan horizon, then convert curvature to steering. */
class FpPlanner final : public IPlanner {
public:
  struct Config {
    double Lf = 2.67;            ///< Lever arm for converting curvature into an angle [m].
    double wheelbase_m = 2.636;  ///< Wheelbase [m], for the slip model.

    double max_lateral_jerk = 5.0;      ///< Comfort bound on lateral jerk [m/s^3].
    double steering_rate_weight = 1.0;  ///< Cost on steering rate inside the solver.
    double steer_delay_s = 0.2;         ///< Actuator delay compensated for [s].

    double steer_slew_limit_deg = 0.0;  ///< Rate limit on the output [deg/s]; 0 disables it.

    std::string solver = "grad";  ///< Numerical method: "grad" or "acados".

    SteerLimits limits{};  ///< Speed-dependent ceiling on the angle.
  };

  /// \param[in] config Solver choice, weights and limits.
  explicit FpPlanner(Config config);

  const char* name() const override { return "fp"; }
  const char* solverName() const override;

  void reset() override;

  Output update(const Input& in) override;

  /// The angle plus the numbers by which it is read back from a recorded drive.
  struct Recompute {
    double steer_rad = 0.0;      ///< Commanded road-wheel angle [rad].
    double curvature = 0.0;      ///< Curvature the command corresponds to [1/m].
    double max_steer_rad = 0.0;  ///< Limit in force this tick [rad].
  };

  /** Setpoint at the current speed between vision frames; empty when there is nothing to redo.
   *
   *  The rate limit applies per call rather than per frame, as it did before the planner was split
   *  out. */
  std::optional<Recompute> recomputeSteer(double speed_mps, double frame_dt_s, const VehicleParams& vehicle);

  /// \return The config in force.
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
