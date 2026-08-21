#pragma once

#include <memory>
#include <optional>
#include <string>

#include "adas/lateral/types.h"

namespace adas {
namespace lateral {
/// Shared by every method: this describes the car, not the choice of method.
struct KappaSolverConfig {
  double wheelbase_m = 2.636;         ///< Wheelbase [m].
  double max_lateral_jerk = 5.0;      ///< Comfort bound on lateral jerk [m/s^3].
  double steering_rate_weight = 1.0;  ///< Cost on steering rate.
  double steer_delay_s = 0.2;         ///< Actuator delay compensated for [s].
};

/** Numerical method producing the desired curvature from the plan ahead. */
class KappaSolver {
public:
  virtual ~KappaSolver() = default;

  /// Solver name for logs: "grad" or "acados".
  virtual const char* name() const = 0;

  /// acados may be absent from the build.
  virtual bool available() const { return true; }

  /// Drop warm-start state.
  virtual void reset() {}

  /// Replace the solver config.
  virtual void setConfig(const KappaSolverConfig& cfg) { cfg_ = cfg; }

  /**
   * \brief Solve one horizon.
   * \param[in] in Speed and state; \p poly reference path [m]; \p yaw heading error [rad]; \p Lf lever arm [m].
   * \param[out] kappa Commanded curvature [1/m].
   * \return False when the solver failed; the caller keeps the previous command.
   */
  virtual bool solve(const Input& in, const std::vector<Vec2>& poly, double yaw, double Lf, double& kappa,
                     double& kappa_rate) = 0;

  /** Curvature from the last solution at another speed; empty when the method cannot do that.
   *
   *  Used to refine the setpoint between frames: chassis arrives more often than the picture, and
   *  solving again on every chassis tick costs more than reading the profile already computed. */
  virtual std::optional<double> curvatureAtSpeed(double speed_mps, double dt_s)
  {
    (void)speed_mps;
    (void)dt_s;
    return std::nullopt;
  }

protected:
  KappaSolverConfig cfg_{};
};

/** Method by name: "acados", anything else means gradient descent.
 *
 *  An unavailable acados falls back to grad with a log line: a build without acados must still
 *  drive. */
std::unique_ptr<KappaSolver> makeKappaSolver(const std::string& name, const KappaSolverConfig& cfg);

}  // namespace lateral
}  // namespace adas
