#pragma once

#include <memory>
#include <optional>
#include <string>

#include "adas/lateral/types.hpp"

namespace adas {
namespace lateral {
/// Shared by every method: this describes the car, not the choice of method.
struct KappaSolverConfig {
  double wheelbase_m = 2.636;
  double max_lateral_jerk = 5.0;
  double steering_rate_weight = 1.0;
  double steer_delay_s = 0.2;
};

/**
 * @brief Numerical method producing the desired curvature from the plan ahead.
 *
 * The swappable part of the fp planner: the methods solve one problem and differ only in cost and
 * accuracy.
 */
class KappaSolver {
public:
  virtual ~KappaSolver() = default;

  virtual const char* name() const = 0;

  /// acados may be absent from the build.
  virtual bool available() const { return true; }

  virtual void reset() {}

  virtual void setConfig(const KappaSolverConfig& cfg) { cfg_ = cfg; }

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
