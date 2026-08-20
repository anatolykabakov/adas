#pragma once

#include <memory>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace flowpilot {
/** Wrapper over the acados-generated lateral MPC solver. */
class AcadosLatMpc {
public:
  struct Result {
    bool ok = false;
    double desired_curvature = 0.0;
    double desired_curvature_rate = 0.0;
    double cost = 0.0;
    int status = -1;
    std::vector<double> psi_sol;
    std::vector<double> r_sol;
  };

  struct Weights {
    double path = 1.0;
    double heading = 0.11;
    double lat_accel = 0.0;
    double lat_jerk = 0.05;
    double steering_rate = 400.0;
  };

  /// Creates the acados solver instance; check available() before use.
  AcadosLatMpc();
  ~AcadosLatMpc();
  AcadosLatMpc(const AcadosLatMpc&) = delete;
  AcadosLatMpc& operator=(const AcadosLatMpc&) = delete;

  /// \return False when the acados capsule failed to build.
  bool available() const;

  /// \return Number of horizon nodes the generated solver was built with.
  static int horizonNodes();
  /// \return Time of node \p i along the horizon [s].
  static double nodeTime(int i);

  /// Replace the cost weights.
  void setWeights(const Weights& w);
  /// Drop the warm start.
  void reset();

  /**
   * \brief Solve one horizon.
   * \param[in] v_ego Speed [m/s]; \p rotation_radius lever arm [m]; \p yaw_rate measured [rad/s].
   * \param[in] y_ref Lateral reference at the horizon nodes [m].
   * \return Curvature sequence and solver status.
   */
  Result solve(double v_ego, double rotation_radius, double yaw_rate, const std::vector<double>& y_ref,
               const std::vector<double>& psi_ref, const std::vector<double>& r_ref, double steer_delay_s);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flowpilot
}  // namespace adas
