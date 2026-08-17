#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace flowpilot {
/// Horizon, weights and limits of the flowpilot lateral MPC.
struct LatMpcConfig {
  double path_weight = 1.0;             ///< Cost on distance from the path.
  double heading_weight = 0.11;         ///< Cost on heading error.
  double lat_accel_weight = 0.0;        ///< Cost on lateral acceleration; 0 leaves comfort to the jerk term.
  double lat_jerk_weight = 0.05;        ///< Cost on lateral jerk — what keeps the command from stepping.
  double steering_rate_weight = 400.0;  ///< Cost on steering rate; higher is smoother and slower to react.
  double max_lateral_jerk = 5.0;        ///< Hard bound on lateral jerk [m/s^3].
  double steer_delay_s = 0.35;          ///< Actuator delay compensated for [s].
  double speed_offset = 10.0;           ///< Speed floor inside the cost [m/s], so terms stay finite at a standstill.
  double rotation_radius = 0.0;  ///< Offset of the controlled point from the rear axle [m]; 0 is the axle itself.
  int gd_iters = 50;             ///< Gradient-descent iterations per solve.
  double gd_step = 0.1;          ///< Gradient-descent step size.

  bool riccati_solver = true;  ///< Use the Riccati recursion instead of plain gradient descent.
  int riccati_iters = 8;       ///< Riccati iterations per solve.
};

/// One MPC solution: the command plus what the solver did to arrive at it.
struct LatMpcResult {
  bool ok = false;  ///< False when the solver did not converge; the caller must not use the result.
  double desired_curvature = 0.0;
  double desired_curvature_rate = 0.0;
  double steer_rad_vp = 0.0;
  double cost = 0.0;
};

/**
 * \brief Flowpilot's lateral MPC over the lane path.
 *
 * \details Optimises a steering trajectory against the path rather than a single point on it, which is
 * what lets it anticipate a curve instead of reacting inside one. The cost, the horizon and the limits
 * come from `LatMpcConfig`, so a drive's configuration records what it drove with.
 */
class LateralMpc {
public:
  static constexpr int N = 16;
  static constexpr int CONTROL_N = 17;
  static constexpr int X_DIM = 4;
  static constexpr double kTIdxMax = 32.0;
  static constexpr double kDefaultDtS = 0.05;

  explicit LateralMpc(LatMpcConfig cfg = {});

  void setConfig(const LatMpcConfig& cfg) { cfg_ = cfg; }
  void reset();

  LatMpcResult update(double speed_mps, double yaw_rate, double Lf, const std::vector<Vec2>& polyline_ego,
                      const std::vector<Vec2>& plan_poly_device = {}, const std::vector<double>& plan_yaw_device = {},
                      const std::vector<double>& plan_yaw_rate_device = {}, double dt_s = kDefaultDtS);

  std::optional<double> curvatureAtSpeed(double speed_mps, double dt_s) const;

  static bool sampleRefs(const std::vector<Vec2>& polyline_ego, double v, const std::vector<Vec2>& plan_poly_device,
                         const std::vector<double>& plan_yaw_device, const std::vector<double>& plan_yaw_rate_device,
                         std::vector<double>& y_ref, std::vector<double>& psi_ref, std::vector<double>& r_ref);

private:
  LatMpcConfig cfg_;
  std::array<double, N> u_{};
  std::array<double, X_DIM> x0_{};
  bool x0_inited_ = false;
  std::array<double, N + 1> psi_sol_{};
  std::array<double, N + 1> r_sol_{};
  bool has_sol_ = false;

  static double tNode(int i);
  static void buildRefs(const std::vector<Vec2>& poly_left, double v, std::array<double, N + 1>& y_ref,
                        std::array<double, N + 1>& psi_ref, std::array<double, N + 1>& r_ref);
  static void buildRefsWithOrientation(const std::vector<Vec2>& poly_left, double v, const std::vector<Vec2>& plan_left,
                                       const std::vector<double>& plan_yaw_left,
                                       const std::vector<double>& plan_yaw_rate_left, std::array<double, N + 1>& y_ref,
                                       std::array<double, N + 1>& psi_ref, std::array<double, N + 1>& r_ref);
  void forward(const std::array<double, N>& u, const std::array<double, X_DIM>& x0, double v,
               std::array<double, N + 1>& x, std::array<double, N + 1>& y, std::array<double, N + 1>& psi,
               std::array<double, N + 1>& r) const;
  double evalCost(const std::array<double, N>& u, const std::array<double, X_DIM>& x0, double v,
                  const std::array<double, N + 1>& y_ref, const std::array<double, N + 1>& psi_ref,
                  const std::array<double, N + 1>& r_ref, std::array<double, N + 1>& psi_sol,
                  std::array<double, N + 1>& r_sol) const;
  static double interpAtTime(double t, const std::array<double, N + 1>& vals);
  double lagAdjustedCurvature(double v, const std::array<double, N + 1>& psi_sol,
                              const std::array<double, N + 1>& r_sol, const std::array<double, N>& u,
                              double dt_s) const;
};

}  // namespace flowpilot
}  // namespace adas
