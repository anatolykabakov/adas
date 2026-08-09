#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "utils/math_utils.h"

namespace adas {
namespace flowpilot {

struct LatMpcConfig {
  double path_weight = 1.0;
  double heading_weight = 0.11;
  double lat_accel_weight = 0.0;
  double lat_jerk_weight = 0.05;
  double steering_rate_weight = 400.0;
  double max_lateral_jerk = 5.0;
  double steer_delay_s = 0.35;
  double speed_offset = 10.0;
  double rotation_radius = 0.0;
  int gd_iters = 50;
  double gd_step = 0.1;
};

struct LatMpcResult {
  bool ok = false;
  double desired_curvature = 0.0;
  double desired_curvature_rate = 0.0;
  double steer_rad_vp = 0.0;
  double cost = 0.0;
};

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

  /** The setpoint again, from the last solved trajectory but at the *current* speed.
   *
   *  Upstream calls `get_lag_adjusted_curvature` at 100 Hz on a plan that updates at 20 Hz
   *  (`controlsd.py:730`), so the setpoint follows the plan's own trajectory between frames instead of
   *  standing still. We solved once per vision frame and held the answer for 78 ms: median setpoint jump
   *  0.107° against their 0.0067° per 10 ms, p99 **2.48°** and 3.78° at 5–12 m/s. For an actuator limited to
   *  200 cNm/s a step and a ramp are not the same request.
   *
   *  Nothing is re-solved here. The held trajectory is `psi_sol`/`r_sol`; what is re-evaluated is every place
   *  the speed enters — `kappa0 = r_sol[0]/v`, `avg_kappa = psi(delay)/(v·delay)`, and the curvature-rate
   *  ceiling `MAX_LATERAL_JERK/v²`. Be clear about the scope: between frames only the speed moves, so this
   *  does **not** remove the jump a new solve brings at the frame boundary. It removes the part of the jerk
   *  that comes from freezing the setpoint while the car accelerates or brakes — which is largest exactly on
   *  bend entry and exit.
   *
   *  Safe to call at any rate because the clamp is referenced to the plan's own `kappa0`, not to the previous
   *  output, so repeated calls do not ratchet. Empty until a solve has succeeded, cleared by `reset()`. */
  std::optional<double> curvatureAtSpeed(double speed_mps, double dt_s) const;

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
