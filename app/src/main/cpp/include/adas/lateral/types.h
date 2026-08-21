#pragma once

#include <string>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace lateral {
/// The car as the planners see it: geometry, mass distribution, tyre stiffness, road bank.
struct VehicleParams {
  double wheelbase_m = 2.636;          ///< Wheelbase [m].
  double steer_ratio = 15.7;           ///< Steering-wheel to road-wheel ratio.
  double tire_stiffness_factor = 1.0;  ///< Scale on the reference tyre stiffness.
  double mass_kg = 1533.0;             ///< Kerb mass plus a driver [kg].
  double center_to_front_frac = 0.45;  ///< Distance to the front axle as a fraction of the wheelbase.
  double steer_sign = -1.0;            ///< Which way a positive command turns the wheel: +1 or -1.
  double road_roll_rad = 0.0;          ///< Road bank [rad], which otherwise reads as understeer.
  bool road_roll_valid = false;        ///< False when the bank estimate is not usable this tick.
  bool use_vehicle_model = true;       ///< False drops the slip term, leaving the kinematic model.
};

/// One planning tick: speed, the path ahead, the car, and where in the lane we are.
struct Input {
  double speed_mps = 0.0;     ///< Ego speed [m/s].
  double yaw_rate = 0.0;      ///< Measured yaw rate [rad/s].
  bool have_chassis = false;  ///< False when no chassis frame has arrived yet.
  double frame_dt_s = 0.05;   ///< Interval since the previous planning tick [s].

  std::vector<Vec2> polyline_ego;
  std::vector<Vec2> plan_poly;
  std::vector<double> plan_yaw;
  std::vector<double> plan_yaw_rate;
  bool lane_anchored = false;  ///< True when the path follows the lane lines rather than the model plan.
  double lane_width_m = 0.0;   ///< Measured lane width [m]; 0 when unknown.
  double lane_offset_m = 0.0;

  VehicleParams vehicle;
};

/** Everything a planner computed on the way to its command. */
struct Debug {
  double speed_mps = 0.0;
  int n_points = 0;
  double pp_steer_raw_rad = 0.0;
  double pp_lookahead_m = 0.0;
  double mpc_kappa_path = 0.0;
  double mpc_kappa_yaw = 0.0;
  double mpc_kappa_used = 0.0;
  double mpc_dkappa_ds = 0.0;
  double mpc_delta_vp_rad = 0.0;
  double mpc_delta_clamped_rad = 0.0;
  double mpc_max_steer_rad = 0.0;
  double max_steer_rad = 0.0;
  double road_roll_deg = 0.0;
  std::string kappa_solver;
};

/// A planner's result: the command, the limit in force, and the debug trail behind it.
struct Output {
  bool ok = false;
  std::string status = "ok";
  std::string controller = "pp";

  double steer_rad = 0.0;
  double steer_norm = 0.0;
  double max_steer_rad = 0.0;
  double curvature = 0.0;
  double curvature_rate = 0.0;
  double cte_m = 0.0;
  double epsi_rad = 0.0;
  double lookahead_m = 0.0;
  double target_x = 0.0;
  double target_y = 0.0;
  bool has_target = false;

  Debug dbg;
};

}  // namespace lateral
}  // namespace adas
