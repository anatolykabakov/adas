#pragma once

#include <string>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace lateral {

struct VehicleParams {
  double wheelbase_m = 2.636;
  double steer_ratio = 15.7;
  double tire_stiffness_factor = 1.0;
  double mass_kg = 1533.0;
  double center_to_front_frac = 0.45;
  bool use_vehicle_model = true;
  double steer_sign = -1.0;
  double road_roll_rad = 0.0;
  bool road_roll_valid = false;
};

struct Input {
  double speed_mps = 0.0;
  double yaw_rate = 0.0;
  bool have_chassis = false;
  double frame_dt_s = 0.05;

  std::vector<Vec2> polyline_ego;
  std::vector<Vec2> plan_poly;
  std::vector<double> plan_yaw;
  std::vector<double> plan_yaw_rate;
  bool lane_anchored = false;
  double lane_width_m = 0.0;
  double lane_offset_m = 0.0;

  VehicleParams vehicle;
};

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

struct Output {
  bool ok = false;
  std::string status = "ok";
  std::string controller = "pp";

  double steer_rad = 0.0;
  double steer_norm = 0.0;
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
