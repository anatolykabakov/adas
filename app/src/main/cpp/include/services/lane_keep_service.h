#pragma once

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

#include "middleware/middleware.hpp"
#include "utils/adas_topics.h"
#include "utils/lat_control_pid.h"
#include "utils/math_utils.h"
#include "utils/path_lateral_state.h"
#include "utils/pure_pursuit.h"
#include "utils/vehicle_model.h"
#include "visionpilot/lateral_planning.hpp"
#include "flowpilot/lateral_mpc.hpp"

namespace adas {

struct LaneKeepOutput {
  int64_t timestamp_us = 0;
  int64_t capture_ts_us = 0;
  int64_t vision_ts_us = 0;
  int64_t chassis_ts_us = 0;
  int64_t publish_ts_us = 0;
  double steer_rad = 0.0;
  double steer_norm = 0.0;
  double desired_swa_deg = 0.0;
  double actual_swa_deg = 0.0;
  double angle_error_deg = 0.0;
  double lookahead_m = 0.0;
  double target_x = 0.0;
  double target_y = 0.0;
  bool has_target = false;
  double curvature = 0.0;
  double cte_m = 0.0;
  double epsi_rad = 0.0;
  std::string status = "ok";
  std::string controller = "pp";

  struct Debug {
    double speed_mps = 0.0;
    int n_points = 0;
    double pp_steer_raw_rad = 0.0;
    double mpc_kappa_path = 0.0;
    double mpc_kappa_yaw = 0.0;
    double mpc_kappa_used = 0.0;
    double mpc_dkappa_ds = 0.0;
    double mpc_delta_vp_rad = 0.0;
    double mpc_delta_clamped_rad = 0.0;
    double mpc_max_steer_rad = 0.0;
    double max_steer_rad = 0.0;
    bool slew_clipped = false;
    int torque_cnm = 0;
    bool steer_output_enabled = false;

    bool lane_anchored = false;
    double lane_width_m = 0.0;
    double lane_offset_m = 0.0;
    double center_force_m = 0.0;
    double p_lane_blend_scale = 0.0;
    double p_camera_offset_m = 0.0;
    double p_center_force_gain = 0.0;
  } dbg;
};

class LaneKeepService : public adas::Service {
public:
  struct Config {
    std::string controller = "pp";
    double wheelbase_m = 2.636;
    double max_steer_deg = 8.0;
    double pp_k_dd = 0.4;
    double pp_ld_min = 3.0;
    double pp_ld_max = 20.0;
    double pp_shift = 1.4;

    double pp_ld_curv_gain = 0.0;

    double cam_y_left_m = 0.0;
    double max_torque_cnm = 300.0;
    double steer_ratio = 15.7;
    double pid_kp = 0.6;
    double pid_ki = 0.2;
    double pid_kf = 0.00006;
    bool steer_output_enabled = false;
    double steer_sign = -1.0;
    double mpc_Lf = 2.67;
    double mpc_max_steer_deg = 25.0;

    double mpc_low_speed_steer_deg = 8.0;
    double mpc_steer_deg_per_mps = 1.0;

    double mpc_max_lateral_jerk = 5.0;
    double mpc_rate_min_speed = 2.0;
    double mpc_rate_limit_deg = 6.0;

    double mpc_kappa_yaw_blend = 0.0;
    double mpc_kappa_yaw_min_speed = 3.0;

    double mpc_epsi_gain = 0.5;
    double mpc_ff_scale = 2.0;

    double mpc_cte_weight_base = 20.0;
    double mpc_cte_quartic_scale = 5.0;

    double mpc_cte_gain_base = 0.6;

    double mpc_cte_gain_floor = 0.0;

    double fp_steering_rate_weight = 400.0;

    double mpc_kappa_ema_alpha = 1.0;

    double mpc_epsi_ema_alpha = 1.0;

    double mpc_cte_ema_alpha = 1.0;

    double steer_slew_limit_deg = 8.0;

    bool lat_use_vehicle_model = true;

    double tire_stiffness_factor = 0.64;

    double fp_steer_delay_s = 0.35;

    double min_control_speed_mps = 1.5;

    double min_control_speed_hyst_mps = 0.5;

    double vision_nominal_dt_s = 0.09;

    double lane_max_age_s = 0.30;
  };

  LaneKeepService() : LaneKeepService(Config{}) {}
  explicit LaneKeepService(Config config);

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "lane_keep"; }

  LaneKeepOutput step(double speed_mps, const LanePathMsg& path);
  LaneKeepOutput step(double speed_mps, const std::vector<Vec2>& polyline_ego)
  {
    LanePathMsg m;
    m.polyline = polyline_ego;
    return step(speed_mps, m);
  }

  PurePursuit& purePursuit() { return pp_; }
  const LaneKeepOutput& last() const { return last_; }
  const Config& config() const { return config_; }
  bool useMpc() const { return config_.controller == "mpc"; }
  bool useFlowpilot() const { return config_.controller == "fp"; }

  bool useMpcFamily() const { return useMpc() || useFlowpilot(); }

  void registerParameters();

  void setController(std::string controller);
  void setPurePursuit(double k_dd, double ld_min, double ld_max, double shift);
  void setPpLdCurvGain(double gain)
  {
    config_.pp_ld_curv_gain = std::max(gain, 0.0);
    pp_.ld_curv_gain = config_.pp_ld_curv_gain;
  }
  void setMaxSteerDeg(double max_steer_deg);

  void setSteerOutputEnabled(bool enabled) { steer_output_enabled_ = enabled; }
  bool steerOutputEnabled() const { return steer_output_enabled_; }

  void setSteerRatio(double ratio) { steer_ratio_ = std::max(ratio, 1e-3); }
  void setMpcKappaYawBlend(double alpha, double min_speed)
  {
    config_.mpc_kappa_yaw_blend = std::clamp(alpha, 0.0, 1.0);
    config_.mpc_kappa_yaw_min_speed = std::max(min_speed, 0.0);
  }

  void setMpcEmaAlphas(double kappa_alpha, double epsi_alpha, double cte_alpha)
  {
    config_.mpc_kappa_ema_alpha = std::clamp(kappa_alpha, 0.0, 1.0);
    config_.mpc_epsi_ema_alpha = std::clamp(epsi_alpha, 0.0, 1.0);
    config_.mpc_cte_ema_alpha = std::clamp(cte_alpha, 0.0, 1.0);
    kappa_ema_init_ = false;
    epsi_ema_init_ = false;
    cte_ema_init_ = false;
  }
  void setSteerSlewLimitDeg(double deg) { config_.steer_slew_limit_deg = deg; }

  void setVehicleModel(bool on, double tire_stiffness_factor)
  {
    config_.lat_use_vehicle_model = on;
    if (tire_stiffness_factor > 0.05)
      config_.tire_stiffness_factor = tire_stiffness_factor;
  }

  void setFpSteeringRateWeight(double w) { config_.fp_steering_rate_weight = std::max(0.0, w); }

  void setFpSteerDelayS(double s) { config_.fp_steer_delay_s = std::max(0.05, s); }
  void setCamYLeftM(double m) { config_.cam_y_left_m = m; }
  void setPidGains(double kp, double ki, double kf) { lat_.setGains(kp, ki, kf); }
  void setSteerSign(double sign) { steer_sign_ = (sign < 0.0) ? -1.0 : 1.0; }

private:
  void onChassis(const ChassisSample& msg);
  void onLanes(const LanePathMsg& msg);
  void publishLaneKeep(const LaneKeepOutput& out);
  void publishLaneKeepDebug(const LaneKeepOutput& out);
  void publishSteer(const LaneKeepOutput& out);
  void updateTorqueFromAngle();
  LaneKeepOutput stepPp(double speed_mps, const std::vector<Vec2>& polyline_ego);
  LaneKeepOutput stepMpc(double speed_mps, const std::vector<Vec2>& polyline_ego);
  LaneKeepOutput stepFlowpilot(double speed_mps, const LanePathMsg& path);

  void updateFrameDt(const LanePathMsg& msg);

  void updateChassisDt(const ChassisSample& msg);

  double emaAlpha(double alpha_at_nominal) const;

  double slipFactorOrZero() const;
  double activeMaxSteerRad() const;
  double mpcMaxSteerRad(double speed_mps) const;

  Config config_;
  PurePursuit pp_;
  LatControlPid lat_;
  visionpilot::LateralPlanner mpc_;
  flowpilot::LateralMpc fp_mpc_;
  visionpilot::KappaRateFilter kappa_rate_;
  double kappa_ema_ = 0.0;
  bool kappa_ema_init_ = false;
  double epsi_ema_ = 0.0;
  bool epsi_ema_init_ = false;
  double cte_ema_ = 0.0;
  bool cte_ema_init_ = false;
  double max_steer_rad_ = 8.0 * M_PI / 180.0;
  double max_torque_cnm_ = 300.0;
  double steer_ratio_ = 15.7;
  double steer_sign_ = -1.0;
  bool steer_output_enabled_ = false;

  ChassisSample chassis_;
  bool have_chassis_ = false;
  double desired_swa_deg_ = 0.0;
  bool have_desired_ = false;
  LaneKeepOutput last_;
  double last_mpc_steer_rad_ = 0.0;
  bool have_mpc_prev_ = false;
  double last_pub_steer_rad_ = 0.0;
  bool have_pub_prev_ = false;
  bool ref_stale_ = false;
  int pub_gap_frames_ = 0;
  int64_t last_frame_ts_us_ = 0;
  double frame_dt_s_ = 0.09;
  int64_t last_chassis_ts_us_ = 0;
  int64_t last_pid_us_ = 0;
  bool speed_gate_open_ = false;
  double chassis_dt_s_ = 0.05;
};

}  // namespace adas
