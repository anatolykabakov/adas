#pragma once

#include "adas/utils/lane_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/lateral/slew_guard.hpp"
#include "adas/services/lane_keep_gates.h"
#include "adas/utils/interval_filter.h"
#include "adas/utils/lat_control_pid.h"
#include "adas/utils/math_utils.h"
#include "adas/lateral/pp_planner.hpp"
#include "adas/lateral/vp_planner.hpp"
#include "adas/lateral/fp_planner.hpp"
#include "adas/lateral/planner.hpp"

namespace adas {
namespace services {
class LaneKeep : public adas::middleware::Service {
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
    double pid_kf = 6e-5;
    double pid_ff_floor_mps = 0.0;
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
    bool use_learned_params = false;
    bool roll_compensation = true;
    bool dp_parity_pack = false;
    std::string fp_solver = "grad";

    double fp_steer_delay_s = 0.35;

    double min_control_speed_mps = 1.5;

    double min_control_speed_hyst_mps = 0.5;

    double vision_nominal_dt_s = 0.09;

    double lane_max_age_s = 0.30;
    bool lka_suppress_on_blinker = true;
    double lka_blinker_resume_delay_s = 1.0;
    bool lat_recompute_setpoint = false;

    bool lat_require_assist = true;
    double assist_max_age_s = 0.5;

    LanePathConfig lane_path{};
  };

  LaneKeep() : LaneKeep(Config{}) {}
  explicit LaneKeep(Config config);

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

  const LaneKeepOutput& last() const { return last_; }

  lateral::PpPlanner::Config ppPlannerConfig() const;
  lateral::VpPlanner::Config vpPlannerConfig() const;
  lateral::FpPlanner::Config fpPlannerConfig() const;
  const Config& config() const { return config_; }
  bool useFlowpilot() const { return config_.controller == "fp"; }
  const char* solverName() const;
  const char* kappaSolverName() const;

  void registerParameters();

  void setController(std::string controller);
  void setPurePursuit(double k_dd, double ld_min, double ld_max, double shift);
  void setPpLdCurvGain(double gain)
  {
    config_.pp_ld_curv_gain = std::max(gain, 0.0);
    solver_.reset();
  }
  void setMaxSteerDeg(double max_steer_deg);

  void setSteerOutputEnabled(bool enabled) { steer_output_enabled_ = enabled; }
  bool steerOutputEnabled() const { return steer_output_enabled_; }

  void setSteerRatio(double ratio) { steer_ratio_ = std::max(ratio, 1e-3); }
  void setMpcKappaYawBlend(double alpha, double min_speed)
  {
    config_.mpc_kappa_yaw_blend = std::clamp(alpha, 0.0, 1.0);
    config_.mpc_kappa_yaw_min_speed = std::max(min_speed, 0.0);
    solver_.reset();
  }

  void setMpcEmaAlphas(double kappa_alpha, double epsi_alpha, double cte_alpha)
  {
    config_.mpc_kappa_ema_alpha = std::clamp(kappa_alpha, 0.0, 1.0);
    config_.mpc_epsi_ema_alpha = std::clamp(epsi_alpha, 0.0, 1.0);
    config_.mpc_cte_ema_alpha = std::clamp(cte_alpha, 0.0, 1.0);
    solver_.reset();
  }
  void setSteerSlewLimitDeg(double deg)
  {
    config_.steer_slew_limit_deg = deg;
    solver_.reset();
  }

  void setVehicleModel(bool on, double tire_stiffness_factor)
  {
    config_.lat_use_vehicle_model = on;
    if (tire_stiffness_factor > 0.05)
      config_.tire_stiffness_factor = tire_stiffness_factor;
  }

  void setFpSteeringRateWeight(double w)
  {
    config_.fp_steering_rate_weight = std::max(0.0, w);
    solver_.reset();
  }

  /** Hand the controller the online estimate. `valid` is the learner's own gate; when it is false the
   *  configured constants are used, so losing validity mid-drive walks the parameters back rather than
   *  freezing them at whatever the estimator last believed. */
  void setLearnedParams(bool valid, double tire_stiffness_factor, double steer_ratio, double angle_offset_deg)
  {
    learned_valid_ = valid && tire_stiffness_factor > 0.05 && steer_ratio > 1.0;
    if (learned_valid_) {
      learned_stiffness_ = tire_stiffness_factor;
      learned_steer_ratio_ = steer_ratio;
      learned_angle_offset_deg_ = angle_offset_deg;
    }
  }

  /** The parameters actually in force this tick, config or learned. Public because the debug message
   *  publishes them: a bag that does not say which stiffness produced a command cannot be used to judge
   *  the switch. */
  double effectiveStiffnessFactor() const
  {
    return (config_.use_learned_params && learned_valid_) ? learned_stiffness_ : config_.tire_stiffness_factor;
  }
  double effectiveSteerRatio() const
  {
    return (config_.use_learned_params && learned_valid_) ? learned_steer_ratio_ : steer_ratio_;
  }
  double effectiveAngleOffsetDeg() const
  {
    return (config_.use_learned_params && learned_valid_) ? learned_angle_offset_deg_ : 0.0;
  }
  bool usingLearnedParams() const { return config_.use_learned_params && learned_valid_; }

  void setFpSteerDelayS(double s)
  {
    config_.fp_steer_delay_s = std::max(0.05, s);
    solver_.reset();
  }
  /** Смещение камеры влево. Число живёт в двух местах: здесь его использует сдвиг опоры в ego-кадр,
   *  а `lane_path.cam_y_left_m` — расчёт положения в полосе. Обновлять надо обе копии, иначе
   *  параметр, поданный на ходу, разведёт их между собой. */
  void setCamYLeftM(double m)
  {
    config_.cam_y_left_m = m;
    config_.lane_path.cam_y_left_m = m;
  }
  void setPidGains(double kp, double ki, double kf) { lat_.setGains(kp, ki, kf); }
  void setSteerSign(double sign) { steer_sign_ = (sign < 0.0) ? -1.0 : 1.0; }

private:
  LaneFusionState lane_fusion_{};
  void onChassis(const ChassisSample& msg);
  void onLanes(const LanePathMsg& msg);
  void updateTorqueFromAngle();

  /** Steering setpoint from the road-wheel angle. The learned offset is added rather than
   *  subtracted: the learner solved `delta = (SWA - offset) / ratio`, and this is that relation run
   *  the other way. */
  void setDesiredFromSteer(double steer_rad);

  /// Reasons to withdraw the command, applied on every chassis tick.
  void applyGates(LaneKeepOutput& out, int64_t now_us, bool& assist_ok);

  /** Steering setpoint recomputed at the current speed between vision frames.
   *
   *  The whole chain — vehicle model, speed-dependent angle ceiling, rate limit — is computed by the
   *  fp planner itself: a recompute skipping any link would command something the per-frame path
   *  never produces. What is left here is the road-wheel to steering-wheel conversion and the debug
   *  fields. False when there is nothing to recompute from. */
  bool recomputeSetpoint(LaneKeepOutput& out);

  lateral::VehicleParams vehicleParams() const
  {
    lateral::VehicleParams v;
    v.wheelbase_m = config_.wheelbase_m;
    v.steer_ratio = effectiveSteerRatio();
    v.tire_stiffness_factor = effectiveStiffnessFactor();
    v.use_vehicle_model = config_.lat_use_vehicle_model;
    v.steer_sign = steer_sign_;
    v.road_roll_rad = road_roll_rad_;
    v.road_roll_valid = road_roll_valid_;
    return v;
  }
  void makeSolver();

  Config config_;
  std::unique_ptr<lateral::IPlanner> solver_;
  LatControlPid lat_;
  double max_steer_rad_ = 8.0 * M_PI / 180.0;
  double max_torque_cnm_ = 300.0;
  double steer_ratio_ = 15.7;
  bool learned_valid_ = false;
  double learned_stiffness_ = 0.64;
  double learned_steer_ratio_ = 15.7;
  double learned_angle_offset_deg_ = 0.0;
  double road_roll_rad_ = 0.0;
  bool road_roll_valid_ = false;
  double steer_sign_ = -1.0;
  bool steer_output_enabled_ = false;

  lateral::SlewGuard slew_;
  StaleGate stale_gate_;
  BlinkerGate blinker_gate_;
  AssistGate assist_gate_;
  IntervalFilter frame_dt_;
  HysteresisGate speed_gate_;

  ChassisSample chassis_;
  bool have_chassis_ = false;
  double desired_swa_deg_ = 0.0;
  double desired_swa_no_offset_deg_ = 0.0;
  bool have_desired_ = false;
  LaneKeepOutput last_;
};

}  // namespace services

}  // namespace adas
