#pragma once

#include "adas/utils/lane_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/lateral/learned_vehicle.h"
#include "adas/utils/interval_filter.h"
#include "adas/utils/lat_control_pid.h"
#include "adas/utils/math_utils.h"
#include "adas/lateral/pp_planner.h"
#include "adas/lateral/fp_planner.h"
#include "adas/lateral/planner.h"
#include "adas/longitudinal/long_planner.h"

namespace adas {
namespace services {
/** Turns lane lines into a plan: curvature laterally, target speed longitudinally. */
class Planner : public adas::middleware::Service {
public:
  struct Config {
    std::string controller = "pp";  ///< Active planner: "pp" or "fp" — `vehicle.lane_keep_controller`.
    double wheelbase_m = 2.636;     ///< Wheelbase [m].
    double max_steer_deg = 8.0;     ///< Ceiling on the commanded road-wheel angle [deg].
    double pp_k_dd = 0.4;           ///< Pure pursuit: lookahead per unit speed [s].
    double pp_ld_min = 3.0;         ///< Pure pursuit: lower clamp on the lookahead [m]; below it the command chatters.
    double pp_ld_max = 20.0;        ///< Pure pursuit: upper clamp [m]; above it the car cuts corners.
    double pp_shift = 1.4;          ///< Lateral offset of the reference [m], positive to the left.

    double pp_ld_curv_gain = 0.0;  ///< Extra lookahead per unit curvature [m]; 0 keeps it speed-only.

    /// Camera offset from the centreline [m], positive left. Second copy lives in `lane_path`.
    double cam_y_left_m = 0.0;
    double max_torque_cnm = 300.0;  ///< Torque at full command [cNm]; must not exceed what the panda allows.
    double steer_ratio = 15.7;      ///< Steering-wheel to road-wheel ratio.
    double pid_kp = 0.6;            ///< Angle PID: proportional gain.
    double pid_ki = 0.2;            ///< Angle PID: integral gain, per second.
    double pid_kf = 6e-5;           ///< Angle PID: feedforward gain on `swa * (v^2 + v0^2)`.
    double pid_ff_floor_mps = 0.0;  ///< Speed floor in the feedforward [m/s], so it does not vanish at low speed.
    double steer_sign = -1.0;       ///< Which way a positive command turns the wheel on this car: +1 or -1.
    double mpc_Lf = 2.67;  ///< Lever arm for curvature to angle [m]. Not the wheelbase — see `Control::Config::lf_m`.
    double mpc_max_steer_deg = 25.0;  ///< Absolute ceiling on the solver output [deg].

    double mpc_low_speed_steer_deg = 8.0;  ///< Angle allowed at walking pace [deg]; the limit ramps up from here.
    double mpc_steer_deg_per_mps = 1.0;    ///< Slope of that ramp [deg per m/s].

    double mpc_max_lateral_jerk = 5.0;  ///< Comfort bound on lateral jerk [m/s^3].
    double mpc_rate_min_speed = 2.0;    ///< Below this speed the rate limit is not applied [m/s].
    double mpc_rate_limit_deg = 6.0;    ///< Maximum change of the solver output per second [deg/s].

    double fp_steering_rate_weight = 400.0;  ///< fp solver: cost on steering rate; higher is smoother and slower.

    double steer_slew_limit_deg = 8.0;  ///< Rate limit on the commanded steering-wheel angle [deg/s].

    double tire_stiffness_factor = 0.64;  ///< Scale on the reference tyre stiffness; 1.0 is the reference car.
    bool lat_use_vehicle_model = true;    ///< False drops the slip term, leaving the kinematic model.
    std::string fp_solver = "grad";       ///< Numerical method inside fp: "grad" or "acados".

    double fp_steer_delay_s = 0.35;  ///< Actuator delay the fp solver compensates for [s].

    double min_control_speed_mps = 1.5;  ///< Below this speed nothing is commanded [m/s].

    double min_control_speed_hyst_mps = 0.5;  ///< Hysteresis on that threshold [m/s], so the command does not flicker.

    double vision_nominal_dt_s = 0.09;  ///< Assumed interval between model frames [s], until a real one is measured.

    double lane_max_age_s = 0.30;             ///< A path older than this stops being planned on [s].
    double lka_blinker_resume_delay_s = 1.0;  ///< Delay before steering resumes after the blinker goes off [s].

    double assist_max_age_s = 0.5;  ///< A panda report older than this counts as unknown [s].
    /// The longitudinal plan is produced here too, the way upstream's `plannerd` emits both plans.
    bool long_plan_enabled = true;
    longitudinal::PlannerConfig long_plan{};  ///< Longitudinal planner settings.

    LanePathConfig lane_path{};  ///< How lane lines are turned into a reference path.
  };

  /// Constructs with the default config.
  Planner() : Planner(Config{}) {}
  /// \param[in] config Planner settings; see Config fields.
  explicit Planner(Config config);

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "planner"; }

  /**
   * \brief One planning tick.
   * \param[in] speed_mps Ego speed [m/s]; the lookahead and every speed-dependent limit scale with it.
   * \param[in] path Reference path in the ego frame plus the lane-line diagnostics that came with it.
   * \return The plan and the debug trail behind it.
   */
  LaneKeepOutput step(double speed_mps, const LanePathMsg& path);
  /**
   * \brief Same tick from a bare polyline, for tests and offline harnesses.
   * \param[in] speed_mps Ego speed [m/s].
   * \param[in] polyline_ego Reference path in the ego frame [m], x forward.
   */
  LaneKeepOutput step(double speed_mps, const std::vector<Vec2>& polyline_ego)
  {
    LanePathMsg m;
    m.polyline = polyline_ego;
    return step(speed_mps, m);
  }

  /// The last plan produced, as published. Empty until the first tick.
  const LaneKeepOutput& last() const { return last_; }

  /// Per-planner configuration derived from this service's config, as each planner expects it.
  lateral::PpPlanner::Config ppPlannerConfig() const;
  /// \return The flowpilot planner's config, derived from this service's.
  lateral::FpPlanner::Config fpPlannerConfig() const;
  /// \return The config in force.
  const Config& config() const { return config_; }
  /// \return True when the fp solver is the active controller.
  bool useFlowpilot() const { return config_.controller == "fp"; }
  /// Name of the active planner, as it goes into the bag: "pp", "vp" or "fp".
  const char* solverName() const;
  /// Numerical method inside the fp planner ("grad" or "acados"); empty for the others.
  const char* kappaSolverName() const;

  /// Binds the runtime knobs, so a parameter can be changed on a moving car.
  void registerParameters();

  /**
   * \brief Switch the active planner.
   *
   * \param[in] controller One of "pp", "vp", "fp". An unknown name leaves the current planner in place.
   */
  void setController(std::string controller);
  /**
   * \brief Pure-pursuit geometry.
   * \param[in] k_dd Lookahead per unit speed [s]: the distance is `k_dd * v`, clamped below.
   * \param[in] ld_min Lower clamp on the lookahead [m]; below it the command chatters.
   * \param[in] ld_max Upper clamp on the lookahead [m]; above it the car cuts corners.
   * \param[in] shift Lateral offset applied to the reference [m], positive to the left.
   */
  void setPurePursuit(double k_dd, double ld_min, double ld_max, double shift);
  /**
   * \brief Extra lookahead on curvature.
   *
   * \param[in] gain Metres of lookahead per unit curvature; negative values are clamped to zero.
   */
  void setPpLdCurvGain(double gain)
  {
    config_.pp_ld_curv_gain = std::max(gain, 0.0);
    solver_.reset();
  }
  /// \param[in] max_steer_deg Ceiling on the commanded road-wheel angle [deg].
  void setMaxSteerDeg(double max_steer_deg);

  /// \param[in] ratio Steering-wheel to road-wheel ratio; the configured value, not the learned one.
  void setSteerRatio(double ratio) { veh_.setSteerRatio(ratio); }
  /// \param[in] deg Maximum change of the commanded steering-wheel angle per second [deg/s].
  void setSteerSlewLimitDeg(double deg)
  {
    config_.steer_slew_limit_deg = deg;
    solver_.reset();
  }
  /// The configured steering ratio, ignoring the learned estimate.
  double steerRatio() const { return veh_.steerRatio(); }

  /// \param[in] tire_stiffness_factor Scale on the reference tyre stiffness; values <= 0.05 are ignored
  /// as implausible rather than clamped, so a bad parameter cannot quietly change the car model.
  void setTireStiffnessFactor(double tire_stiffness_factor)
  {
    if (tire_stiffness_factor > 0.05) {
      config_.tire_stiffness_factor = tire_stiffness_factor;
      veh_.setTireStiffnessFactor(tire_stiffness_factor);
    }
  }

  /// \brief Choose between the slip model and plain kinematics, and set the stiffness scale.
  ///
  /// \param[in] on False leaves the kinematic bicycle model, which is what a simulated ego needs.
  /// \param[in] tire_stiffness_factor Scale on the reference tyre stiffness; ignored when <= 0.05.
  void setVehicleModel(bool on, double tire_stiffness_factor)
  {
    config_.lat_use_vehicle_model = on;
    setTireStiffnessFactor(tire_stiffness_factor);
  }

  /// \param[in] w Cost weight on steering rate in the fp solver; higher is smoother and slower to react.
  void setFpSteeringRateWeight(double w)
  {
    config_.fp_steering_rate_weight = std::max(0.0, w);
    solver_.reset();
  }

  /** \brief Feed the online vehicle estimate into the solver.
   *
   *  Curvature must be computed for the same car the controller will later turn into an angle. `Control`
   *  keeps its own copy of the estimate, which is what the shared `LearnedVehicle` class exists for. */
  void setLearnedParams(bool valid, double tire_stiffness_factor, double steer_ratio, double angle_offset_deg)
  {
    veh_.setLearnedParams(valid, tire_stiffness_factor, steer_ratio, angle_offset_deg);
  }

  /** \brief Parameters in force on this tick.
   *
   *  Public because the debug message prints them: a recording that does not say which stiffness produced
   *  a command cannot be used to judge the transition. */
  double effectiveStiffnessFactor() const { return veh_.effectiveStiffnessFactor(); }
  /// \return Steering ratio in force.
  double effectiveSteerRatio() const { return veh_.effectiveSteerRatio(); }
  /// Learned steering-wheel angle zero [deg]; 0 while the estimate is not in use.
  double effectiveAngleOffsetDeg() const { return veh_.effectiveAngleOffsetDeg(); }
  /// True when the numbers above come from the learner rather than from the config.
  bool usingLearnedParams() const { return veh_.usingLearnedParams(); }

  /// \param[in] s Actuator delay the fp solver compensates for [s], floored at 0.05.
  void setFpSteerDelayS(double s)
  {
    config_.fp_steer_delay_s = std::max(0.05, s);
    solver_.reset();
  }
  /** \brief Camera offset to the left [m].
   *
   *  The number lives in two places: here it shifts the reference into the ego frame, and
   *  `lane_path.cam_y_left_m` computes the in-lane position from it. Both copies must be updated, or a
   *  parameter changed while driving will drive them apart. */
  void setCamYLeftM(double m)
  {
    config_.cam_y_left_m = m;
    config_.lane_path.cam_y_left_m = m;
  }

private:
  LaneFusionState lane_fusion_{};
  void onChassis(const ChassisSample& msg);
  void onLanes(const LanePathMsg& msg);
  double commandCurvature(double speed_mps) const;
  /// Longitudinal plan: the speed/acceleration trajectory behind the lead or at the set speed. Own tick
  /// (20 Hz, upstream's model rate), shared inputs.
  void longTick();
  /// The set speed as the stalk and the parameters drive it.
  void updateCruiseSetpoint(const adas::proto::CarState& cs);

  lateral::VehicleParams vehicleParams() const
  {
    lateral::VehicleParams v;
    v.wheelbase_m = config_.wheelbase_m;
    v.steer_ratio = veh_.effectiveSteerRatio();
    v.tire_stiffness_factor = veh_.effectiveStiffnessFactor();
    v.use_vehicle_model = config_.lat_use_vehicle_model;
    v.steer_sign = config_.steer_sign < 0.0 ? -1.0 : 1.0;
    v.road_roll_rad = road_roll_rad_;
    v.road_roll_valid = road_roll_valid_;
    return v;
  }
  void makeSolver();

  Config config_;
  lateral::LearnedVehicle veh_;
  /// This frame's reference path. The longitudinal plan needs it to see the curve ahead.
  std::vector<Vec2> path_{};
  adas::proto::ModelLongPlan model_long_{};
  bool have_model_long_ = false;
  int64_t model_long_ts_ms_ = 0;
  adas::proto::CarState car_state_{};
  longitudinal::Planner long_planner_;
  longitudinal::CruiseSetpoint cruise_;
  bool long_reset_pending_ = true;
  bool cruise_from_param_ = false;
  double a_ego_est_ = 0.0;
  double prev_v_ego_ = 0.0;
  int64_t prev_v_ts_us_ = 0;
  /// The controller's last command. Debug record only — the planner does not act on it.
  adas::proto::SteerCommand cmd_{};
  std::unique_ptr<lateral::IPlanner> solver_;
  double max_torque_cnm_ = 300.0;
  double road_roll_rad_ = 0.0;
  bool road_roll_valid_ = false;

  IntervalFilter frame_dt_;
  HysteresisGate speed_gate_;

  ChassisSample chassis_;
  bool have_chassis_ = false;
  LaneKeepOutput last_;
};

}  // namespace services

}  // namespace adas
