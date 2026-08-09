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

    /** Whether the rack was actually being given torque, as reported by the panda safety layer. */
    bool assist_allowed = false;
    /** False when no panda has reported yet, or when its last report has aged out. */
    bool assist_known = false;

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
    // Fitted over three drives; the previous 0.00006 under-delivered 4-8x at every speed.
    // See kFeedforwardFloorMps in utils/lat_control_pid.h.
    double pid_kf = 0.00015;
    double pid_ff_floor_mps = kFeedforwardFloorMps;
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

    /** Take `tire_stiffness_factor`, `steer_ratio` and the steering bias from `localization/pose` — the
     *  online estimate from `utils/params_learner.h` — instead of from this config.
     *
     *  Separate from `localization.learn_vehicle_params` on purpose. The learner can run and publish for a
     *  whole drive while the controller keeps using the constants; that is how the learned value earns the
     *  right to be used, by being logged next to the number it would replace. One flag for both would mean
     *  the first drive that tests the estimator is also the first drive that trusts it.
     *
     *  Even when on, the estimate is only taken while `learned_params_valid` holds — the learner's own
     *  gate on sample count and uncertainty — and the configured value is used until then and again
     *  immediately if validity is lost. */
    bool use_learned_params = false;

    double fp_steer_delay_s = 0.35;

    double min_control_speed_mps = 1.5;

    double min_control_speed_hyst_mps = 0.5;

    double vision_nominal_dt_s = 0.09;

    double lane_max_age_s = 0.30;
    /** Hand the wheel back while a turn signal is on: the driver is changing lanes and we have no
     *  lane-change planner (no `DesireHelper`), so holding the current lane fights them.
     *
     *  Signal source is `Gateway_72` (`0x3DB`), decoded into `ChassisSample::left/right_blinker`.
     *  Verified present on run 2026_08_04_21_00_18: left on 2.4 % of the run (7 episodes), right
     *  4.1 % (11 episodes) — the frame really arrives, so this gate is not dead code.
     *
     *  Deliberately *not* keyed on `steering_pressed`: that flag fired 520 times in 23.5 min with a
     *  median episode of 30 ms, so suppressing on it would drop the assist every few seconds. Small
     *  driver inputs are already handled by the panda's STEER_DRIVER_ALLOWANCE and by the PID
     *  integrator unwind. */
    bool lka_suppress_on_blinker = true;
    /** Keep the wheel handed back for this long after the signal cancels. Blinkers self-cancel when
     *  the wheel returns, i.e. mid-manoeuvre, so re-engaging on the falling edge would grab the wheel
     *  while the car is still crossing the line. */
    double lka_blinker_resume_delay_s = 1.0;

    /** Recompute the setpoint between vision frames instead of holding the last one.
     *
     *  Upstream calls `get_lag_adjusted_curvature` and `LaC.update` at 100 Hz on a plan that refreshes at
     *  20 Hz (`controlsd.py:730`); we solved once per frame and held the answer for 78 ms. Measured on their
     *  log against ours: median setpoint step 0.0067° per 10 ms there, 0.107° here, p99 **2.48°** and 3.78° at
     *  5–12 m/s. The rack does 200 cNm/s, so a step is a request it cannot follow and the response reads as
     *  lag — which is where the differential replay put the remaining disagreement, at 10–15 m/s.
     *
     *  Nothing is re-solved: only the two terms the speed enters, `psi/(v·delay)` and the curvature-rate
     *  ceiling. `fp` only — `pp` and `mpc` keep the once-per-frame setpoint, which is worth knowing when
     *  comparing controllers with this on. */
    bool lat_recompute_setpoint = false;

    /** Gate the angle PID on whether the rack is actually receiving torque.
     *
     *  Without this the loop cannot tell a followed command from a discarded one. On run
     *  2026_08_06_18_27_12 the panda withheld torque 70.7 % of the time — the stock VW cruise drops
     *  below ~30 km/h and on brake, and `controls_allowed` for MQB follows `TSK_06.TSK_Status ∈
     *  {3,4,5}` — while `steer_output_enabled` stayed true, the debug topic claimed the system was
     *  steering, and the integrator wound up against an error it could not influence. Upstream resets
     *  the controller whenever `not active` (`latcontrol_pid.py`); this is that gate.
     *
     *  Reporting matters as much as the reset: `dbg.assist_allowed` is what lets offline analysis stop
     *  averaging actuated and non-actuated frames together, which is what every lateral number in
     *  `docs/BACKLOG.md` did before this existed. */
    bool lat_require_assist = true;
    /** How long a panda report stays usable. `panda/health` publishes at 10 Hz, so this is five
     *  periods — long enough not to flap, short enough that a dead panda closes the gate. */
    double assist_max_age_s = 0.5;
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

  /** Whether torque is reaching the rack right now, and whether we know.
   *
   *  The two answers when we do not know are deliberately different. Having never heard from a panda
   *  means there is no panda in the loop — a bag replay, the Python bindings, a bench run — and gating
   *  there would silence the command in every offline harness we measure with. Having heard from one
   *  and then losing it means we are on the car and the device stopped talking, in which case it is
   *  not passing torque either. So: never seen, gate open; seen and aged out, gate closed. */
  bool assistPresent(int64_t now_us, bool& known) const;

  /** The `fp` curvature → steering-wheel angle chain, run again at the current speed. Reproduces exactly
   *  what `stepFlowpilot` does after the solve — vehicle model, the speed-dependent angle ceiling, the slew
   *  guard — because a recompute that skipped any of them would command something the once-per-frame path
   *  never would. Returns false when there is nothing to recompute from. */
  bool recomputeSetpoint(LaneKeepOutput& out);

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
  bool learned_valid_ = false;
  double learned_stiffness_ = 0.64;
  double learned_steer_ratio_ = 15.7;
  double learned_angle_offset_deg_ = 0.0;
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
  /** When a turn signal last went off; suppression persists for `lka_blinker_resume_delay_s`. */
  int64_t blinker_off_us_ = 0;
  bool blinker_off_armed_ = false;
  bool blinker_suppressed_ = false;
  /** Last `panda/health`: whether the safety layer allowed control, and when it said so. */
  bool assist_allowed_ = false;
  int64_t assist_ts_us_ = 0;
  bool have_assist_ = false;
  bool assist_absent_logged_ = false;
  int pub_gap_frames_ = 0;
  int64_t last_frame_ts_us_ = 0;
  double frame_dt_s_ = 0.09;
  int64_t last_chassis_ts_us_ = 0;
  int64_t last_pid_us_ = 0;
  bool speed_gate_open_ = false;
  double chassis_dt_s_ = 0.05;
};

}  // namespace adas
