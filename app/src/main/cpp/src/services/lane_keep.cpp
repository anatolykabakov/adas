#include "adas/services/lane_keep.h"

#include <cmath>

#include "messages.pb.h"
#include "adas/utils/logger.h"
#include "adas/utils/protobuf_utils.h"

namespace adas {
namespace services {
namespace {

constexpr double kMinFrameDtS = 0.02;
constexpr double kMaxFrameDtS = 0.5;
constexpr double kMaxFrameGapS = 0.5;

}  // namespace

LaneKeep::LaneKeep(Config p)
  : config_(std::move(p))
  , pp_(config_.pp_k_dd, config_.wheelbase_m, config_.pp_shift, config_.pp_ld_min, config_.pp_ld_max,
        config_.pp_ld_curv_gain)
  , lat_(config_.pid_kp, config_.pid_ki, config_.pid_kf, 50.0, config_.pid_ff_floor_mps)
  , max_steer_rad_(config_.max_steer_deg * M_PI / 180.0)
  , max_torque_cnm_(config_.max_torque_cnm)
  , steer_ratio_(std::max(config_.steer_ratio, 1e-3))
  , steer_sign_(config_.steer_sign < 0.0 ? -1.0 : 1.0)
  , steer_output_enabled_(config_.steer_output_enabled)
{
  if (config_.controller != "mpc" && config_.controller != "fp")
    config_.controller = "pp";
  visionpilot::set_warm_start_gains(config_.mpc_epsi_gain, config_.mpc_ff_scale);
  visionpilot::set_cost_weights(config_.mpc_cte_weight_base, config_.mpc_cte_quartic_scale);
  visionpilot::set_cte_gain_base(config_.mpc_cte_gain_base);
  visionpilot::set_cte_gain_floor(config_.mpc_cte_gain_floor);
}

void LaneKeep::configure()
{
  makeSolver();
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<LanePathMsg>(topics::kVisionPath, [this](const LanePathMsg& m) { onLanes(m); });
  if (config_.use_learned_params || config_.roll_compensation) {
    subscribe<ai::flow::adas::ZMQMessage>(topics::kLocalizationPose, [this](const ai::flow::adas::ZMQMessage& m) {
      if (!m.has_localization_pose())
        return;
      const auto& p = m.localization_pose();
      if (config_.use_learned_params) {
        setLearnedParams(p.learned_params_valid(), p.learned_stiffness_factor(), p.learned_steer_ratio(),
                         p.learned_angle_offset_deg());
      }
      road_roll_valid_ = p.road_roll_valid();
      road_roll_rad_ = road_roll_valid_ ? p.road_roll_deg() * M_PI / 180.0 : 0.0;
    });
  }
  subscribe<ai::flow::adas::ZMQMessage>(topics::kPandaHealth, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_panda_health())
      return;
    assist_allowed_ = m.panda_health().lat_actuation_allowed();
    assist_ts_us_ = static_cast<int64_t>(now());
    have_assist_ = true;
  });
  registerParameters();
  LOGI("LaneKeep: controller=%s  ratio=%.1f  cam_y_left=%.3f  → %s / %s", config_.controller.c_str(), steer_ratio_,
       config_.cam_y_left_m, topics::kLaneKeep, topics::kSteerCommand);
}

void LaneKeep::registerParameters()
{
  registerParameter<double>(
      "steer_ratio", [this](const double& v) { setSteerRatio(v); }, [this] { return steer_ratio_; });
  registerParameter<double>(
      "max_steer_deg", [this](const double& v) { setMaxSteerDeg(v); }, [this] { return config_.max_steer_deg; });
  registerParameter<double>(
      "cam_y_left_m", [this](const double& v) { setCamYLeftM(v); }, [this] { return config_.cam_y_left_m; });
  registerParameter<std::string>(
      "lane_keep_controller", [this](const std::string& v) { setController(v); },
      [this] { return config_.controller; });
  registerParameter<double>(
      "pp_k_dd", [this](const double& v) { setPurePursuit(v, config_.pp_ld_min, config_.pp_ld_max, config_.pp_shift); },
      [this] { return config_.pp_k_dd; });
  registerParameter<double>(
      "pp_ld_min", [this](const double& v) { setPurePursuit(config_.pp_k_dd, v, config_.pp_ld_max, config_.pp_shift); },
      [this] { return config_.pp_ld_min; });
  registerParameter<double>(
      "pp_ld_max", [this](const double& v) { setPurePursuit(config_.pp_k_dd, config_.pp_ld_min, v, config_.pp_shift); },
      [this] { return config_.pp_ld_max; });
  registerParameter<double>(
      "pp_shift", [this](const double& v) { setPurePursuit(config_.pp_k_dd, config_.pp_ld_min, config_.pp_ld_max, v); },
      [this] { return config_.pp_shift; });
  registerParameter<double>(
      "pp_ld_curv_gain", [this](const double& v) { setPpLdCurvGain(v); }, [this] { return config_.pp_ld_curv_gain; });
  registerParameter<double>(
      "steer_slew_limit_deg", [this](const double& v) { setSteerSlewLimitDeg(v); },
      [this] { return config_.steer_slew_limit_deg; });
  registerParameter<double>(
      "tire_stiffness_factor", [this](const double& v) { setVehicleModel(config_.lat_use_vehicle_model, v); },
      [this] { return config_.tire_stiffness_factor; });
  registerParameter<bool>(
      "lat_use_vehicle_model", [this](const bool& v) { setVehicleModel(v, config_.tire_stiffness_factor); },
      [this] { return config_.lat_use_vehicle_model; });
  registerParameter<double>(
      "fp_steer_delay_s", [this](const double& v) { setFpSteerDelayS(v); },
      [this] { return config_.fp_steer_delay_s; });
  registerParameter<double>(
      "fp_steering_rate_weight", [this](const double& v) { setFpSteeringRateWeight(v); },
      [this] { return config_.fp_steering_rate_weight; });
  registerParameter<double>("lane_max_age_s", config_.lane_max_age_s);
  registerParameter<bool>("lka_suppress_on_blinker", config_.lka_suppress_on_blinker);
  registerParameter<double>("lka_blinker_resume_delay_s", config_.lka_blinker_resume_delay_s);
  registerParameter<bool>("lat_recompute_setpoint", config_.lat_recompute_setpoint);
  registerParameter<bool>("lat_require_assist", config_.lat_require_assist);
  registerParameter<double>("assist_max_age_s", config_.assist_max_age_s);
}

void LaneKeep::reset()
{
  if (solver_)
    solver_->reset();
  last_ = LaneKeepOutput{};
  have_chassis_ = false;
  have_desired_ = false;
  desired_swa_deg_ = 0.0;
  lat_.reset();
  kappa_rate_.reset();
  fp_mpc_.reset();
  kappa_ema_init_ = false;
  epsi_ema_init_ = false;
  cte_ema_init_ = false;
  last_mpc_steer_rad_ = 0.0;
  have_mpc_prev_ = false;
  last_pub_steer_rad_ = 0.0;
  have_pub_prev_ = false;
  last_frame_ts_us_ = 0;
  ref_stale_ = false;
  blinker_off_us_ = 0;
  blinker_off_armed_ = false;
  blinker_suppressed_ = false;
  assist_allowed_ = false;
  assist_ts_us_ = 0;
  have_assist_ = false;
  assist_absent_logged_ = false;
  frame_dt_s_ = config_.vision_nominal_dt_s;
  last_chassis_ts_us_ = 0;
}

bool LaneKeep::recomputeSetpoint(LaneKeepOutput& out)
{
  if (!useFlowpilot() || !have_chassis_ || !have_desired_)
    return false;
  const auto k = fp_mpc_.curvatureAtSpeed(chassis_.speed_mps, frame_dt_s_);
  if (!k)
    return false;

  const double Lf = config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m;
  double steer =
      -steerFromCurvature(curvatureWithRoll(*k, chassis_.speed_mps), chassis_.speed_mps, Lf, slipFactorOrZero());
  const double lim = mpcMaxSteerRad(chassis_.speed_mps);
  if (lim > 1e-6)
    steer = std::clamp(steer, -lim, lim);
  // The slew guard is per call, and at 100 Hz that is seven times more headroom per frame than the
  // once-per-frame path had. Left as is rather than rescaled because 8 degrees of *road wheel* is a sanity
  // bound, not a shaping filter: it is about 125 degrees at the steering wheel and never binds in normal
  // driving. If it starts binding, the interesting question is what produced the jump, not the ceiling.
  if (config_.steer_slew_limit_deg > 1e-9 && have_mpc_prev_) {
    const double ceil = config_.steer_slew_limit_deg * M_PI / 180.0;
    steer = last_mpc_steer_rad_ + std::clamp(steer - last_mpc_steer_rad_, -ceil, ceil);
  }
  last_mpc_steer_rad_ = steer;
  have_mpc_prev_ = true;

  desired_swa_no_offset_deg_ = steer_sign_ * (steer * 180.0 / M_PI) * effectiveSteerRatio();
  desired_swa_deg_ = desired_swa_no_offset_deg_ + effectiveAngleOffsetDeg();
  // Written into the caller's `out`, not into `last_`: `updateTorqueFromAngle` copies `last_` before this
  // runs and assigns it back afterwards, so touching `last_` here would be silently discarded — and the bag
  // would report the once-per-frame setpoint while the PID chased the recomputed one. That is exactly the
  // class of defect this session has been removing, so it is worth the parameter.
  out.steer_rad = steer;
  out.curvature = *k;
  out.dbg.mpc_kappa_used = *k;
  out.dbg.mpc_max_steer_rad = lim;
  out.dbg.mpc_delta_clamped_rad = steer;
  return true;
}

bool LaneKeep::assistPresent(int64_t now_us, bool& known) const
{
  if (!have_assist_) {
    known = false;
    return true;
  }
  const double age_s = static_cast<double>(now_us - assist_ts_us_) * 1e-6;
  if (config_.assist_max_age_s > 0.0 && age_s > config_.assist_max_age_s) {
    known = false;
    return false;
  }
  known = true;
  return assist_allowed_;
}

void LaneKeep::setController(std::string controller)
{
  if (controller == "mpc")
    config_.controller = "mpc";
  else if (controller == "fp_acados")
    config_.controller = "fp_acados";
  else if (controller == "fp" || controller == "flowpilot")
    config_.controller = "fp";
  else
    config_.controller = "pp";
  makeSolver();
  kappa_rate_.reset();
  lat_.reset();
  fp_mpc_.reset();
  kappa_ema_init_ = false;
  epsi_ema_init_ = false;
  cte_ema_init_ = false;
  last_mpc_steer_rad_ = 0.0;
  have_mpc_prev_ = false;
  last_pub_steer_rad_ = 0.0;
  have_pub_prev_ = false;
  last_frame_ts_us_ = 0;
  frame_dt_s_ = config_.vision_nominal_dt_s;
  last_chassis_ts_us_ = 0;
  LOGI("LaneKeep: controller → %s", solverName());
}

void LaneKeep::setPurePursuit(double k_dd, double ld_min, double ld_max, double shift)
{
  pp_.K_dd = k_dd;
  pp_.ld_min = ld_min;
  pp_.ld_max = ld_max;
  pp_.waypoint_shift = shift;
  config_.pp_k_dd = k_dd;
  config_.pp_ld_min = ld_min;
  config_.pp_ld_max = ld_max;
  config_.pp_shift = shift;
}

void LaneKeep::setMaxSteerDeg(double max_steer_deg)
{
  config_.max_steer_deg = max_steer_deg;
  max_steer_rad_ = max_steer_deg * M_PI / 180.0;
}

double LaneKeep::activeMaxSteerRad() const
{
  if (useMpcFamily())
    return std::min(config_.mpc_max_steer_deg, 25.0) * M_PI / 180.0;
  return max_steer_rad_;
}

double LaneKeep::mpcMaxSteerRad(double speed_mps) const
{
  const double ceil_deg = std::min(config_.mpc_max_steer_deg, 25.0);
  const double lo_deg = std::clamp(config_.mpc_low_speed_steer_deg, 1.0, ceil_deg);
  const double slope = std::max(config_.mpc_steer_deg_per_mps, 0.0);
  const double v = std::max(0.0, speed_mps);
  const double lim_deg = std::clamp(lo_deg + slope * v, lo_deg, ceil_deg);
  return lim_deg * M_PI / 180.0;
}

void LaneKeep::updateChassisDt(const ChassisSample& msg)
{
  const int64_t ts = msg.timestamp_us;
  if (ts <= 0)
    return;
  if (last_chassis_ts_us_ > 0) {
    const double raw = static_cast<double>(ts - last_chassis_ts_us_) * 1e-6;
    if (raw > 0.0 && raw <= kMaxFrameGapS) {
      constexpr double kAlpha = 0.3;
      chassis_dt_s_ = kAlpha * std::clamp(raw, kMinFrameDtS, kMaxFrameDtS) + (1.0 - kAlpha) * chassis_dt_s_;
      lat_.setRate(1.0 / chassis_dt_s_);
    }
  }
  last_chassis_ts_us_ = ts;
}

double LaneKeep::emaAlpha(double alpha_at_nominal) const
{
  if (!(alpha_at_nominal < 1.0 - 1e-9) || alpha_at_nominal <= 0.0)
    return alpha_at_nominal;
  const double nominal = std::max(config_.vision_nominal_dt_s, 1e-3);
  const double tau = -nominal / std::log(1.0 - alpha_at_nominal);
  return std::clamp(1.0 - std::exp(-frame_dt_s_ / tau), 1e-3, 1.0);
}

void LaneKeep::onChassis(const ChassisSample& msg)
{
  updateChassisDt(msg);
  chassis_ = msg;
  have_chassis_ = true;
  if (have_desired_)
    updateTorqueFromAngle();
}

double LaneKeep::curvatureWithRoll(double kappa, double speed_mps) const
{
  if (!config_.roll_compensation || !road_roll_valid_)
    return kappa;
  return kappa - rollCompensationCurvature(road_roll_rad_, speed_mps, slipFactorOrZero());
}

double LaneKeep::slipFactorOrZero() const
{
  if (!config_.lat_use_vehicle_model)
    return 0.0;
  VehicleModelParams p;
  p.wheelbase_m = config_.wheelbase_m;
  p.tire_stiffness_factor = effectiveStiffnessFactor();
  return slipFactor(p);
}

void LaneKeep::updateFrameDt(const LanePathMsg& msg)
{
  const int64_t ts = msg.capture_ts_us > 0 ? msg.capture_ts_us : msg.timestamp_us;
  if (ts <= 0) {
    frame_dt_s_ = config_.vision_nominal_dt_s;
    return;
  }
  if (last_frame_ts_us_ > 0) {
    const double raw = static_cast<double>(ts - last_frame_ts_us_) * 1e-6;
    if (raw > 0.0 && raw <= kMaxFrameGapS) {
      constexpr double kAlpha = 0.3;
      const double clamped = std::clamp(raw, kMinFrameDtS, kMaxFrameDtS);
      frame_dt_s_ = kAlpha * clamped + (1.0 - kAlpha) * frame_dt_s_;
    } else {
      frame_dt_s_ = config_.vision_nominal_dt_s;
    }
  }
  last_frame_ts_us_ = ts;
}

void LaneKeep::onLanes(const LanePathMsg& msg)
{
  const double speed = have_chassis_ ? chassis_.speed_mps : 0.0;
  auto out = step(speed, msg);
  out.dbg.lane_anchored = msg.lane_anchored;
  out.dbg.lanelines_active = msg.lanelines_active;
  out.dbg.road_roll_deg = road_roll_valid_ ? road_roll_rad_ * 180.0 / M_PI : 0.0;
  out.dbg.lane_width_m = msg.lane_width_m;
  out.dbg.lane_offset_m = msg.lane_offset_m;
  out.dbg.center_force_m = msg.center_force_m;
  out.dbg.p_lane_blend_scale = msg.p_lane_blend_scale;
  out.dbg.p_camera_offset_m = msg.p_camera_offset_m;
  out.dbg.p_center_force_gain = msg.p_center_force_gain;
  out.capture_ts_us = msg.capture_ts_us > 0 ? msg.capture_ts_us : (msg.timestamp_us > 0 ? msg.timestamp_us : 0);
  out.vision_ts_us = msg.infer_ts_us > 0 ? msg.infer_ts_us : 0;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : 0;

  const bool slew_guard = config_.steer_slew_limit_deg > 1e-9;
  if (out.has_target && out.status == "ok" && slew_guard && have_pub_prev_) {
    const double Lf = config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m;
    const double v_eff = std::max(speed, config_.mpc_rate_min_speed);
    const double dkappa_max = config_.mpc_max_lateral_jerk / (v_eff * v_eff) * frame_dt_s_;
    const double ceil = config_.steer_slew_limit_deg * M_PI / 180.0;
    const double slew = std::min(dkappa_max * Lf, ceil);
    const double d = std::clamp(out.steer_rad - last_pub_steer_rad_, -slew, slew);
    if (d != out.steer_rad - last_pub_steer_rad_) {
      out.steer_rad = last_pub_steer_rad_ + d;
      const double lim = useMpcFamily() ? mpcMaxSteerRad(speed) : activeMaxSteerRad();
      out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
      out.dbg.slew_clipped = true;
    }
  }

  constexpr int kMaxPubGapFrames = 10;
  if (out.has_target && out.status == "ok") {
    last_pub_steer_rad_ = out.steer_rad;
    have_pub_prev_ = true;
    pub_gap_frames_ = 0;
  } else if (have_pub_prev_ && ++pub_gap_frames_ > kMaxPubGapFrames) {
    have_pub_prev_ = false;
  }

  if (out.has_target && out.status == "ok") {
    // The learned angle offset is where the steering column actually reads zero, so it is added to the
    // commanded angle rather than subtracted: the learner solved `delta = (SWA - offset) / ratio`, and this
    // is that relation run the other way.
    desired_swa_no_offset_deg_ = steer_sign_ * (out.steer_rad * 180.0 / M_PI) * effectiveSteerRatio();
    desired_swa_deg_ = desired_swa_no_offset_deg_ + effectiveAngleOffsetDeg();
    have_desired_ = true;
  } else {
    have_desired_ = false;
    desired_swa_deg_ = 0.0;
    desired_swa_no_offset_deg_ = 0.0;
    lat_.reset();
  }

  out.desired_swa_deg = desired_swa_deg_;
  if (have_chassis_) {
    out.actual_swa_deg = chassis_.steering_angle_deg;
    out.angle_error_deg = desired_swa_deg_ - chassis_.steering_angle_deg;
  }
  out.dbg.steer_output_enabled = steer_output_enabled_;
  last_ = out;
  publishLaneKeep(last_);

  updateTorqueFromAngle();
  publishLaneKeepDebug(last_);
}

void LaneKeep::updateTorqueFromAngle()
{
  LaneKeepOutput out = last_;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : out.chassis_ts_us;

  const int64_t now_us = static_cast<int64_t>(now());
  if (last_pid_us_ > 0) {
    const double dt = std::clamp(static_cast<double>(now_us - last_pid_us_) * 1e-6, 0.005, 0.2);
    lat_.setRate(1.0 / dt);
  }
  last_pid_us_ = now_us;

  if (config_.lat_recompute_setpoint)
    recomputeSetpoint(out);

  if (config_.lane_max_age_s > 0.0 && out.capture_ts_us > 0 && out.status == "ok") {
    const double age_s = static_cast<double>(now_us - out.capture_ts_us) * 1e-6;
    if (age_s > config_.lane_max_age_s) {
      out.status = "stale";
      if (!ref_stale_) {
        LOGW("LaneKeep: path stale (%.0f ms > %.0f) — command cleared", age_s * 1e3, config_.lane_max_age_s * 1e3);
        ref_stale_ = true;
      }
    } else if (ref_stale_) {
      LOGI("LaneKeep: path fresh again (%.0f ms)", age_s * 1e3);
      ref_stale_ = false;
    }
  }

  if (config_.lka_suppress_on_blinker && have_chassis_) {
    const bool on = chassis_.left_blinker || chassis_.right_blinker;
    if (on) {
      blinker_off_armed_ = false;
    } else if (!blinker_off_armed_ && blinker_suppressed_) {
      blinker_off_us_ = now_us;  // falling edge — start the resume timer
      blinker_off_armed_ = true;
    }
    const double since_off_s = blinker_off_armed_ ? static_cast<double>(now_us - blinker_off_us_) * 1e-6 : 0.0;
    const bool hold = blinker_off_armed_ && since_off_s < std::max(0.0, config_.lka_blinker_resume_delay_s);
    if (on || hold) {
      if (out.status == "ok")
        out.status = "blinker";
      if (!blinker_suppressed_) {
        LOGI("LaneKeep: turn signal (%s) — wheel handed back", chassis_.left_blinker ? "left" : "right");
        blinker_suppressed_ = true;
      }
    } else {
      // Cleared symmetrically with the way it is set, as the actuation gate below does. `out` is a copy
      // of `last_` and this runs on chassis ticks, several times per vision frame: without clearing,
      // "blinker" would survive until the next frame, or forever if frames stop, holding the command at
      // zero after the log already said steering had resumed.
      if (out.status == "blinker")
        out.status = "ok";
      if (blinker_suppressed_) {
        LOGI("LaneKeep: turn signal off for %.1f s — steering again", since_off_s);
        blinker_suppressed_ = false;
        blinker_off_armed_ = false;
      }
    }
  }

  // Whether the rack is actually being given torque. Checked last of the three gates so that our own
  // withdrawals — a stale reference, a turn signal — report first: if we chose not to steer, whether the
  // assist was there is moot. `dbg.assist_allowed` carries the answer regardless of what the status says,
  // which is what offline analysis needs.
  bool assist_known = false;
  const bool assist = assistPresent(now_us, assist_known);
  out.dbg.assist_allowed = assist_known && assist;
  out.dbg.assist_known = assist_known;
  if (config_.lat_require_assist) {
    // Cleared as well as set: `out` is a copy of the previous tick, and between vision frames this runs
    // ~7 times, so escalate-only would hold the status for a whole frame after the assist returned.
    if (!assist && out.status == "ok")
      out.status = "no_assist";
    else if (assist && out.status == "no_assist")
      out.status = "ok";
    if (!assist && !assist_absent_logged_) {
      LOGW("LaneKeep: panda is not passing torque (controls_allowed=%d, known=%d) — command cleared, PID reset",
           static_cast<int>(assist_allowed_), static_cast<int>(assist_known));
      assist_absent_logged_ = true;
    } else if (assist && assist_absent_logged_) {
      LOGI("LaneKeep: torque reaching the rack again");
      assist_absent_logged_ = false;
    }
  }

  const bool active = steer_output_enabled_ && have_desired_ && have_chassis_ && out.has_target && out.status == "ok" &&
                      (!config_.lat_require_assist || assist);
  const auto lat = lat_.update(active, desired_swa_deg_, chassis_.steering_angle_deg, chassis_.speed_mps,
                               chassis_.steering_pressed, desired_swa_no_offset_deg_);

  out.desired_swa_deg = lat.angle_des_deg;
  out.actual_swa_deg = lat.angle_act_deg;
  out.angle_error_deg = lat.angle_error_deg;
  out.steer_norm = lat.steer_norm;
  last_ = out;
  publishSteer(out);
}

void LaneKeep::publishLaneKeep(const LaneKeepOutput& out)
{
  const int64_t publish_ms = static_cast<int64_t>(now()) / 1000;
  const int64_t capture_ms = out.capture_ts_us / 1000;
  const int64_t vision_ms = out.vision_ts_us / 1000;
  const int64_t chassis_ms = out.chassis_ts_us / 1000;

  ai::flow::adas::ZMQMessage lk_zmq;
  lk_zmq.set_timestamp(publish_ms);
  lk_zmq.set_topic(topics::kLaneKeep);
  auto* lk = lk_zmq.mutable_lane_keep();
  lk->set_timestamp(publish_ms);
  lk->set_steer_rad(out.steer_rad);
  lk->set_steer_norm(out.steer_norm);
  lk->set_throttle(0.0);
  lk->set_brake(0.0);
  lk->set_lookahead_m(out.lookahead_m);
  lk->set_target_x(out.target_x);
  lk->set_target_y(out.target_y);
  lk->set_has_target(out.has_target);
  lk->set_curvature(out.curvature);
  lk->set_status(out.status + ":" + out.controller);

  const int torque_cnm = static_cast<int>(std::lround(out.steer_norm * max_torque_cnm_));
  lk->set_torque_cnm(torque_cnm);
  lk->set_torque_saturated(std::abs(torque_cnm) >= static_cast<int>(std::lround(0.99 * max_torque_cnm_)));
  lk->set_capture_ts_ms(capture_ms);
  lk->set_vision_ts_ms(vision_ms);
  lk->set_chassis_ts_ms(chassis_ms);
  lk->set_publish_ts_ms(publish_ms);
  publish(topics::kLaneKeep, lk_zmq);
}

void LaneKeep::publishLaneKeepDebug(const LaneKeepOutput& out)
{
  const int64_t publish_ms = static_cast<int64_t>(now()) / 1000;

  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(publish_ms);
  zmq.set_topic(topics::kLaneKeepDebug);
  auto* d = zmq.mutable_lane_keep_debug();
  d->set_timestamp(publish_ms);
  d->set_controller(out.controller);
  d->set_status(out.status);
  d->set_has_target(out.has_target);

  d->set_speed_mps(out.dbg.speed_mps);
  d->set_n_points(out.dbg.n_points);
  d->set_cam_y_left_m(config_.cam_y_left_m);

  d->set_pp_lookahead_m(out.lookahead_m);
  d->set_pp_target_x(out.target_x);
  d->set_pp_target_y(out.target_y);
  d->set_pp_curvature(out.curvature);
  d->set_pp_steer_raw_rad(out.dbg.pp_steer_raw_rad);

  d->set_mpc_cte_m(out.cte_m);
  d->set_mpc_epsi_rad(out.epsi_rad);
  d->set_mpc_kappa_path(out.dbg.mpc_kappa_path);
  d->set_mpc_kappa_yaw(out.dbg.mpc_kappa_yaw);
  d->set_mpc_kappa_used(out.dbg.mpc_kappa_used);
  d->set_mpc_dkappa_ds(out.dbg.mpc_dkappa_ds);
  d->set_mpc_delta_vp_rad(out.dbg.mpc_delta_vp_rad);
  d->set_mpc_delta_clamped_rad(out.dbg.mpc_delta_clamped_rad);
  d->set_mpc_max_steer_rad(out.dbg.mpc_max_steer_rad);

  d->set_steer_rad(out.steer_rad);
  d->set_steer_norm(out.steer_norm);
  d->set_slew_clipped(out.dbg.slew_clipped);
  d->set_max_steer_rad(out.dbg.max_steer_rad);
  d->set_desired_swa_deg(out.desired_swa_deg);
  d->set_actual_swa_deg(out.actual_swa_deg);
  d->set_angle_error_deg(out.angle_error_deg);
  d->set_torque_cnm(static_cast<int>(std::lround(out.steer_norm * max_torque_cnm_)));
  d->set_steer_output_enabled(out.dbg.steer_output_enabled);
  d->set_assist_allowed(out.dbg.assist_allowed);
  d->set_assist_known(out.dbg.assist_known);
  d->set_frame_dt_ms(frame_dt_s_ * 1000.0);

  d->set_p_k_dd(config_.pp_k_dd);
  d->set_p_ld_min(config_.pp_ld_min);
  d->set_p_ld_max(config_.pp_ld_max);
  d->set_p_ld_curv_gain(config_.pp_ld_curv_gain);
  d->set_p_max_steer_deg(config_.max_steer_deg);
  d->set_p_max_torque_cnm(static_cast<int>(std::lround(max_torque_cnm_)));
  d->set_p_mpc_epsi_gain(config_.mpc_epsi_gain);
  d->set_p_mpc_ff_scale(config_.mpc_ff_scale);
  d->set_p_mpc_kappa_yaw_blend(config_.mpc_kappa_yaw_blend);

  d->set_lane_anchored(out.dbg.lane_anchored);
  d->set_lanelines_active(out.dbg.lanelines_active);
  d->set_road_roll_deg(out.dbg.road_roll_deg);
  d->set_lane_width_m(out.dbg.lane_width_m);
  d->set_lane_offset_m(out.dbg.lane_offset_m);
  d->set_center_force_m(out.dbg.center_force_m);
  d->set_p_lane_blend_scale(out.dbg.p_lane_blend_scale);
  d->set_p_camera_offset_m(out.dbg.p_camera_offset_m);
  d->set_p_center_force_gain(out.dbg.p_center_force_gain);

  d->set_capture_ts_ms(out.capture_ts_us / 1000);
  d->set_vision_ts_ms(out.vision_ts_us / 1000);
  d->set_chassis_ts_ms(out.chassis_ts_us / 1000);
  d->set_publish_ts_ms(publish_ms);

  publish(topics::kLaneKeepDebug, zmq);
}

void LaneKeep::publishSteer(const LaneKeepOutput& out)
{
  const int64_t publish_ms = static_cast<int64_t>(now()) / 1000;
  const int64_t capture_ms = out.capture_ts_us / 1000;
  const int64_t vision_ms = out.vision_ts_us / 1000;
  const int64_t chassis_ms = out.chassis_ts_us / 1000;

  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(publish_ms);
  zmq.set_topic(topics::kSteerCommand);
  auto* cmd = zmq.mutable_steer_command();
  const int torque = static_cast<int>(std::lround(out.steer_norm * max_torque_cnm_));
  const bool en = steer_output_enabled_ && out.has_target && out.status == "ok" && have_desired_;
  cmd->set_torque_cnm(en ? torque : 0);
  cmd->set_enabled(en);
  cmd->set_capture_ts_ms(capture_ms);
  cmd->set_vision_ts_ms(vision_ms);
  cmd->set_chassis_ts_ms(chassis_ms);
  cmd->set_publish_ts_ms(publish_ms);
  publish(topics::kSteerCommand, zmq);
}

class LaneKeep::PpSolver final : public lateral::Solver {
public:
  explicit PpSolver(LaneKeep& o) : o_(o) {}
  const char* name() const override { return "pp"; }
  LaneKeepOutput solve(const lateral::SolverInput& in) override
  {
    const auto& poly = in.path->polyline;
    LaneKeepOutput out;
    out.controller = "pp";
    out.dbg.speed_mps = in.speed_mps;
    out.dbg.n_points = static_cast<int>(poly.size());
    if (poly.size() < 2) {
      out.status = "no_polyline";
      return out;
    }

    const auto pp = o_.pp_.compute(poly, in.speed_mps);
    out.steer_rad = pp.steer_rad;
    out.dbg.pp_steer_raw_rad = pp.steer_rad;
    const double lim = o_.activeMaxSteerRad();
    out.dbg.max_steer_rad = lim;
    if (lim > 1e-6)
      out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
    out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
    out.lookahead_m = pp.lookahead_m;
    out.curvature = pp.curvature();
    if (pp.target_ego) {
      out.has_target = true;
      out.target_x = pp.target_ego->x();
      out.target_y = pp.target_ego->y();
    }
    out.status = "ok";
    return out;
  }

private:
  LaneKeep& o_;
};

class LaneKeep::VisionPilotSolver final : public lateral::Solver {
public:
  explicit VisionPilotSolver(LaneKeep& o) : o_(o) {}
  const char* name() const override { return "mpc"; }
  void reset() override { o_.kappa_rate_.reset(); }
  LaneKeepOutput solve(const lateral::SolverInput& in) override
  {
    const auto& poly = in.path->polyline;
    LaneKeepOutput out;
    out.controller = "mpc";
    out.dbg.speed_mps = in.speed_mps;
    out.dbg.n_points = static_cast<int>(poly.size());

    const auto lat = estimatePathLateralState(poly);
    out.cte_m = lat.cte_m;
    out.epsi_rad = lat.epsi_rad;
    out.curvature = lat.kappa;
    out.dbg.mpc_kappa_path = lat.kappa;
    out.target_x = lat.cte_m;
    out.target_y = lat.epsi_rad;

    if (!lat.valid) {
      out.status = lat.n_points < 5 ? "no_polyline" : "bad_fit";
      return out;
    }

    const double Lf = o_.config_.mpc_Lf > 1e-3 ? o_.config_.mpc_Lf : o_.config_.wheelbase_m;

    double cte = lat.cte_m;
    double epsi = lat.epsi_rad;
    const double cte_alpha = o_.emaAlpha(o_.config_.mpc_cte_ema_alpha);
    if (cte_alpha < 1.0 - 1e-6) {
      o_.cte_ema_ = o_.cte_ema_init_ ? (cte_alpha * cte + (1.0 - cte_alpha) * o_.cte_ema_) : cte;
      o_.cte_ema_init_ = true;
      cte = o_.cte_ema_;
    }
    const double epsi_alpha = o_.emaAlpha(o_.config_.mpc_epsi_ema_alpha);
    if (epsi_alpha < 1.0 - 1e-6) {
      o_.epsi_ema_ = o_.epsi_ema_init_ ? (epsi_alpha * epsi + (1.0 - epsi_alpha) * o_.epsi_ema_) : epsi;
      o_.epsi_ema_init_ = true;
      epsi = o_.epsi_ema_;
    }
    out.cte_m = cte;
    out.epsi_rad = epsi;
    out.target_x = cte;
    out.target_y = epsi;

    double kappa = lat.kappa;
    const double alpha = o_.config_.mpc_kappa_yaw_blend;
    if (alpha > 1e-6 && o_.have_chassis_ && in.speed_mps > o_.config_.mpc_kappa_yaw_min_speed) {
      const double kappa_yaw = o_.chassis_.yaw_rate / in.speed_mps;
      out.dbg.mpc_kappa_yaw = kappa_yaw;
      kappa = (1.0 - alpha) * lat.kappa + alpha * kappa_yaw;
    }
    const double kappa_alpha = o_.emaAlpha(o_.config_.mpc_kappa_ema_alpha);
    if (kappa_alpha < 1.0 - 1e-6) {
      o_.kappa_ema_ = o_.kappa_ema_init_ ? (kappa_alpha * kappa + (1.0 - kappa_alpha) * o_.kappa_ema_) : kappa;
      o_.kappa_ema_init_ = true;
      kappa = o_.kappa_ema_;
    }
    out.curvature = kappa;
    out.dbg.mpc_kappa_used = kappa;

    const double dkappa_ds = o_.kappa_rate_.update(kappa, in.speed_mps, o_.frame_dt_s_);
    out.dbg.mpc_dkappa_ds = dkappa_ds;
    const auto kappa_sched = visionpilot::build_kappa_schedule(Lf, epsi, kappa, dkappa_ds);

    Eigen::VectorXd v_sched(static_cast<int>(visionpilot::N));
    for (int i = 0; i < (int)visionpilot::N; ++i)
      v_sched[i] = in.speed_mps;

    Eigen::VectorXd state(3);
    state << cte, epsi, kappa;
    const auto deltas = o_.mpc_.compute_steering(Lf, state, v_sched, kappa_sched);

    double delta_vp = 0.0;
    if (deltas.size() > 1)
      delta_vp = deltas[1];
    else if (!deltas.empty())
      delta_vp = deltas[0];

    out.steer_rad = -delta_vp;
    out.dbg.mpc_delta_vp_rad = out.steer_rad;
    const double lim = o_.mpcMaxSteerRad(in.speed_mps);
    out.dbg.mpc_max_steer_rad = lim;
    if (lim > 1e-6)
      out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
    out.dbg.mpc_delta_clamped_rad = out.steer_rad;

    if (o_.config_.mpc_rate_limit_deg > 1e-9 && o_.have_mpc_prev_) {
      const double v_eff = std::max(in.speed_mps, o_.config_.mpc_rate_min_speed);
      const double dkappa_max = o_.config_.mpc_max_lateral_jerk / (v_eff * v_eff) * o_.frame_dt_s_;
      const double rate_ceil = o_.config_.mpc_rate_limit_deg * M_PI / 180.0;
      const double rate = std::min(dkappa_max * Lf, rate_ceil);
      const double d = std::clamp(out.steer_rad - o_.last_mpc_steer_rad_, -rate, rate);
      out.steer_rad = o_.last_mpc_steer_rad_ + d;
    }
    o_.last_mpc_steer_rad_ = out.steer_rad;
    o_.have_mpc_prev_ = true;

    out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
    out.has_target = true;
    out.status = "ok";
    return out;
  }

private:
  LaneKeep& o_;
};

class LaneKeep::FlowPilotSolver final : public lateral::Solver {
public:
  FlowPilotSolver(LaneKeep& o, std::unique_ptr<lateral::KappaSolver> kappa) : o_(o), kappa_(std::move(kappa)) {}
  const char* name() const override { return "fp"; }
  const char* kappaSolverName() const { return kappa_->name(); }
  void reset() override { kappa_->reset(); }
  LaneKeepOutput solve(const lateral::SolverInput& in) override
  {
    LaneKeepOutput out;
    out.controller = "fp";
    const auto& poly = in.path->polyline;
    out.dbg.speed_mps = in.speed_mps;
    out.dbg.n_points = static_cast<int>(poly.size());

    const auto lat = estimatePathLateralState(poly);
    out.cte_m = lat.cte_m;
    out.epsi_rad = lat.epsi_rad;
    out.curvature = lat.kappa;
    out.dbg.mpc_kappa_path = lat.kappa;
    out.target_x = lat.cte_m;
    out.target_y = lat.epsi_rad;

    if (poly.size() < 4) {
      out.status = "no_polyline";
      return out;
    }

    const double Lf = o_.config_.mpc_Lf > 1e-3 ? o_.config_.mpc_Lf : o_.config_.wheelbase_m;
    flowpilot::LatMpcConfig fcfg;
    fcfg.max_lateral_jerk = o_.config_.mpc_max_lateral_jerk;
    fcfg.steering_rate_weight = std::max(0.0, o_.config_.fp_steering_rate_weight);
    fcfg.steer_delay_s = std::max(0.05, o_.config_.fp_steer_delay_s);

    fcfg.rotation_radius = std::max(0.0, 0.5 * o_.config_.wheelbase_m);
    o_.fp_mpc_.setConfig(fcfg);

    std::vector<Vec2> plan_poly = in.path->plan_poly;
    if (std::abs(o_.config_.cam_y_left_m) > 1e-12) {
      for (auto& p : plan_poly)
        p.y() -= o_.config_.cam_y_left_m;
    }

    const double yaw = o_.have_chassis_ ? o_.chassis_.yaw_rate : 0.0;

    double kappa_cmd = 0.0;
    double kappa_rate = 0.0;
    if (!kappa_->solve(in, poly, plan_poly, yaw, Lf, kappa_cmd, kappa_rate)) {
      out.status = "bad_fit";
      return out;
    }

    out.curvature = kappa_cmd;
    out.dbg.mpc_kappa_used = kappa_cmd;
    out.dbg.mpc_dkappa_ds = kappa_rate;

    out.steer_rad =
        -steerFromCurvature(o_.curvatureWithRoll(kappa_cmd, in.speed_mps), in.speed_mps, Lf, o_.slipFactorOrZero());
    out.dbg.mpc_delta_vp_rad = out.steer_rad;
    const double lim = o_.mpcMaxSteerRad(in.speed_mps);
    out.dbg.mpc_max_steer_rad = lim;
    if (lim > 1e-6)
      out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
    out.dbg.mpc_delta_clamped_rad = out.steer_rad;

    if (o_.config_.steer_slew_limit_deg > 1e-9 && o_.have_mpc_prev_) {
      const double ceil = o_.config_.steer_slew_limit_deg * M_PI / 180.0;
      const double d = std::clamp(out.steer_rad - o_.last_mpc_steer_rad_, -ceil, ceil);
      out.steer_rad = o_.last_mpc_steer_rad_ + d;
    }
    o_.last_mpc_steer_rad_ = out.steer_rad;
    o_.have_mpc_prev_ = true;

    out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
    out.has_target = true;
    out.status = "ok";
    return out;
  }

private:
  LaneKeep& o_;
  std::unique_ptr<lateral::KappaSolver> kappa_;
};

class LaneKeep::GradKappaSolver final : public lateral::KappaSolver {
public:
  explicit GradKappaSolver(LaneKeep& o) : o_(o) {}
  const char* name() const override { return "grad"; }
  void reset() override { o_.fp_mpc_.reset(); }
  bool solve(const lateral::SolverInput& in, const std::vector<Vec2>& poly, const std::vector<Vec2>& plan_poly,
             double yaw, double Lf, double& kappa, double& kappa_rate) override
  {
    const auto sol = o_.fp_mpc_.update(in.speed_mps, yaw, Lf, poly, plan_poly, in.path->plan_yaw,
                                       in.path->plan_yaw_rate, o_.frame_dt_s_);
    if (!sol.ok)
      return false;
    kappa = sol.desired_curvature;
    kappa_rate = sol.desired_curvature_rate;
    return true;
  }

private:
  LaneKeep& o_;
};

#ifdef ADAS_WITH_ACADOS
class LaneKeep::AcadosKappaSolver final : public lateral::KappaSolver {
public:
  explicit AcadosKappaSolver(LaneKeep& o) : o_(o) {}
  const char* name() const override { return "acados"; }
  bool available() const override { return o_.acados_mpc_.available(); }
  void reset() override { o_.acados_mpc_.reset(); }
  bool solve(const lateral::SolverInput& in, const std::vector<Vec2>& poly, const std::vector<Vec2>& plan_poly,
             double yaw, double Lf, double& kappa, double& kappa_rate) override
  {
    (void)Lf;
    std::vector<double> y_ref, psi_ref, r_ref;
    if (!flowpilot::LateralMpc::sampleRefs(poly, in.speed_mps, plan_poly, in.path->plan_yaw, in.path->plan_yaw_rate,
                                           y_ref, psi_ref, r_ref))
      return false;
    o_.acados_mpc_.setWeights({1.0, 0.11, 0.0, 0.05, std::max(0.0, o_.config_.fp_steering_rate_weight)});
    const auto a = o_.acados_mpc_.solve(in.speed_mps, std::max(0.0, 0.5 * o_.config_.wheelbase_m), yaw, y_ref, psi_ref,
                                        r_ref, std::max(0.05, o_.config_.fp_steer_delay_s));
    if (!a.ok)
      return false;
    kappa = a.desired_curvature;
    kappa_rate = a.desired_curvature_rate;
    return true;
  }

private:
  LaneKeep& o_;
};
#endif

void LaneKeep::makeSolver()
{
  if (config_.controller == "mpc") {
    solver_ = std::make_unique<VisionPilotSolver>(*this);
    return;
  }
  if (config_.controller != "fp") {
    solver_ = std::make_unique<PpSolver>(*this);
    return;
  }

  std::unique_ptr<lateral::KappaSolver> kappa;
#ifdef ADAS_WITH_ACADOS
  if (config_.fp_solver == "acados") {
    auto a = std::make_unique<AcadosKappaSolver>(*this);
    if (a->available())
      kappa = std::move(a);
    else
      LOGW("LaneKeep: fp_solver=acados недоступен, работает grad");
  }
#endif
  if (!kappa)
    kappa = std::make_unique<GradKappaSolver>(*this);
  solver_ = std::make_unique<FlowPilotSolver>(*this, std::move(kappa));
}

const char* LaneKeep::solverName() const { return solver_ ? solver_->name() : config_.controller.c_str(); }

const char* LaneKeep::kappaSolverName() const
{
  const auto* fp = dynamic_cast<const FlowPilotSolver*>(solver_.get());
  return fp ? fp->kappaSolverName() : "";
}

LaneKeepOutput LaneKeep::step(double speed_mps, const LanePathMsg& path)
{
  const double gate_on = config_.min_control_speed_mps + std::max(0.0, config_.min_control_speed_hyst_mps);
  if (speed_gate_open_ ? (speed_mps < config_.min_control_speed_mps) : (speed_mps < gate_on)) {
    if (speed_gate_open_) {
      fp_mpc_.reset();
      kappa_rate_.reset();
      lat_.reset();
      kappa_ema_init_ = false;
      have_mpc_prev_ = false;
      have_pub_prev_ = false;
    }
    speed_gate_open_ = false;
    LaneKeepOutput out;
    out.controller = config_.controller;
    out.status = "low_speed";
    out.dbg.speed_mps = speed_mps;
    return out;
  }
  speed_gate_open_ = true;

  updateFrameDt(path);
  if (!solver_)
    makeSolver();

  lateral::SolverInput in;
  in.speed_mps = speed_mps;
  in.yaw_rate = have_chassis_ ? chassis_.yaw_rate : 0.0;
  in.have_chassis = have_chassis_;
  in.frame_dt_s = frame_dt_s_;

  if (std::abs(config_.cam_y_left_m) < 1e-12) {
    in.path = &path;
    return solver_->solve(in);
  }

  LanePathMsg shifted = path;
  for (auto& p : shifted.polyline)
    p.y() -= config_.cam_y_left_m;
  in.path = &shifted;
  return solver_->solve(in);
}

}  // namespace services
}  // namespace adas
