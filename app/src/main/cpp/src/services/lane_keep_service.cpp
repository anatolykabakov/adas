#include "services/lane_keep_service.h"

#include <cmath>

#include "messages.pb.h"
#include "utils/logger.h"
#include "utils/protobuf_utils.h"

namespace adas {
namespace {

int64_t nowUs() { return utils::getCurrentTimestamp() * 1000; }

constexpr double kMinFrameDtS = 0.02;
constexpr double kMaxFrameDtS = 0.5;
constexpr double kMaxFrameGapS = 0.5;

}  // namespace

LaneKeepService::LaneKeepService(Config p)
  : config_(std::move(p))
  , pp_(config_.pp_k_dd, config_.wheelbase_m, config_.pp_shift, config_.pp_ld_min, config_.pp_ld_max,
        config_.pp_ld_curv_gain)
  , lat_(config_.pid_kp, config_.pid_ki, config_.pid_kf, 50.0)
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

void LaneKeepService::configure()
{
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<LanePathMsg>(topics::kVisionPath, [this](const LanePathMsg& m) { onLanes(m); });
  registerParameters();
  LOGI("LaneKeepService: controller=%s  ratio=%.1f  cam_y_left=%.3f  → %s / %s", config_.controller.c_str(),
       steer_ratio_, config_.cam_y_left_m, topics::kLaneKeep, topics::kSteerCommand);
}

void LaneKeepService::registerParameters()
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
}

void LaneKeepService::reset()
{
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
  frame_dt_s_ = config_.vision_nominal_dt_s;
  last_chassis_ts_us_ = 0;
}

void LaneKeepService::setController(std::string controller)
{
  if (controller == "mpc")
    config_.controller = "mpc";
  else if (controller == "fp" || controller == "flowpilot")
    config_.controller = "fp";
  else
    config_.controller = "pp";
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
  LOGI("LaneKeepService: controller → %s", config_.controller.c_str());
}

void LaneKeepService::setPurePursuit(double k_dd, double ld_min, double ld_max, double shift)
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

void LaneKeepService::setMaxSteerDeg(double max_steer_deg)
{
  config_.max_steer_deg = max_steer_deg;
  max_steer_rad_ = max_steer_deg * M_PI / 180.0;
}

double LaneKeepService::activeMaxSteerRad() const
{
  if (useMpcFamily())
    return std::min(config_.mpc_max_steer_deg, 25.0) * M_PI / 180.0;
  return max_steer_rad_;
}

double LaneKeepService::mpcMaxSteerRad(double speed_mps) const
{
  const double ceil_deg = std::min(config_.mpc_max_steer_deg, 25.0);
  const double lo_deg = std::clamp(config_.mpc_low_speed_steer_deg, 1.0, ceil_deg);
  const double slope = std::max(config_.mpc_steer_deg_per_mps, 0.0);
  const double v = std::max(0.0, speed_mps);
  const double lim_deg = std::clamp(lo_deg + slope * v, lo_deg, ceil_deg);
  return lim_deg * M_PI / 180.0;
}

void LaneKeepService::updateChassisDt(const ChassisSample& msg)
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

double LaneKeepService::emaAlpha(double alpha_at_nominal) const
{
  if (!(alpha_at_nominal < 1.0 - 1e-9) || alpha_at_nominal <= 0.0)
    return alpha_at_nominal;
  const double nominal = std::max(config_.vision_nominal_dt_s, 1e-3);
  const double tau = -nominal / std::log(1.0 - alpha_at_nominal);
  return std::clamp(1.0 - std::exp(-frame_dt_s_ / tau), 1e-3, 1.0);
}

void LaneKeepService::onChassis(const ChassisSample& msg)
{
  updateChassisDt(msg);
  chassis_ = msg;
  have_chassis_ = true;
  if (have_desired_)
    updateTorqueFromAngle();
}

double LaneKeepService::slipFactorOrZero() const
{
  if (!config_.lat_use_vehicle_model)
    return 0.0;
  VehicleModelParams p;
  p.wheelbase_m = config_.wheelbase_m;
  p.tire_stiffness_factor = config_.tire_stiffness_factor;
  return slipFactor(p);
}

void LaneKeepService::updateFrameDt(const LanePathMsg& msg)
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

void LaneKeepService::onLanes(const LanePathMsg& msg)
{
  const double speed = have_chassis_ ? chassis_.speed_mps : 0.0;
  auto out = step(speed, msg);
  out.dbg.lane_anchored = msg.lane_anchored;
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
    desired_swa_deg_ = steer_sign_ * (out.steer_rad * 180.0 / M_PI) * steer_ratio_;
    have_desired_ = true;
  } else {
    have_desired_ = false;
    desired_swa_deg_ = 0.0;
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

void LaneKeepService::updateTorqueFromAngle()
{
  LaneKeepOutput out = last_;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : out.chassis_ts_us;

  const int64_t now_us = nowUs();
  if (last_pid_us_ > 0) {
    const double dt = std::clamp(static_cast<double>(now_us - last_pid_us_) * 1e-6, 0.005, 0.2);
    lat_.setRate(1.0 / dt);
  }
  last_pid_us_ = now_us;

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

  const bool active = steer_output_enabled_ && have_desired_ && have_chassis_ && out.has_target && out.status == "ok";
  const auto lat =
      lat_.update(active, desired_swa_deg_, chassis_.steering_angle_deg, chassis_.speed_mps, chassis_.steering_pressed);

  out.desired_swa_deg = lat.angle_des_deg;
  out.actual_swa_deg = lat.angle_act_deg;
  out.angle_error_deg = lat.angle_error_deg;
  out.steer_norm = lat.steer_norm;
  last_ = out;
  publishSteer(out);
}

void LaneKeepService::publishLaneKeep(const LaneKeepOutput& out)
{
  const int64_t publish_ms = nowUs() / 1000;
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

void LaneKeepService::publishLaneKeepDebug(const LaneKeepOutput& out)
{
  const int64_t publish_ms = nowUs() / 1000;

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
  d->set_frame_dt_ms(frame_dt_s_ * 1000.0);

  d->set_p_k_dd(config_.pp_k_dd);
  d->set_p_ld_min(config_.pp_ld_min);
  d->set_p_ld_max(config_.pp_ld_max);
  d->set_p_ld_curv_gain(config_.pp_ld_curv_gain);
  d->set_p_max_steer_deg(config_.max_steer_deg);
  d->set_p_mpc_epsi_gain(config_.mpc_epsi_gain);
  d->set_p_mpc_ff_scale(config_.mpc_ff_scale);
  d->set_p_mpc_kappa_yaw_blend(config_.mpc_kappa_yaw_blend);

  d->set_lane_anchored(out.dbg.lane_anchored);
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

void LaneKeepService::publishSteer(const LaneKeepOutput& out)
{
  const int64_t publish_ms = nowUs() / 1000;
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

LaneKeepOutput LaneKeepService::step(double speed_mps, const LanePathMsg& path)
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

  auto run = [this, speed_mps, &path](const std::vector<Vec2>& poly) {
    if (useFlowpilot()) {
      LanePathMsg p = path;
      p.polyline = poly;
      return stepFlowpilot(speed_mps, p);
    }
    if (useMpc())
      return stepMpc(speed_mps, poly);
    return stepPp(speed_mps, poly);
  };

  if (std::abs(config_.cam_y_left_m) < 1e-12)
    return run(path.polyline);

  std::vector<Vec2> shifted = path.polyline;
  for (auto& p : shifted)
    p.y() -= config_.cam_y_left_m;
  return run(shifted);
}

LaneKeepOutput LaneKeepService::stepPp(double speed_mps, const std::vector<Vec2>& polyline_ego)
{
  LaneKeepOutput out;
  out.controller = "pp";
  out.dbg.speed_mps = speed_mps;
  out.dbg.n_points = static_cast<int>(polyline_ego.size());
  if (polyline_ego.size() < 2) {
    out.status = "no_polyline";
    return out;
  }

  const auto pp = pp_.compute(polyline_ego, speed_mps);
  out.steer_rad = pp.steer_rad;
  out.dbg.pp_steer_raw_rad = pp.steer_rad;
  const double lim = activeMaxSteerRad();
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

LaneKeepOutput LaneKeepService::stepMpc(double speed_mps, const std::vector<Vec2>& polyline_ego)
{
  LaneKeepOutput out;
  out.controller = "mpc";
  out.dbg.speed_mps = speed_mps;
  out.dbg.n_points = static_cast<int>(polyline_ego.size());

  const auto lat = estimatePathLateralState(polyline_ego);
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

  const double Lf = config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m;

  double cte = lat.cte_m;
  double epsi = lat.epsi_rad;
  const double cte_alpha = emaAlpha(config_.mpc_cte_ema_alpha);
  if (cte_alpha < 1.0 - 1e-6) {
    cte_ema_ = cte_ema_init_ ? (cte_alpha * cte + (1.0 - cte_alpha) * cte_ema_) : cte;
    cte_ema_init_ = true;
    cte = cte_ema_;
  }
  const double epsi_alpha = emaAlpha(config_.mpc_epsi_ema_alpha);
  if (epsi_alpha < 1.0 - 1e-6) {
    epsi_ema_ = epsi_ema_init_ ? (epsi_alpha * epsi + (1.0 - epsi_alpha) * epsi_ema_) : epsi;
    epsi_ema_init_ = true;
    epsi = epsi_ema_;
  }
  out.cte_m = cte;
  out.epsi_rad = epsi;
  out.target_x = cte;
  out.target_y = epsi;

  double kappa = lat.kappa;
  const double alpha = config_.mpc_kappa_yaw_blend;
  if (alpha > 1e-6 && have_chassis_ && speed_mps > config_.mpc_kappa_yaw_min_speed) {
    const double kappa_yaw = chassis_.yaw_rate / speed_mps;
    out.dbg.mpc_kappa_yaw = kappa_yaw;
    kappa = (1.0 - alpha) * lat.kappa + alpha * kappa_yaw;
  }
  const double kappa_alpha = emaAlpha(config_.mpc_kappa_ema_alpha);
  if (kappa_alpha < 1.0 - 1e-6) {
    kappa_ema_ = kappa_ema_init_ ? (kappa_alpha * kappa + (1.0 - kappa_alpha) * kappa_ema_) : kappa;
    kappa_ema_init_ = true;
    kappa = kappa_ema_;
  }
  out.curvature = kappa;
  out.dbg.mpc_kappa_used = kappa;

  const double dkappa_ds = kappa_rate_.update(kappa, speed_mps, frame_dt_s_);
  out.dbg.mpc_dkappa_ds = dkappa_ds;
  const auto kappa_sched = visionpilot::build_kappa_schedule(Lf, epsi, kappa, dkappa_ds);

  Eigen::VectorXd v_sched(static_cast<int>(visionpilot::N));
  for (int i = 0; i < (int)visionpilot::N; ++i)
    v_sched[i] = speed_mps;

  Eigen::VectorXd state(3);
  state << cte, epsi, kappa;
  const auto deltas = mpc_.compute_steering(Lf, state, v_sched, kappa_sched);

  double delta_vp = 0.0;
  if (deltas.size() > 1)
    delta_vp = deltas[1];
  else if (!deltas.empty())
    delta_vp = deltas[0];

  out.steer_rad = -delta_vp;
  out.dbg.mpc_delta_vp_rad = out.steer_rad;
  const double lim = mpcMaxSteerRad(speed_mps);
  out.dbg.mpc_max_steer_rad = lim;
  if (lim > 1e-6)
    out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
  out.dbg.mpc_delta_clamped_rad = out.steer_rad;

  if (config_.mpc_rate_limit_deg > 1e-9 && have_mpc_prev_) {
    const double v_eff = std::max(speed_mps, config_.mpc_rate_min_speed);
    const double dkappa_max = config_.mpc_max_lateral_jerk / (v_eff * v_eff) * frame_dt_s_;
    const double rate_ceil = config_.mpc_rate_limit_deg * M_PI / 180.0;
    const double rate = std::min(dkappa_max * Lf, rate_ceil);
    const double d = std::clamp(out.steer_rad - last_mpc_steer_rad_, -rate, rate);
    out.steer_rad = last_mpc_steer_rad_ + d;
  }
  last_mpc_steer_rad_ = out.steer_rad;
  have_mpc_prev_ = true;

  out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
  out.has_target = true;
  out.status = "ok";
  return out;
}

LaneKeepOutput LaneKeepService::stepFlowpilot(double speed_mps, const LanePathMsg& path)
{
  LaneKeepOutput out;
  out.controller = "fp";
  const auto& polyline_ego = path.polyline;
  out.dbg.speed_mps = speed_mps;
  out.dbg.n_points = static_cast<int>(polyline_ego.size());

  const auto lat = estimatePathLateralState(polyline_ego);
  out.cte_m = lat.cte_m;
  out.epsi_rad = lat.epsi_rad;
  out.curvature = lat.kappa;
  out.dbg.mpc_kappa_path = lat.kappa;
  out.target_x = lat.cte_m;
  out.target_y = lat.epsi_rad;

  if (polyline_ego.size() < 4) {
    out.status = "no_polyline";
    return out;
  }

  const double Lf = config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m;
  flowpilot::LatMpcConfig fcfg;
  fcfg.max_lateral_jerk = config_.mpc_max_lateral_jerk;
  fcfg.steering_rate_weight = std::max(0.0, config_.fp_steering_rate_weight);
  fcfg.steer_delay_s = std::max(0.05, config_.fp_steer_delay_s);

  fcfg.rotation_radius = std::max(0.0, 0.5 * config_.wheelbase_m);
  fp_mpc_.setConfig(fcfg);

  std::vector<Vec2> plan_poly = path.plan_poly;
  if (std::abs(config_.cam_y_left_m) > 1e-12) {
    for (auto& p : plan_poly)
      p.y() -= config_.cam_y_left_m;
  }

  const double yaw = have_chassis_ ? chassis_.yaw_rate : 0.0;
  const auto sol =
      fp_mpc_.update(speed_mps, yaw, Lf, polyline_ego, plan_poly, path.plan_yaw, path.plan_yaw_rate, frame_dt_s_);
  if (!sol.ok) {
    out.status = "bad_fit";
    return out;
  }

  const double kappa_cmd = sol.desired_curvature;

  out.curvature = kappa_cmd;
  out.dbg.mpc_kappa_used = kappa_cmd;
  out.dbg.mpc_dkappa_ds = sol.desired_curvature_rate;

  out.steer_rad = -steerFromCurvature(kappa_cmd, speed_mps, Lf, slipFactorOrZero());
  out.dbg.mpc_delta_vp_rad = out.steer_rad;
  const double lim = mpcMaxSteerRad(speed_mps);
  out.dbg.mpc_max_steer_rad = lim;
  if (lim > 1e-6)
    out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
  out.dbg.mpc_delta_clamped_rad = out.steer_rad;

  if (config_.steer_slew_limit_deg > 1e-9 && have_mpc_prev_) {
    const double ceil = config_.steer_slew_limit_deg * M_PI / 180.0;
    const double d = std::clamp(out.steer_rad - last_mpc_steer_rad_, -ceil, ceil);
    out.steer_rad = last_mpc_steer_rad_ + d;
  }
  last_mpc_steer_rad_ = out.steer_rad;
  have_mpc_prev_ = true;

  out.steer_norm = lim > 1e-6 ? out.steer_rad / lim : 0.0;
  out.has_target = true;
  out.status = "ok";
  return out;
}

}  // namespace adas
