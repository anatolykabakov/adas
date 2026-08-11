#include "adas/utils/proto_convert.h"

namespace adas {

adas::proto::ZMQMessage createTrafficVision(const traffic::State& state, const traffic::Assessment& a, int64_t now_ms)
{
  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(now_ms);
  zmq.set_topic(topics::kTrafficVision);
  auto* s = zmq.mutable_traffic_vision();
  s->set_timestamp(now_ms);
  s->set_speed_limit_kmh(state.speed_limit_kmh);
  s->set_speed_limit_age_ms(a.speed_limit_age_ms);
  s->set_v_ego_kmh(static_cast<float>(a.v_kmh));
  s->set_overspeed(a.overspeed);
  s->set_overspeed_kmh(a.overspeed_kmh);
  s->set_tfl_color(static_cast<adas::proto::TrafficLightColor>(state.tfl_color));
  s->set_tfl_conf(state.tfl_conf);
  s->set_tfl_age_ms(a.tfl_age_ms);
  s->set_status(state.status);
  s->set_speed_limit_label(state.speed_limit_label);
  s->set_n_dets(state.n_dets);
  return zmq;
}

adas::proto::ZMQMessage createCameraCalibState(const CameraCalibrationState& state, int64_t timestamp_us)
{
  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(timestamp_us / 1000);
  zmq.set_topic(topics::kCameraCalib);
  auto* c = zmq.mutable_camera_calib();
  c->set_timestamp(timestamp_us / 1000);
  c->set_roll_deg(state.roll_deg);
  c->set_pitch_deg(state.pitch_deg);
  c->set_yaw_deg(state.yaw_deg);
  c->set_camera_height_m(state.camera_height_m);
  c->set_fx(state.fx);
  c->set_fy(state.fy);
  c->set_cx(state.cx);
  c->set_cy(state.cy);
  c->set_calibration_success(state.calibration_success);
  c->set_n_updates(state.n_updates);
  c->set_vp_u(state.vp_u);
  c->set_vp_v(state.vp_v);
  c->set_has_vp(state.has_vp);
  c->set_cal_percent(state.cal_percent);
  c->set_cal_status(state.cal_status);
  return zmq;
}

adas::proto::ZMQMessage createCameraCalibDebug(const CameraCalibrationState& state, const PoseCalibrator& pose,
                                               const VanishingPointCalibrator& vp, int64_t timestamp_us,
                                               const char* source)
{
  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(timestamp_us / 1000);
  zmq.set_topic(topics::kCameraCalibDebug);
  auto* d = zmq.mutable_camera_calib_debug();
  d->set_timestamp(timestamp_us / 1000);
  d->set_source(source ? source : "none");
  d->set_status(PoseCalibrator::statusName(pose.status()));
  d->set_cal_status(static_cast<int>(pose.status()));
  d->set_cal_percent(pose.calPercent());
  d->set_calibration_success(pose.calibrated() || (state.has_vp && vp.success()));
  d->set_roll_deg(state.roll_deg);
  d->set_pitch_deg(state.pitch_deg);
  d->set_yaw_deg(state.yaw_deg);
  d->set_height_m(state.camera_height_m);

  const auto& s = pose.lastSample();
  d->set_sample_accepted(s.accepted);
  d->set_v_ego(s.v_ego);
  d->set_odom_trans_x(s.odom_trans_x);
  d->set_odom_trans_y(s.odom_trans_y);
  d->set_odom_trans_z(s.odom_trans_z);
  d->set_odom_rot_z(s.odom_rot_z);
  d->set_odom_angle_std(s.odom_angle_std);
  d->set_gate_speed(s.gate_speed);
  d->set_gate_yaw_rate(s.gate_yaw_rate);
  d->set_gate_rpy_certain(s.gate_rpy_certain);
  d->set_gate_odom_valid(s.odom_valid);
  d->set_observed_pitch_deg(s.observed_pitch_deg);
  d->set_observed_yaw_deg(s.observed_yaw_deg);
  d->set_reject_reason(s.reject_reason ? s.reject_reason : "");

  const Vec3 spread = pose.calibSpread();
  d->set_spread_pitch_deg(spread.y() * 180.0 / M_PI);
  d->set_spread_yaw_deg(spread.z() * 180.0 / M_PI);
  d->set_spread_max_deg(spread.maxCoeff() * 180.0 / M_PI);
  d->set_valid_blocks(pose.validBlocks());
  d->set_block_idx(pose.blockIdx());
  d->set_sample_in_block(pose.sampleInBlock());
  d->set_old_rpy_weight(pose.oldRpyWeight());

  d->set_has_vp(vp.hasVp());
  d->set_vp_u(vp.vpU());
  d->set_vp_v(vp.vpV());
  d->set_vp_n_updates(vp.nUpdates());
  d->set_vp_history(vp.historySize());
  d->set_vp_success(vp.success());

  return zmq;
}

adas::proto::ZMQMessage createLongPlan(const longplan::Input& in, const longplan::Plan& plan, int64_t now_ms)
{
  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(now_ms);
  zmq.set_topic(topics::kLongPlan);
  auto* lp = zmq.mutable_long_plan();
  lp->set_timestamp(now_ms);
  lp->set_v_ego(in.v_ego);
  lp->set_v_target(plan.v_target);
  lp->set_a_target(plan.a_target);
  lp->set_lead_d(in.lead.d_rel);
  lp->set_lead_v(in.lead.v_lead);
  lp->set_lead_prob(in.lead.prob);
  lp->set_has_lead(plan.has_lead);
  lp->set_source(plan.source);
  lp->set_v_curv(plan.v_curv);
  lp->set_kappa_ahead(plan.kappa_ahead);
  lp->set_status(plan.status);
  return zmq;
}

adas::proto::ZMQMessage createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm)
{
  const int64_t publish_ms = static_cast<int64_t>(now_us) / 1000;
  const int64_t capture_ms = out.capture_ts_us / 1000;
  const int64_t vision_ms = out.vision_ts_us / 1000;
  const int64_t chassis_ms = out.chassis_ts_us / 1000;

  adas::proto::ZMQMessage lk_zmq;
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

  const int torque_cnm = static_cast<int>(std::lround(out.steer_norm * max_torque_cnm));
  lk->set_torque_cnm(torque_cnm);
  lk->set_torque_saturated(std::abs(torque_cnm) >= static_cast<int>(std::lround(0.99 * max_torque_cnm)));
  lk->set_capture_ts_ms(capture_ms);
  lk->set_vision_ts_ms(vision_ms);
  lk->set_chassis_ts_ms(chassis_ms);
  lk->set_publish_ts_ms(publish_ms);
  return lk_zmq;
}

adas::proto::ZMQMessage createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                            double frame_dt_s, const services::LaneKeep::Config& config)
{
  const int64_t publish_ms = static_cast<int64_t>(now_us) / 1000;

  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(publish_ms);
  zmq.set_topic(topics::kLaneKeepDebug);
  auto* d = zmq.mutable_lane_keep_debug();
  d->set_timestamp(publish_ms);
  d->set_controller(out.controller);
  d->set_status(out.status);
  d->set_has_target(out.has_target);

  d->set_speed_mps(out.dbg.speed_mps);
  d->set_n_points(out.dbg.n_points);
  d->set_cam_y_left_m(config.cam_y_left_m);

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
  d->set_torque_cnm(static_cast<int>(std::lround(out.steer_norm * max_torque_cnm)));
  d->set_steer_output_enabled(out.dbg.steer_output_enabled);
  d->set_assist_allowed(out.dbg.assist_allowed);
  d->set_assist_known(out.dbg.assist_known);
  d->set_frame_dt_ms(frame_dt_s * 1000.0);

  d->set_p_k_dd(config.pp_k_dd);
  d->set_p_ld_min(config.pp_ld_min);
  d->set_p_ld_max(config.pp_ld_max);
  d->set_p_ld_curv_gain(config.pp_ld_curv_gain);
  d->set_p_max_steer_deg(config.max_steer_deg);
  d->set_p_max_torque_cnm(static_cast<int>(std::lround(max_torque_cnm)));
  d->set_p_mpc_epsi_gain(config.mpc_epsi_gain);
  d->set_p_mpc_ff_scale(config.mpc_ff_scale);
  d->set_p_mpc_kappa_yaw_blend(config.mpc_kappa_yaw_blend);

  d->set_lane_anchored(out.dbg.lane_anchored);
  d->set_lanelines_active(out.dbg.lanelines_active);
  d->set_road_roll_deg(out.dbg.road_roll_deg);
  d->set_kappa_solver(out.dbg.kappa_solver);
  d->set_pid_p(out.dbg.pid_p);
  d->set_pid_i(out.dbg.pid_i);
  d->set_pid_f(out.dbg.pid_f);
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
  return zmq;
}

adas::proto::ZMQMessage createSteerCommand(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                           bool steer_output_enabled, bool have_desired)
{
  const int64_t publish_ms = static_cast<int64_t>(now_us) / 1000;
  const int64_t capture_ms = out.capture_ts_us / 1000;
  const int64_t vision_ms = out.vision_ts_us / 1000;
  const int64_t chassis_ms = out.chassis_ts_us / 1000;

  adas::proto::ZMQMessage zmq;
  zmq.set_timestamp(publish_ms);
  zmq.set_topic(topics::kSteerCommand);
  auto* cmd = zmq.mutable_steer_command();
  const int torque = static_cast<int>(std::lround(out.steer_norm * max_torque_cnm));
  const bool en = steer_output_enabled && out.has_target && out.status == "ok" && have_desired;
  cmd->set_torque_cnm(en ? torque : 0);
  cmd->set_enabled(en);
  cmd->set_capture_ts_ms(capture_ms);
  cmd->set_vision_ts_ms(vision_ms);
  cmd->set_chassis_ts_ms(chassis_ms);
  cmd->set_publish_ts_ms(publish_ms);
  return zmq;
}

LaneKeepOutput laneKeepFromProto(const adas::proto::LaneKeepState& p, int64_t timestamp_us)
{
  LaneKeepOutput o;
  o.timestamp_us = timestamp_us;
  o.steer_rad = p.steer_rad();
  o.steer_norm = p.steer_norm();
  o.lookahead_m = p.lookahead_m();
  o.target_x = p.target_x();
  o.target_y = p.target_y();
  o.has_target = p.has_target();
  o.curvature = p.curvature();
  o.status = p.status();
  // Published as "ok:mpc" / "ok:pp" (see LaneKeep::publishLaneKeep).
  const auto colon = o.status.find(':');
  if (colon != std::string::npos) {
    o.controller = o.status.substr(colon + 1);
    o.status = o.status.substr(0, colon);
  } else {
    o.controller = "pp";
  }
  if (o.controller == "mpc" || o.controller == "fp") {
    o.cte_m = o.target_x;
    o.epsi_rad = o.target_y;
  }
  return o;
}

LocalizationPose localizationFromProto(const adas::proto::LocalizationPose& p, int64_t timestamp_us)
{
  LocalizationPose o;
  o.timestamp_us = timestamp_us;
  o.x = p.x();
  o.y = p.y();
  o.yaw = p.yaw();
  o.v = p.v();
  o.yaw_rate = p.yaw_rate();
  o.odom_x = p.odom_x();
  o.odom_y = p.odom_y();
  o.ekf_x = p.ekf_x();
  o.ekf_y = p.ekf_y();
  return o;
}

CameraCalibrationState cameraCalibFromProto(const adas::proto::CameraCalibrationState& p, int64_t timestamp_us)
{
  CameraCalibrationState o;
  o.timestamp_us = timestamp_us;
  o.roll_deg = p.roll_deg();
  o.pitch_deg = p.pitch_deg();
  o.yaw_deg = p.yaw_deg();
  o.camera_height_m = p.camera_height_m();
  o.fx = p.fx();
  o.fy = p.fy();
  o.cx = p.cx();
  o.cy = p.cy();
  o.calibration_success = p.calibration_success();
  o.n_updates = p.n_updates();
  o.vp_u = p.vp_u();
  o.vp_v = p.vp_v();
  o.has_vp = p.has_vp();
  return o;
}

}  // namespace adas
