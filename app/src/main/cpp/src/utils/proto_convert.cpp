#include "adas/utils/proto_convert.h"

namespace utils {
adas::proto::CANData createCANMessage(const std::vector<can_frame>& frames, int64_t now_ms)
{
  const int64_t current_timestamp = now_ms;

  adas::proto::CANData msg;
  auto* can_data = &msg;
  can_data->set_timestamp(current_timestamp);

  for (const auto& frame : frames) {
    auto* can_frame = can_data->add_frames();
    can_frame->set_address(frame.address);
    can_frame->set_data(frame.dat);
    can_frame->set_bus_time(frame.busTime);
    can_frame->set_src(frame.src);
  }

  return msg;
}

adas::proto::PandaHealth createHealthMessage(const health_t& health, int64_t now_ms, bool ignition,
                                             bool lat_actuation_allowed)
{
  const int64_t current_timestamp = now_ms;

  adas::proto::PandaHealth msg;
  auto* health_data = &msg;
  health_data->set_timestamp(current_timestamp);

  health_data->set_uptime_pkt(health.uptime_pkt);

  health_data->set_controls_allowed(health.controls_allowed_pkt);
  health_data->set_safety_mode(health.safety_mode_pkt);
  health_data->set_safety_param(health.safety_param_pkt);
  health_data->set_fault_status(health.fault_status_pkt);

  health_data->set_voltage_mv(health.voltage_pkt);
  health_data->set_current_ma(health.current_pkt);
  health_data->set_power_save_enabled(health.power_save_enabled_pkt);

  health_data->set_tx_blocked(health.safety_tx_blocked_pkt);
  health_data->set_tx_overflow(health.tx_buffer_overflow_pkt);
  health_data->set_rx_invalid(health.safety_rx_invalid_pkt);
  health_data->set_rx_overflow(health.rx_buffer_overflow_pkt);
  health_data->set_rx_checks_invalid(health.safety_rx_checks_invalid_pkt);

  health_data->set_faults_pkt(health.faults_pkt);
  health_data->set_spi_error_count(health.spi_checksum_error_count_pkt);

  health_data->set_ignition_line(health.ignition_line_pkt);
  health_data->set_ignition_can(health.ignition_can_pkt);

  health_data->set_car_harness_status(health.car_harness_status_pkt);

  health_data->set_heartbeat_lost(health.heartbeat_lost_pkt);

  health_data->set_alternative_experience(health.alternative_experience_pkt);

  health_data->set_interrupt_load(health.interrupt_load_pkt);

  health_data->set_fan_power(health.fan_power);

  health_data->set_sbu1_voltage_mv(health.sbu1_voltage_mV);
  health_data->set_sbu2_voltage_mv(health.sbu2_voltage_mV);
  health_data->set_som_reset_triggered(health.som_reset_triggered != 0);

  msg.set_ignition(ignition);
  msg.set_lat_actuation_allowed(lat_actuation_allowed);
  return msg;
}

adas::proto::CarState createCarStateMessage(const adas::proto::CarState& state) { return state; }

}  // namespace utils

namespace adas {
adas::proto::TrafficVisionState createTrafficVision(const traffic::State& state, const traffic::Assessment& a,
                                                    int64_t now_ms)
{
  adas::proto::TrafficVisionState msg;
  auto* s = &msg;
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
  return msg;
}

adas::proto::CameraCalibrationState createCameraCalibState(const CameraCalibrationState& state, int64_t timestamp_us)
{
  adas::proto::CameraCalibrationState msg;
  auto* c = &msg;
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
  return msg;
}

adas::proto::CameraCalibDebug createCameraCalibDebug(const CameraCalibrationState& state, const PoseCalibrator& pose,
                                                     const VanishingPointCalibrator& vp, int64_t timestamp_us,
                                                     const char* source)
{
  adas::proto::CameraCalibDebug msg;
  auto* d = &msg;
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

  return msg;
}

adas::proto::LongPlanState createLongPlan(const longplan::Input& in, const longplan::Plan& plan, int64_t now_ms)
{
  adas::proto::LongPlanState msg;
  auto* lp = &msg;
  lp->set_timestamp(now_ms);
  lp->set_v_ego(static_cast<float>(in.v_ego));
  lp->set_v_target(static_cast<float>(plan.v_target));
  lp->set_a_target(static_cast<float>(plan.a_target));
  lp->set_lead_d(static_cast<float>(in.lead.d_rel));
  lp->set_lead_v(static_cast<float>(in.lead.v_lead));
  lp->set_lead_prob(static_cast<float>(in.lead.prob));
  lp->set_has_lead(plan.has_lead);
  lp->set_source(plan.source);
  lp->set_v_curv(static_cast<float>(plan.v_curv));
  lp->set_kappa_ahead(static_cast<float>(plan.kappa_ahead));
  lp->set_status(plan.status);
  return msg;
}

adas::proto::LaneKeepState createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm)
{
  const int64_t publish_ms = static_cast<int64_t>(now_us) / 1000;
  const int64_t capture_ms = out.capture_ts_us / 1000;
  const int64_t vision_ms = out.vision_ts_us / 1000;
  const int64_t chassis_ms = out.chassis_ts_us / 1000;

  adas::proto::LaneKeepState msg;
  auto* lk = &msg;
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
  return msg;
}

adas::proto::LaneKeepDebug createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                               double frame_dt_s, const services::Planner::Config& config)
{
  const int64_t publish_ms = static_cast<int64_t>(now_us) / 1000;

  adas::proto::LaneKeepDebug msg;
  auto* d = &msg;
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
  return msg;
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
  // Road bank and the learned parameters: the publishing side fills them, and they used to be dropped
  // here — so everything reading the pose through `InternalSubscriber`, replays and pyadas alike, saw
  // zeros where the controller was already driving on learned numbers.
  o.road_roll_deg = p.road_roll_deg();
  o.road_roll_std_deg = p.road_roll_std_deg();
  o.road_roll_valid = p.road_roll_valid();
  o.learned_stiffness_factor = p.learned_stiffness_factor();
  o.learned_steer_ratio = p.learned_steer_ratio();
  o.learned_angle_offset_deg = p.learned_angle_offset_deg();
  o.learned_stiffness_std = p.learned_stiffness_std();
  o.learned_steer_ratio_std = p.learned_steer_ratio_std();
  o.learned_params_valid = p.learned_params_valid();
  o.learned_sample_count = p.learned_sample_count();
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

adas::proto::MiddlewareStats createMiddlewareStats(const adas::middleware::MiddlewareSnapshot& snap)
{
  const int64_t ts_ms = static_cast<int64_t>(snap.timestamp_us / 1000ULL);
  adas::proto::MiddlewareStats msg;
  auto* st = &msg;
  st->set_timestamp(ts_ms);
  st->set_dropped_total(snap.dropped_total);
  st->set_services(snap.services);
  st->set_running(snap.running);
  st->set_any_lagging(snap.any_lagging);

  for (const auto& s : snap.services_timing) {
    auto* t = st->add_services_timing();
    t->set_name(s.name);
    t->set_running(s.running);
    t->set_messages_processed(s.messages_processed);
    t->set_timers_fired(s.timers_fired);
    t->set_exceptions(s.exceptions);
    t->set_dropped(s.dropped);
    t->set_inbox_depth(s.inbox_depth);
    t->set_backlog_depth(s.backlog_depth);
    t->set_last_cb_ms(s.last_cb_ms);
    t->set_mean_cb_ms(s.mean_cb_ms);
    t->set_max_cb_ms(s.max_cb_ms);
    t->set_period_ms(s.period_ms);
    t->set_last_dt_ms(s.last_dt_ms);
    t->set_mean_dt_ms(s.mean_dt_ms);
    t->set_max_dt_ms(s.max_dt_ms);
    t->set_lagging(s.lagging);
    for (const auto& tm : s.timers) {
      auto* tt = t->add_timers();
      tt->set_name(tm.name);
      tt->set_period_ms(tm.period_ms);
      tt->set_last_dt_ms(tm.last_dt_ms);
      tt->set_mean_dt_ms(tm.mean_dt_ms);
      tt->set_max_dt_ms(tm.max_dt_ms);
      tt->set_lagging(tm.lagging);
      tt->set_fired(tm.fired);
    }
  }

  return msg;
}

adas::proto::LocalizationPose createLocalizationPose(const LocalizationPose& pose, int64_t timestamp_us)
{
  adas::proto::LocalizationPose msg;
  auto* p = &msg;
  p->set_timestamp(timestamp_us / 1000);
  p->set_x(pose.x);
  p->set_y(pose.y);
  p->set_yaw(pose.yaw);
  p->set_road_roll_deg(pose.road_roll_deg);
  p->set_road_roll_std_deg(pose.road_roll_std_deg);
  p->set_road_roll_valid(pose.road_roll_valid);
  p->set_learned_stiffness_factor(pose.learned_stiffness_factor);
  p->set_learned_steer_ratio(pose.learned_steer_ratio);
  p->set_learned_angle_offset_deg(pose.learned_angle_offset_deg);
  p->set_learned_stiffness_std(pose.learned_stiffness_std);
  p->set_learned_steer_ratio_std(pose.learned_steer_ratio_std);
  p->set_learned_params_valid(pose.learned_params_valid);
  p->set_learned_sample_count(pose.learned_sample_count);
  p->set_v(pose.v);
  p->set_yaw_rate(pose.yaw_rate);
  p->set_odom_x(pose.odom_x);
  p->set_odom_y(pose.odom_y);
  p->set_ekf_x(pose.ekf_x);
  p->set_ekf_y(pose.ekf_y);
  return msg;
}

adas::proto::SafetyWarnState createSafetyWarn(const safety::PlannerInput& in, const safety::SafetyPlan& plan,
                                              const SafetyWarnFlags& w, int64_t ms)
{
  adas::proto::SafetyWarnState msg;
  auto* sw = &msg;
  sw->set_timestamp(ms);
  sw->set_accel_ms2(static_cast<float>(plan.acceleration_ms2));
  sw->set_cte_m(static_cast<float>(in.lateral.cte_m));
  sw->set_epsi_rad(static_cast<float>(in.lateral.epsi_rad));
  sw->set_kappa(static_cast<float>(in.lateral.kappa));
  sw->set_lateral_valid(in.lateral.valid);
  sw->set_v_ego(static_cast<float>(in.ego_speed_ms));
  sw->set_lead_d(static_cast<float>(w.lead_d));
  sw->set_lead_v(static_cast<float>(w.lead_v));
  sw->set_lead_prob(static_cast<float>(w.lead_prob));
  sw->set_has_lead(w.has_lead);
  sw->set_fcw(w.fcw);
  sw->set_aeb(w.aeb);
  sw->set_lldw(w.lldw);
  sw->set_rldw(w.rldw);
  sw->set_cte_rate_ms(static_cast<float>(in.lateral.cte_rate_ms));
  sw->set_ttc_s(static_cast<float>(plan.threat.valid ? plan.threat.ttc_s : 0.0));
  sw->set_a_req_ms2(static_cast<float>(plan.threat.valid ? plan.threat.a_req_ms2 : 0.0));
  sw->set_threat_valid(plan.threat.valid);
  sw->set_driver_steering(in.driver_steering);
  sw->set_lane_anchored(in.lateral.lane_anchored);
  sw->set_status(w.status);
  return msg;
}

ChassisSample carStateToChassis(const adas::proto::CarState& cs, double steer_ratio)
{
  ChassisSample s;
  s.timestamp_us = cs.timestamp() * 1000;
  s.speed_mps = cs.v_ego();
  s.yaw_rate = cs.yaw_rate();
  s.steering_angle_deg = cs.steering_angle_deg();
  s.steering_pressed = cs.steering_pressed();
  s.left_blinker = cs.left_blinker();
  s.right_blinker = cs.right_blinker();
  const double ratio = std::max(steer_ratio, 1e-3);
  s.steer_rad = (cs.steering_angle_deg() * M_PI / 180.0) / ratio;
  return s;
}

RawImuSample imuToRaw(const adas::proto::IMUData& imu)
{
  RawImuSample s;
  s.timestamp_us = imu.timestamp() * 1000;
  s.ax = imu.accel_x();
  s.ay = imu.accel_y();
  s.az = imu.accel_z();
  s.gx = imu.gyro_x();
  s.gy = imu.gyro_y();
  s.gz = imu.gyro_z();
  s.valid = true;
  return s;
}

CameraOdometrySample cameraOdometryToSample(const adas::proto::CameraOdometry& odom)
{
  CameraOdometrySample s;
  s.timestamp_us = odom.timestamp() * 1000;
  for (int i = 0; i < 3; ++i) {
    s.trans(i) = (i < odom.trans_size()) ? odom.trans(i) : 0.0;
    s.rot(i) = (i < odom.rot_size()) ? odom.rot(i) : 0.0;
    s.trans_std(i) = (i < odom.trans_std_size()) ? odom.trans_std(i) : 1.0;
    s.rot_std(i) = (i < odom.rot_std_size()) ? odom.rot_std(i) : 1.0;
  }
  s.valid = odom.trans_size() >= 3 && odom.rot_size() >= 3;
  return s;
}

void applyLanePath(LaneKeepOutput& out, const LanePathMsg& msg)
{
  out.dbg.lane_anchored = msg.lane_anchored;
  out.dbg.lanelines_active = msg.lanelines_active;
  out.dbg.lane_width_m = msg.lane_width_m;
  out.dbg.lane_offset_m = msg.lane_offset_m;
  out.dbg.center_force_m = msg.center_force_m;
  out.dbg.p_lane_blend_scale = msg.p_lane_blend_scale;
  out.dbg.p_camera_offset_m = msg.p_camera_offset_m;
  out.dbg.p_center_force_gain = msg.p_center_force_gain;
  out.capture_ts_us = msg.capture_ts_us > 0 ? msg.capture_ts_us : (msg.timestamp_us > 0 ? msg.timestamp_us : 0);
  out.vision_ts_us = msg.infer_ts_us > 0 ? msg.infer_ts_us : 0;
}

void applySteerFeedback(LaneKeepOutput& out, const adas::proto::SteerCommand& cmd)
{
  out.desired_swa_deg = cmd.desired_swa_deg();
  out.actual_swa_deg = cmd.actual_swa_deg();
  out.angle_error_deg = cmd.angle_error_deg();
  out.steer_norm = cmd.steer_norm();
  out.dbg.pid_p = cmd.pid_p();
  out.dbg.pid_i = cmd.pid_i();
  out.dbg.pid_f = cmd.pid_f();
  out.dbg.slew_clipped = cmd.slew_clipped();
  out.dbg.assist_allowed = cmd.assist_allowed();
  out.dbg.assist_known = cmd.assist_known();
  out.dbg.steer_output_enabled = cmd.enabled();
}

adas::proto::SteerCommand createSteerCommand(const SteerCommandInputs& in, const LatControlPid::Result& lat)
{
  adas::proto::SteerCommand cmd;
  cmd.set_torque_cnm(in.torque_cnm);
  cmd.set_enabled(in.enabled);
  cmd.set_capture_ts_ms(in.capture_ts_ms);
  cmd.set_vision_ts_ms(in.vision_ts_ms);
  cmd.set_chassis_ts_ms(in.chassis_ts_ms);
  cmd.set_publish_ts_ms(in.publish_ts_ms);
  cmd.set_desired_swa_deg(lat.angle_des_deg);
  cmd.set_actual_swa_deg(lat.angle_act_deg);
  cmd.set_angle_error_deg(lat.angle_error_deg);
  cmd.set_steer_norm(lat.steer_norm);
  cmd.set_pid_p(lat.p);
  cmd.set_pid_i(lat.i);
  cmd.set_pid_f(lat.f);
  cmd.set_slew_clipped(in.slew_clipped);
  cmd.set_assist_allowed(in.assist_allowed);
  cmd.set_assist_known(in.assist_known);
  cmd.set_status(in.status);
  cmd.set_hud_left_lane_visible(in.hud_left_lane_visible);
  cmd.set_hud_right_lane_visible(in.hud_right_lane_visible);
  cmd.set_cruise_intent(in.cruise_intent);
  return cmd;
}

adas::proto::LatPlan createLatPlan(const LaneKeepOutput& out, double command_curvature, double frame_dt_s,
                                   const char* kappa_solver)
{
  adas::proto::LatPlan msg;
  msg.set_timestamp(out.timestamp_us / 1000);
  msg.set_capture_ts_ms(out.capture_ts_us / 1000);
  msg.set_infer_ts_ms(out.vision_ts_us / 1000);
  msg.set_desired_curvature(command_curvature);
  msg.set_desired_curvature_rate(out.dbg.mpc_dkappa_ds);
  msg.set_valid(out.has_target && out.status == "ok");
  msg.set_status(out.status);
  msg.set_controller(out.controller);
  msg.set_kappa_solver(kappa_solver ? kappa_solver : "");
  msg.set_cte_m(out.cte_m);
  msg.set_epsi_rad(out.epsi_rad);
  msg.set_speed_mps(out.dbg.speed_mps);
  msg.set_frame_dt_s(frame_dt_s);
  return msg;
}

adas::proto::LanePath createLanePath(const LanePathMsg& path)
{
  adas::proto::LanePath msg;
  msg.set_timestamp(path.timestamp_us / 1000);
  msg.set_capture_ts_ms(path.capture_ts_us / 1000);
  msg.set_infer_ts_ms(path.infer_ts_us / 1000);
  msg.set_frame_id(path.frame_id);
  for (const auto& p : path.polyline) {
    auto* pt = msg.add_polyline();
    pt->set_x(p.x());
    pt->set_y(p.y());
  }
  for (const auto& p : path.plan_poly) {
    auto* pt = msg.add_plan_poly();
    pt->set_x(p.x());
    pt->set_y(p.y());
  }
  for (const double v : path.plan_yaw)
    msg.add_plan_yaw(v);
  for (const double v : path.plan_yaw_rate)
    msg.add_plan_yaw_rate(v);
  msg.set_lane_anchored(path.lane_anchored);
  msg.set_lanelines_active(path.lanelines_active);
  msg.set_lane_width_m(path.lane_width_m);
  msg.set_center_force_m(path.center_force_m);
  msg.set_lane_offset_m(path.lane_offset_m);
  msg.set_p_lane_blend_scale(path.p_lane_blend_scale);
  msg.set_p_camera_offset_m(path.p_camera_offset_m);
  msg.set_p_center_force_gain(path.p_center_force_gain);
  return msg;
}

LanePathMsg lanePathFromProto(const adas::proto::LanePath& msg)
{
  LanePathMsg path;
  path.timestamp_us = msg.timestamp() * 1000;
  path.capture_ts_us = msg.capture_ts_ms() * 1000;
  path.infer_ts_us = msg.infer_ts_ms() * 1000;
  path.frame_id = msg.frame_id();
  path.polyline.reserve(msg.polyline_size());
  for (const auto& p : msg.polyline())
    path.polyline.push_back({p.x(), p.y()});
  path.plan_poly.reserve(msg.plan_poly_size());
  for (const auto& p : msg.plan_poly())
    path.plan_poly.push_back({p.x(), p.y()});
  path.plan_yaw.assign(msg.plan_yaw().begin(), msg.plan_yaw().end());
  path.plan_yaw_rate.assign(msg.plan_yaw_rate().begin(), msg.plan_yaw_rate().end());
  path.lane_anchored = msg.lane_anchored();
  path.lanelines_active = msg.lanelines_active();
  path.lane_width_m = msg.lane_width_m();
  path.center_force_m = msg.center_force_m();
  path.lane_offset_m = msg.lane_offset_m();
  path.p_lane_blend_scale = msg.p_lane_blend_scale();
  path.p_camera_offset_m = msg.p_camera_offset_m();
  path.p_center_force_gain = msg.p_center_force_gain();
  return path;
}

adas::proto::CarState carStateFromChassis(const ChassisSample& chassis)
{
  adas::proto::CarState cs;
  cs.set_timestamp(chassis.timestamp_us / 1000);
  cs.set_v_ego(static_cast<float>(chassis.speed_mps));
  cs.set_yaw_rate(static_cast<float>(chassis.yaw_rate));
  cs.set_steering_angle_deg(static_cast<float>(chassis.steering_angle_deg));
  cs.set_steering_pressed(chassis.steering_pressed);
  cs.set_left_blinker(chassis.left_blinker);
  cs.set_right_blinker(chassis.right_blinker);
  return cs;
}

longplan::LeadState leadFromModel(const adas::proto::ModelLongPlan& plan)
{
  const auto& lead = plan.lead0();
  longplan::LeadState out;
  out.prob = lead.prob();
  out.d_rel = lead.d_rel() > 0 ? lead.d_rel() : (lead.x_size() > 0 ? lead.x(0) : 0.0);
  out.v_lead = lead.v_lead() != 0 ? lead.v_lead() : (lead.v_size() > 0 ? lead.v(0) : 0.0);
  out.y_rel = lead.y_rel();
  return out;
}

}  // namespace adas
