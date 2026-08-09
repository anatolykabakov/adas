#include "adas/utils/adas_config.h"

#include <fstream>
#include <string>

#include <json/json.h>

#include "adas/utils/logger.h"

namespace {

void setBool(const Json::Value& o, const char* key, bool& field)
{
  if (o.isObject() && o.isMember(key) && o[key].isBool())
    field = o[key].asBool();
}

void setDouble(const Json::Value& o, const char* key, double& field)
{
  if (o.isObject() && o.isMember(key) && o[key].isNumeric())
    field = o[key].asDouble();
}

void setString(const Json::Value& o, const char* key, std::string& field)
{
  if (o.isObject() && o.isMember(key) && o[key].isString()) {
    const std::string v = o[key].asString();
    if (!v.empty())
      field = v;
  }
}

bool parseFile(const std::string& path, Json::Value* root, std::string* err)
{
  std::ifstream in(path);
  if (!in) {
    if (err)
      *err = "cannot open file";
    return false;
  }
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  return Json::parseFromStream(builder, in, root, err);
}

}  // namespace

AdasApp::Config AdasApp::Config::forSimulated(double wheelbase_m, double pitch_deg, double yaw_deg,
                                              double camera_height_m)
{
  Config cfg;
  cfg.feature_flags.enable_panda = false;
  cfg.feature_flags.enable_zmq_bridge = false;
  cfg.feature_flags.enable_lane_keep = true;
  cfg.feature_flags.enable_localization = true;
  cfg.feature_flags.enable_camera_calib = true;
  cfg.feature_flags.enable_imu_calib = true;
  cfg.feature_flags.enable_vision_supercombo = false;

  cfg.lane_keep.wheelbase_m = wheelbase_m;
  cfg.localization.wheelbase_m = wheelbase_m;
  cfg.lane_keep.steer_output_enabled = true;

  cfg.camera_calib.pitch_deg = pitch_deg;
  cfg.camera_calib.yaw_deg = yaw_deg;
  cfg.camera_calib.height_m = camera_height_m;

  cfg.imu_calib.mount_roll_deg = 0.0;
  cfg.imu_calib.mount_pitch_deg = pitch_deg;
  cfg.imu_calib.mount_yaw_deg = yaw_deg;
  cfg.imu_calib.has_mount_prior = true;
  return cfg;
}

AdasApp::Config AdasApp::Config::loadFromFile(const std::string& path, bool* ok)
{
  Config cfg;
  Json::Value root;
  std::string err;
  if (!parseFile(path, &root, &err) || !root.isObject()) {
    LOGE("AdasApp::Config::loadFromFile: cannot read/parse %s (%s) — using defaults", path.c_str(), err.c_str());
    if (ok)
      *ok = false;
    return cfg;
  }

  auto& f = cfg.feature_flags;
  const Json::Value& nodes = root["nodes"];
  setBool(nodes, "panda", f.enable_panda);
  setBool(nodes, "zmq_bridge", f.enable_zmq_bridge);
  setBool(nodes, "lane_keep", f.enable_lane_keep);
  setBool(nodes, "localization", f.enable_localization);
  setBool(nodes, "camera_calib", f.enable_camera_calib);
  setBool(nodes, "vision_supercombo", f.enable_vision_supercombo);
  setBool(nodes, "long_plan", f.enable_long_plan);
  setBool(nodes, "safety_warn", f.enable_safety_warn);
  setBool(nodes, "traffic_sign", f.enable_traffic_sign);
  setBool(nodes, "map_data", f.enable_map_data);

  const bool had_imu_key = nodes.isObject() && nodes.isMember("imu_calib");
  setBool(nodes, "imu_calib", f.enable_imu_calib);
  if (!had_imu_key)
    f.enable_imu_calib = f.enable_localization;

  const Json::Value& veh = root["vehicle"];
  setString(veh, "name", cfg.vehicle_name);
  setDouble(veh, "wheelbase_m", cfg.lane_keep.wheelbase_m);
  setDouble(veh, "wheelbase_m", cfg.localization.wheelbase_m);
  setDouble(veh, "steer_ratio", cfg.lane_keep.steer_ratio);
  setDouble(veh, "steer_ratio", cfg.topic_convert.steer_ratio);
  setDouble(veh, "path_lane_blend_scale", cfg.topic_convert.path_lane_blend_scale);
  setDouble(veh, "path_camera_offset_m", cfg.topic_convert.path_camera_offset_m);
  setDouble(veh, "lane_std_good_m", cfg.topic_convert.lane_std_good_m);
  setDouble(veh, "lane_std_bad_m", cfg.topic_convert.lane_std_bad_m);
  setDouble(veh, "lane_std_range_m", cfg.topic_convert.lane_std_range_m);
  setDouble(veh, "lane_width_min_m", cfg.topic_convert.lane_width_min_m);
  setDouble(veh, "lane_width_max_m", cfg.topic_convert.lane_width_max_m);
  setDouble(veh, "center_force_gain", cfg.topic_convert.center_force_gain);
  setDouble(veh, "center_force_max_m", cfg.topic_convert.center_force_max_m);
  setDouble(veh, "center_force_turn_scale", cfg.topic_convert.center_force_turn_scale);
  setDouble(veh, "max_steer_deg", cfg.lane_keep.max_steer_deg);
  setDouble(veh, "max_torque_cnm", cfg.lane_keep.max_torque_cnm);
  setDouble(veh, "pp_k_dd", cfg.lane_keep.pp_k_dd);
  setDouble(veh, "pp_ld_min", cfg.lane_keep.pp_ld_min);
  setDouble(veh, "pp_ld_max", cfg.lane_keep.pp_ld_max);
  setDouble(veh, "pp_shift", cfg.lane_keep.pp_shift);
  setDouble(veh, "pp_ld_curv_gain", cfg.lane_keep.pp_ld_curv_gain);
  setDouble(veh, "lat_pid_kp", cfg.lane_keep.pid_kp);
  setDouble(veh, "lat_pid_ki", cfg.lane_keep.pid_ki);
  setDouble(veh, "lat_pid_kf", cfg.lane_keep.pid_kf);
  setDouble(veh, "lat_pid_ff_floor_mps", cfg.lane_keep.pid_ff_floor_mps);
  setDouble(veh, "steer_sign", cfg.lane_keep.steer_sign);
  setString(veh, "lane_keep_controller", cfg.lane_keep.controller);
  if (cfg.lane_keep.controller == "flowpilot")
    cfg.lane_keep.controller = "fp";
  setBool(veh, "cruise_buttons", cfg.panda.cruise_buttons_enabled);
  setDouble(veh, "cruise_deadband_ms", cfg.panda.cruise_deadband_ms);
  setDouble(veh, "cruise_tip_step_ms", cfg.panda.cruise_tip_step_ms);
  if (veh.isObject() && veh.isMember("cruise_tip_cooldown_ms") && veh["cruise_tip_cooldown_ms"].isNumeric())
    cfg.panda.cruise_tip_cooldown_ms = veh["cruise_tip_cooldown_ms"].asInt();
  setDouble(veh, "mpc_Lf", cfg.lane_keep.mpc_Lf);
  setDouble(veh, "mpc_max_steer_deg", cfg.lane_keep.mpc_max_steer_deg);
  setDouble(veh, "mpc_low_speed_steer_deg", cfg.lane_keep.mpc_low_speed_steer_deg);
  setDouble(veh, "mpc_steer_deg_per_mps", cfg.lane_keep.mpc_steer_deg_per_mps);
  setDouble(veh, "mpc_max_lateral_jerk", cfg.lane_keep.mpc_max_lateral_jerk);
  setDouble(veh, "mpc_rate_min_speed", cfg.lane_keep.mpc_rate_min_speed);
  setDouble(veh, "mpc_rate_limit_deg", cfg.lane_keep.mpc_rate_limit_deg);
  setDouble(veh, "mpc_kappa_yaw_blend", cfg.lane_keep.mpc_kappa_yaw_blend);
  setDouble(veh, "mpc_kappa_yaw_min_speed", cfg.lane_keep.mpc_kappa_yaw_min_speed);
  setDouble(veh, "mpc_epsi_gain", cfg.lane_keep.mpc_epsi_gain);
  setDouble(veh, "mpc_ff_scale", cfg.lane_keep.mpc_ff_scale);
  setDouble(veh, "mpc_cte_weight_base", cfg.lane_keep.mpc_cte_weight_base);
  setDouble(veh, "mpc_cte_quartic_scale", cfg.lane_keep.mpc_cte_quartic_scale);
  setDouble(veh, "mpc_cte_gain_base", cfg.lane_keep.mpc_cte_gain_base);
  setDouble(veh, "mpc_cte_gain_floor", cfg.lane_keep.mpc_cte_gain_floor);
  setDouble(veh, "mpc_kappa_ema_alpha", cfg.lane_keep.mpc_kappa_ema_alpha);
  setDouble(veh, "mpc_epsi_ema_alpha", cfg.lane_keep.mpc_epsi_ema_alpha);
  setDouble(veh, "mpc_cte_ema_alpha", cfg.lane_keep.mpc_cte_ema_alpha);
  setDouble(veh, "fp_steering_rate_weight", cfg.lane_keep.fp_steering_rate_weight);
  setDouble(veh, "steer_slew_limit_deg", cfg.lane_keep.steer_slew_limit_deg);
  setDouble(veh, "vision_nominal_dt_s", cfg.lane_keep.vision_nominal_dt_s);
  setBool(veh, "lat_use_vehicle_model", cfg.lane_keep.lat_use_vehicle_model);
  setDouble(veh, "tire_stiffness_factor", cfg.lane_keep.tire_stiffness_factor);
  setBool(veh, "use_learned_params", cfg.lane_keep.use_learned_params);
  setDouble(veh, "wheel_speed_factor", cfg.panda.speed_filter.wheel_speed_factor);
  setDouble(veh, "speed_accel_process_noise", cfg.panda.speed_filter.accel_process_noise);
  setDouble(veh, "speed_measurement_noise", cfg.panda.speed_filter.speed_measurement_noise);
  setDouble(veh, "fp_steer_delay_s", cfg.lane_keep.fp_steer_delay_s);
  setDouble(veh, "min_control_speed_mps", cfg.lane_keep.min_control_speed_mps);
  setDouble(veh, "lane_max_age_s", cfg.lane_keep.lane_max_age_s);
  setBool(veh, "lka_suppress_on_blinker", cfg.lane_keep.lka_suppress_on_blinker);
  setDouble(veh, "lka_blinker_resume_delay_s", cfg.lane_keep.lka_blinker_resume_delay_s);
  setBool(veh, "lat_recompute_setpoint", cfg.lane_keep.lat_recompute_setpoint);
  setBool(veh, "lat_require_assist", cfg.lane_keep.lat_require_assist);
  setBool(veh, "lat_always_on", cfg.panda.lat_always_on);
  setDouble(veh, "assist_max_age_s", cfg.lane_keep.assist_max_age_s);
  setDouble(veh, "min_control_speed_hyst_mps", cfg.lane_keep.min_control_speed_hyst_mps);
  if (cfg.lane_keep.controller != "mpc" && cfg.lane_keep.controller != "fp")
    cfg.lane_keep.controller = "pp";
  cfg.lane_keep.steer_output_enabled = f.enable_lane_keep;

  const Json::Value& lp = root["long_plan"];
  setDouble(lp, "t_follow", cfg.long_plan.t_follow);
  setDouble(lp, "min_gap_m", cfg.long_plan.min_gap_m);
  setDouble(lp, "a_max", cfg.long_plan.a_max);
  setDouble(lp, "a_min", cfg.long_plan.a_min);
  setDouble(lp, "kp_gap", cfg.long_plan.kp_gap);
  setDouble(lp, "kp_v", cfg.long_plan.kp_v);
  setDouble(lp, "lead_prob_thresh", cfg.long_plan.lead_prob_thresh);
  setBool(lp, "curv_enabled", cfg.long_plan.curv_enabled);
  setDouble(lp, "curv_a_lat_max", cfg.long_plan.curv_a_lat_max);
  setDouble(lp, "curv_preview_s", cfg.long_plan.curv_preview_s);
  setDouble(lp, "curv_min_speed_ms", cfg.long_plan.curv_min_speed_ms);
  setDouble(lp, "curv_v_floor_ms", cfg.long_plan.curv_v_floor_ms);
  setDouble(lp, "a_coast_ms2", cfg.long_plan.a_coast_ms2);
  setDouble(lp, "lead_max_offset_m", cfg.long_plan.lead_max_offset_m);
  setDouble(lp, "lead_min_speed_ms", cfg.long_plan.lead_min_speed_ms);
  setBool(lp, "plan_v_enabled", cfg.long_plan.plan_v_enabled);

  const Json::Value& warn = root["safety_warn"];
  auto& planner = cfg.safety_warn.planner;
  setDouble(warn, "fcw_ttc_s", planner.fcw_ttc_s);
  setDouble(warn, "aeb_ttc_s", planner.aeb_ttc_s);
  setDouble(warn, "fcw_decel_ms2", planner.fcw_decel_ms2);
  setDouble(warn, "aeb_decel_ms2", planner.aeb_decel_ms2);
  setDouble(warn, "warn_min_speed_ms", planner.warn_min_speed_ms);
  setDouble(warn, "min_closing_speed_ms", planner.min_closing_speed_ms);
  setDouble(warn, "lead_prob_thresh", planner.lead_prob_thresh);
  setDouble(warn, "lead_max_offset_m", planner.lead_max_offset_m);
  setDouble(warn, "front_bumper_offset_m", planner.front_bumper_offset_m);
  setDouble(warn, "cte_ldw_threshold_m", planner.cte_ldw_threshold_m);
  setDouble(warn, "cte_ldw_hard_m", planner.cte_ldw_hard_m);
  setDouble(warn, "ldw_min_speed_ms", planner.ldw_min_speed_ms);
  setDouble(warn, "ldw_min_outward_rate_ms", planner.ldw_min_outward_rate_ms);
  setBool(warn, "ldw_suppress_on_driver_steer", planner.ldw_suppress_on_driver_steer);
  setBool(warn, "ldw_suppress_on_blinker", planner.ldw_suppress_on_blinker);
  setBool(warn, "ldw_suppress_on_lat_active", planner.ldw_suppress_on_lat_active);
  if (warn.isObject() && warn.isMember("warn_set_frames") && warn["warn_set_frames"].isNumeric())
    cfg.safety_warn.warn_set_frames = warn["warn_set_frames"].asInt();
  if (warn.isObject() && warn.isMember("warn_hold_frames") && warn["warn_hold_frames"].isNumeric())
    cfg.safety_warn.warn_hold_frames = warn["warn_hold_frames"].asInt();

  const Json::Value& cam = root["calibration"]["camera"];
  const Json::Value& rpy = cam["rpy_deg"];
  setDouble(rpy, "roll", cfg.imu_calib.mount_roll_deg);
  setDouble(rpy, "pitch", cfg.camera_calib.pitch_deg);
  setDouble(rpy, "pitch", cfg.imu_calib.mount_pitch_deg);
  setDouble(rpy, "yaw", cfg.camera_calib.yaw_deg);
  setDouble(rpy, "yaw", cfg.imu_calib.mount_yaw_deg);
  cfg.imu_calib.has_mount_prior = rpy.isObject();

  const Json::Value& pos = cam["position_m"];
  setDouble(pos, "z_up", cfg.camera_calib.height_m);
  setDouble(pos, "y_left", cfg.lane_keep.cam_y_left_m);
  setDouble(pos, "y_left", cfg.topic_convert.cam_y_left_m);

  const Json::Value& K = cam["intrinsics_prior"];
  setDouble(K, "fx", cfg.camera_calib.fx);
  setDouble(K, "fy", cfg.camera_calib.fy);
  setDouble(K, "cx", cfg.camera_calib.cx);
  setDouble(K, "cy", cfg.camera_calib.cy);

  // Localization measurement sources. Each is a separate key so a run can be made to answer "what would
  // the pose do without this sensor" — the experiment that finally quantified the yaw-rate defect.
  const Json::Value& loc = root["localization"];
  auto& src = cfg.localization.sources;
  setBool(loc, "use_gps_position", src.gps_position);
  setBool(loc, "use_gps_course", src.gps_course);
  setBool(loc, "use_gps_velocity", src.gps_velocity);
  setBool(loc, "use_imu_yaw_rate", src.imu_yaw_rate);
  setBool(loc, "use_chassis_yaw_rate", src.chassis_yaw_rate);
  setBool(loc, "use_camera_odometry", src.camera_odometry);
  setBool(loc, "invert_cam_yaw_rate", cfg.localization.invert_cam_yaw_rate);
  setBool(loc, "use_bicycle_model", src.bicycle_model);
  setDouble(loc, "gps_noise_pos", cfg.localization.gps_noise_pos);
  auto& rr = cfg.localization.road_roll;
  setDouble(loc, "road_roll_body_deg_per_g", rr.body_roll_deg_per_g);
  setDouble(loc, "road_roll_tau_s", rr.tau_s);
  setDouble(loc, "road_roll_min_speed_ms", rr.min_speed_ms);
  setDouble(loc, "gps_update_interval", cfg.localization.gps_update_interval);

  // The learner must start from, and be bounded around, exactly the model the controller uses. Reading the
  // vehicle block here rather than giving the learner its own copy of `wheelbase_m` and `steer_ratio` is
  // deliberate: two sources for the same constant is how a "learned" parameter ends up meaning something
  // slightly different from the parameter it replaces, which is worse than a hand-tuned number because it
  // looks principled.
  auto& pl = cfg.localization.params;
  pl.vehicle.wheelbase_m = cfg.localization.wheelbase_m;
  pl.vehicle.tire_stiffness_factor = cfg.lane_keep.tire_stiffness_factor;
  pl.stiffness_init = cfg.lane_keep.tire_stiffness_factor;
  pl.steer_ratio_init = cfg.lane_keep.steer_ratio;
  pl.steer_sign = cfg.lane_keep.steer_sign;
  setBool(loc, "learn_vehicle_params", cfg.localization.learn_vehicle_params);
  setDouble(loc, "params_stiffness_process_std", pl.stiffness_process_std);
  setDouble(loc, "params_steer_ratio_process_std", pl.steer_ratio_process_std);
  setDouble(loc, "params_min_speed_ms", pl.min_speed_ms);
  setDouble(loc, "params_max_lateral_jerk", pl.max_lateral_jerk);
  setDouble(loc, "params_max_roll_std_deg", pl.max_roll_std_deg);
  setBool(loc, "params_use_roll", pl.use_roll);

  const Json::Value& zmq = root["zmq"];
  setString(zmq, "endpoint_in", cfg.zmq_bridge.endpoint_in);
  setString(zmq, "endpoint_out", cfg.zmq_bridge.endpoint_out);

  // OSM curvature ahead. The thresholds are dragonpilot's and are exposed because the only way to know
  // whether 0.002 1/m and 2.6 m/s² are right for this map is to vary them over a recorded run.
  const Json::Value& map = root["map"];
  setString(map, "path", cfg.map_data.map_path);
  setDouble(map, "update_hz", cfg.map_data.update_hz);
  setDouble(map, "local_map_period_s", cfg.map_data.local_map_period_s);
  setDouble(map, "local_map_radius_m", cfg.map_data.local_map_radius_m);
  setDouble(map, "min_speed_mps", cfg.map_data.min_speed_mps);
  setDouble(map, "max_pose_gap_m", cfg.map_data.max_pose_gap_m);
  setDouble(map, "max_fix_age_s", cfg.map_data.max_fix_age_s);
  auto& rt = cfg.map_data.route;
  setDouble(map, "horizon_m", rt.horizon_m);
  setDouble(map, "max_match_dist_m", rt.max_match_dist_m);
  setDouble(map, "max_match_heading_deg", rt.max_match_heading_deg);
  setDouble(map, "step_m", rt.step_m);
  setDouble(map, "window_m", rt.window_m);
  setDouble(map, "turn_kappa", rt.turn_kappa);
  setDouble(map, "max_lat_acc", rt.max_lat_acc);
  setDouble(map, "min_section_m", rt.min_section_m);

  LOGI("AdasApp::Config %s: lane_keep=%d ctrl=%s loc=%d cam=%d imu=%d wb=%.3f "
       "max_steer=%.1f° max_tq=%.0f pid=%.2f/%.2f/%.5f P/Y=%.1f/%.1f h=%.2f",
       path.c_str(), f.enable_lane_keep ? 1 : 0, cfg.lane_keep.controller.c_str(), f.enable_localization ? 1 : 0,
       f.enable_camera_calib ? 1 : 0, f.enable_imu_calib ? 1 : 0, cfg.lane_keep.wheelbase_m,
       cfg.lane_keep.max_steer_deg, cfg.lane_keep.max_torque_cnm, cfg.lane_keep.pid_kp, cfg.lane_keep.pid_ki,
       cfg.lane_keep.pid_kf, cfg.camera_calib.pitch_deg, cfg.camera_calib.yaw_deg, cfg.camera_calib.height_m);

  if (ok)
    *ok = true;
  return cfg;
}
