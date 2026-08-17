#include "adas/utils/adas_config.h"

#include "adas/platform/car_platform.h"

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

/**
 * \brief Derived fields of the parameter learner, in one place.
 *
 * \details The learner does not read the config itself: it is handed a vehicle it must agree with.
 * Without this a simulated app builds the learner with steering sign +1 (the `ParamsLearner::Config`
 * default) while the car drives with −1, and the model can only reconcile an inverted sign by driving
 * stiffness into its clamp and the ratio sideways — the estimator diverges in every replay.
 *
 * Both construction paths call it, so the two cannot drift apart.
 */
/**
 * \brief Fill in what the selected car is, before the config gets a say.
 *
 * \details Upstream keeps wheelbase, steering ratio and mass in `selfdrive/car/<brand>/interface.py`,
 * not in a user config, and for good reason: nobody tunes the wheelbase of their Golf, and a config
 * carried over from another car quietly describes the wrong vehicle. Here the platform supplies them
 * first and `setDouble` overrides only the keys the config actually contains — so an untouched config
 * means the car's own numbers, and a deliberate override still works for a trim that differs.
 *
 * \param[in,out] cfg Config whose vehicle fields are seeded from `cfg.vehicle_name`.
 */
void applyCarDefaults(AdasApp::Config& cfg)
{
  auto car = adas::platform::makeCarPlatform(cfg.vehicle_name, {});
  if (!car) {
    LOGW("config: vehicle.name '%s' is not a car we have — keeping the built-in defaults", cfg.vehicle_name.c_str());
    return;
  }
  const adas::platform::VehicleDefaults d = car->defaults();
  cfg.lane_keep.wheelbase_m = d.wheelbase_m;
  cfg.lane_keep.steer_ratio = d.steer_ratio;
  cfg.lane_keep.steer_sign = d.steer_sign;
  cfg.lane_keep.tire_stiffness_factor = d.tire_stiffness_factor;
  cfg.lane_keep.max_steer_deg = d.max_steer_deg;
  cfg.localization.wheelbase_m = d.wheelbase_m;
  // The longitudinal loop and Control take the same numbers from lane_keep (see
  // AdasApp::controlConfig), so one place has to be seeded, not three.
  LOGI("config: car %s — wheelbase %.3f m, steer ratio %.2f, sign %+.0f", car->name(), d.wheelbase_m, d.steer_ratio,
       d.steer_sign);
}

void applyLearnerDefaults(AdasApp::Config& cfg)
{
  auto& pl = cfg.localization.params;
  pl.vehicle.wheelbase_m = cfg.localization.wheelbase_m;
  pl.vehicle.tire_stiffness_factor = cfg.lane_keep.tire_stiffness_factor;
  pl.stiffness_init = cfg.lane_keep.tire_stiffness_factor;
  pl.steer_ratio_init = cfg.lane_keep.steer_ratio;
  pl.steer_sign = cfg.lane_keep.steer_sign;
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
  cfg.feature_flags.enable_vision_supercombo = false;

  cfg.lane_keep.wheelbase_m = wheelbase_m;
  cfg.localization.wheelbase_m = wheelbase_m;

  cfg.camera_calib.pitch_deg = pitch_deg;
  cfg.camera_calib.yaw_deg = yaw_deg;
  cfg.camera_calib.height_m = camera_height_m;

  cfg.localization.imu_mount_roll_deg = 0.0;
  cfg.localization.imu_mount_pitch_deg = pitch_deg;
  cfg.localization.imu_mount_yaw_deg = yaw_deg;
  cfg.localization.imu_has_mount_prior = true;

  applyLearnerDefaults(cfg);
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

  const Json::Value& veh = root["vehicle"];
  setString(veh, "name", cfg.vehicle_name);
  // Car first, config second: a key that is absent leaves the car's own number in place.
  applyCarDefaults(cfg);
  setDouble(veh, "wheelbase_m", cfg.lane_keep.wheelbase_m);
  setDouble(veh, "wheelbase_m", cfg.localization.wheelbase_m);
  adas::LanePathConfig lane_path{};
  // One steer ratio for everyone, and it starts from the car rather than a literal. There used to be
  // two independent defaults for one key here: while the key was present in the config they agreed,
  // and the moment it was removed the hardcoded 15.7 won over the selected car's number.
  double steer_ratio = cfg.lane_keep.steer_ratio;
  setDouble(veh, "steer_ratio", steer_ratio);
  setDouble(veh, "path_lane_blend_scale", lane_path.lane_blend_scale);
  setDouble(veh, "path_camera_offset_m", lane_path.camera_offset_m);
  setDouble(veh, "lane_std_good_m", lane_path.lane_std_good_m);
  setDouble(veh, "lane_std_bad_m", lane_path.lane_std_bad_m);
  setDouble(veh, "lane_std_range_m", lane_path.lane_std_range_m);
  setDouble(veh, "lane_width_min_m", lane_path.lane_width_min_m);
  setDouble(veh, "lane_width_max_m", lane_path.lane_width_max_m);
  setDouble(veh, "center_force_gain", lane_path.center_force_gain);
  setDouble(veh, "center_force_max_m", lane_path.center_force_max_m);
  setDouble(veh, "center_force_turn_scale", lane_path.center_force_turn_scale);
  setDouble(veh, "max_steer_deg", cfg.lane_keep.max_steer_deg);
  setDouble(veh, "max_torque_cnm", cfg.lane_keep.max_torque_cnm);
  setString(veh, "fp_solver", cfg.lane_keep.fp_solver);
  setDouble(veh, "lane_mode_off_prob", lane_path.lane_mode_off_prob);
  setDouble(veh, "lane_mode_on_prob", lane_path.lane_mode_on_prob);

  cfg.lane_keep.lane_path = lane_path;
  cfg.lane_keep.steer_ratio = steer_ratio;
  cfg.lane_keep.long_plan.lane_path = lane_path;
  cfg.lane_keep.long_plan.steer_ratio = steer_ratio;
  cfg.safety_warn.lane_path = lane_path;
  cfg.safety_warn.steer_ratio = steer_ratio;
  cfg.camera_calib.steer_ratio = steer_ratio;
  cfg.traffic_sign.steer_ratio = steer_ratio;
  cfg.localization.steer_ratio = steer_ratio;
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
  setDouble(veh, "tire_stiffness_factor", cfg.lane_keep.tire_stiffness_factor);
  setBool(veh, "lat_use_vehicle_model", cfg.lane_keep.lat_use_vehicle_model);
  setDouble(veh, "wheel_speed_factor", cfg.panda.speed_filter.wheel_speed_factor);
  setDouble(veh, "speed_accel_process_noise", cfg.panda.speed_filter.accel_process_noise);
  setDouble(veh, "speed_measurement_noise", cfg.panda.speed_filter.speed_measurement_noise);
  setDouble(veh, "fp_steer_delay_s", cfg.lane_keep.fp_steer_delay_s);
  setDouble(veh, "min_control_speed_mps", cfg.lane_keep.min_control_speed_mps);
  setDouble(veh, "lane_max_age_s", cfg.lane_keep.lane_max_age_s);
  setDouble(veh, "lka_blinker_resume_delay_s", cfg.lane_keep.lka_blinker_resume_delay_s);
  setDouble(veh, "assist_max_age_s", cfg.lane_keep.assist_max_age_s);
  setDouble(veh, "min_control_speed_hyst_mps", cfg.lane_keep.min_control_speed_hyst_mps);
  if (cfg.lane_keep.controller != "mpc" && cfg.lane_keep.controller != "fp")
    cfg.lane_keep.controller = "pp";

  const Json::Value& lp = root["long_plan"];
  setDouble(lp, "t_follow", cfg.lane_keep.long_plan.t_follow);
  setDouble(lp, "min_gap_m", cfg.lane_keep.long_plan.min_gap_m);
  setDouble(lp, "a_max", cfg.lane_keep.long_plan.a_max);
  setDouble(lp, "a_min", cfg.lane_keep.long_plan.a_min);
  setDouble(lp, "kp_gap", cfg.lane_keep.long_plan.kp_gap);
  setDouble(lp, "kp_v", cfg.lane_keep.long_plan.kp_v);
  setDouble(lp, "lead_prob_thresh", cfg.lane_keep.long_plan.lead_prob_thresh);
  setDouble(lp, "curv_a_lat_max", cfg.lane_keep.long_plan.curv_a_lat_max);
  setDouble(lp, "curv_preview_s", cfg.lane_keep.long_plan.curv_preview_s);
  setDouble(lp, "curv_min_speed_ms", cfg.lane_keep.long_plan.curv_min_speed_ms);
  setDouble(lp, "curv_v_floor_ms", cfg.lane_keep.long_plan.curv_v_floor_ms);
  setDouble(lp, "a_coast_ms2", cfg.lane_keep.long_plan.a_coast_ms2);
  setDouble(lp, "lead_max_offset_m", cfg.lane_keep.long_plan.lead_max_offset_m);
  setDouble(lp, "lead_min_speed_ms", cfg.lane_keep.long_plan.lead_min_speed_ms);
  setBool(lp, "plan_v_enabled", cfg.lane_keep.long_plan.plan_v_enabled);

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
  if (warn.isObject() && warn.isMember("warn_set_frames") && warn["warn_set_frames"].isNumeric())
    cfg.safety_warn.warn_set_frames = warn["warn_set_frames"].asInt();
  if (warn.isObject() && warn.isMember("warn_hold_frames") && warn["warn_hold_frames"].isNumeric())
    cfg.safety_warn.warn_hold_frames = warn["warn_hold_frames"].asInt();

  const Json::Value& cam = root["calibration"]["camera"];
  const Json::Value& rpy = cam["rpy_deg"];
  setDouble(rpy, "roll", cfg.localization.imu_mount_roll_deg);
  setDouble(rpy, "pitch", cfg.camera_calib.pitch_deg);
  setDouble(rpy, "pitch", cfg.localization.imu_mount_pitch_deg);
  setDouble(rpy, "yaw", cfg.camera_calib.yaw_deg);
  setDouble(rpy, "yaw", cfg.localization.imu_mount_yaw_deg);
  cfg.localization.imu_has_mount_prior = rpy.isObject();

  const Json::Value& pos = cam["position_m"];
  setDouble(pos, "z_up", cfg.camera_calib.height_m);
  setDouble(pos, "y_left", cfg.lane_keep.cam_y_left_m);
  setDouble(pos, "y_left", cfg.lane_keep.lane_path.cam_y_left_m);
  cfg.lane_keep.long_plan.lane_path.cam_y_left_m = cfg.lane_keep.lane_path.cam_y_left_m;
  cfg.safety_warn.lane_path.cam_y_left_m = cfg.lane_keep.lane_path.cam_y_left_m;

  const Json::Value& K = cam["intrinsics_prior"];
  setDouble(K, "fx", cfg.camera_calib.fx);
  setDouble(K, "fy", cfg.camera_calib.fy);
  setDouble(K, "cx", cfg.camera_calib.cx);
  setDouble(K, "cy", cfg.camera_calib.cy);

  const Json::Value& loc = root["localization"];
  auto& src = cfg.localization.sources;
  setBool(loc, "use_camera_odometry", src.camera_odometry);
  setDouble(loc, "gps_noise_pos", cfg.localization.gps_noise_pos);
  setDouble(loc, "gps_max_accuracy_m", cfg.localization.gps_max_accuracy_m);
  auto& rr = cfg.localization.road_roll;
  setDouble(loc, "road_roll_body_deg_per_g", rr.body_roll_deg_per_g);
  setDouble(loc, "road_roll_tau_s", rr.tau_s);
  setDouble(loc, "road_roll_min_speed_ms", rr.min_speed_ms);
  setDouble(loc, "gps_update_interval", cfg.localization.gps_update_interval);

  applyLearnerDefaults(cfg);
  auto& pl = cfg.localization.params;
  setDouble(loc, "params_angle_offset_init_deg", pl.angle_offset_init_deg);
  setDouble(loc, "imu_speed_threshold_kmh", cfg.localization.imu_speed_threshold_kmh);
  setBool(loc, "imu_invert_yaw_rate", cfg.localization.imu_invert_yaw_rate);
  setDouble(loc, "params_stiffness_p0_std", pl.stiffness_p0_std);
  setDouble(loc, "params_stiffness_process_std", pl.stiffness_process_std);
  setDouble(loc, "params_steer_ratio_process_std", pl.steer_ratio_process_std);
  setDouble(loc, "params_min_speed_ms", pl.min_speed_ms);
  setDouble(loc, "params_max_lateral_jerk", pl.max_lateral_jerk);
  setDouble(loc, "params_max_roll_std_deg", pl.max_roll_std_deg);
  setBool(loc, "params_use_roll", pl.use_roll);

  const Json::Value& zmq = root["zmq"];
  setString(zmq, "endpoint_in", cfg.zmq_bridge.endpoint_in);
  setString(zmq, "endpoint_out", cfg.zmq_bridge.endpoint_out);

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

  LOGI("AdasApp::Config %s: lane_keep=%d ctrl=%s loc=%d cam=%d wb=%.3f "
       "max_steer=%.1f° max_tq=%.0f pid=%.2f/%.2f/%.5f P/Y=%.1f/%.1f h=%.2f",
       path.c_str(), f.enable_lane_keep ? 1 : 0, cfg.lane_keep.controller.c_str(), f.enable_localization ? 1 : 0,
       f.enable_camera_calib ? 1 : 0, cfg.lane_keep.wheelbase_m, cfg.lane_keep.max_steer_deg,
       cfg.lane_keep.max_torque_cnm, cfg.lane_keep.pid_kp, cfg.lane_keep.pid_ki, cfg.lane_keep.pid_kf,
       cfg.camera_calib.pitch_deg, cfg.camera_calib.yaw_deg, cfg.camera_calib.height_m);

  if (ok)
    *ok = true;
  return cfg;
}
