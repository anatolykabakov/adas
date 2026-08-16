#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace topics {
inline constexpr const char* kVehicleChassis = "vehicle/chassis";
inline constexpr const char* kVisionLanes = "vision/lanes";
inline constexpr const char* kVisionPath = "vision/path";
/** \brief A ready-made reference path as input, bypassing lane-line parsing.
 *
 *  `kVisionPath` is published by the Planner now, so feeding a path in on the same topic would close a
 *  loop. The harnesses need this: replaying a foreign log and running the simulator both yield a path
 *  already computed rather than lane lines, and `LaneLines` cannot express it — it has neither
 *  `lane_anchored` nor separate `polyline` and `plan_poly`. Empty in the car, where the Planner builds
 *  the path itself. */
inline constexpr const char* kVisionPathIn = "vision/path_in";
inline constexpr const char* kGpsLocation = "sensors/gps/location";
inline constexpr const char* kGpsData = "sensors/gps/data";
inline constexpr const char* kImu = "sensors/imu";
inline constexpr const char* kImuRaw = "sensors/imu_raw";
inline constexpr const char* kImuYaw = "sensors/imu_yaw";
/** \brief Lateral planner output: the desired curvature.
 *
 *  The plan-to-control interface, as upstream's `lateralPlan.desiredCurvature` is. */
inline constexpr const char* kLatPlan = "control/lat_plan";
inline constexpr const char* kLaneKeep = "control/lane_keep";
inline constexpr const char* kLaneKeepDebug = "control/lane_keep_debug";
inline constexpr const char* kLocalizationPose = "localization/pose";
inline constexpr const char* kSteerCommand = "controls/steer";
inline constexpr const char* kCameraCalib = "calibration/camera";
inline constexpr const char* kCameraCalibDebug = "calibration/camera_debug";
/// Camera intrinsics as the device reports them, published once the camera opens.
inline constexpr const char* kCameraIntrinsics = "camera/intrinsics";
inline constexpr const char* kCalibLaneUv = "calibration/lane_uv";
inline constexpr const char* kCameraOdometry = "model/camera_odometry";
inline constexpr const char* kVisionModelLong = "vision/model_long";
inline constexpr const char* kLongPlan = "control/long_plan";
inline constexpr const char* kSafetyWarn = "safety/warn";
inline constexpr const char* kTrafficDetections = "vision/traffic_dets";
inline constexpr const char* kTrafficVision = "traffic/state";
inline constexpr const char* kVehicleState = "vehicle/state";
inline constexpr const char* kCanRx = "can/rx";
/** \brief Received frames, for the record. `Platform` decodes and publishes them; they go out only to
 *  be logged, since the chassis arrives as its own message and no service above reads frames itself. */
inline constexpr const char* kPandaHealth = "panda/health";
inline constexpr const char* kMiddlewareStats = "middleware/stats";
inline constexpr const char* kMapLocal = "map/local";
}  // namespace topics

struct ChassisSample {
  int64_t timestamp_us = 0;
  double speed_mps = 0.0;
  double steer_rad = 0.0;
  double steering_angle_deg = 0.0;
  bool steering_pressed = false;
  double yaw_rate = 0.0;
  bool left_blinker = false;
  bool right_blinker = false;
};

struct LanePathMsg {
  int64_t timestamp_us = 0;
  int64_t capture_ts_us = 0;
  int64_t infer_ts_us = 0;
  int frame_id = 0;
  std::vector<Vec2> polyline;

  std::vector<Vec2> plan_poly;
  std::vector<double> plan_yaw;
  std::vector<double> plan_yaw_rate;

  bool lane_anchored = false;
  bool lanelines_active = true;
  double lane_width_m = 0.0;

  double center_force_m = 0.0;

  double lane_offset_m = 0.0;

  double p_lane_blend_scale = 0.0;
  double p_camera_offset_m = 0.0;
  double p_center_force_gain = 0.0;
};

struct GpsSample {
  int64_t timestamp_us = 0;
  double x = 0.0;
  double y = 0.0;
  double speed_mps = 0.0;
  double bearing_deg = 0.0;

  double yaw_enu = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  bool course_valid = false;
  bool valid = false;
  double accuracy_m = 0.0;
  int satellites = 0;
};

struct RawImuSample {
  int64_t timestamp_us = 0;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
  bool valid = false;
};

struct ImuSample {
  int64_t timestamp_us = 0;
  double yaw_rate = 0.0;
  bool valid = false;
  double lat_accel = 0.0;
  bool lat_accel_valid = false;
};

struct LocalizationPose {
  int64_t timestamp_us = 0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double yaw_rate = 0.0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double ekf_x = 0.0;
  double ekf_y = 0.0;
  double road_roll_deg = 0.0;
  double road_roll_std_deg = 10.0;
  bool road_roll_valid = false;
  double learned_stiffness_factor = 0.0;
  double learned_steer_ratio = 0.0;
  double learned_angle_offset_deg = 0.0;
  double learned_stiffness_std = 0.0;
  double learned_steer_ratio_std = 0.0;
  bool learned_params_valid = false;
  int learned_sample_count = 0;
};

struct LaneUvMsg {
  int64_t timestamp_us = 0;
  std::vector<Vec2> left_uv;
  std::vector<Vec2> right_uv;
};

struct CameraOdometrySample {
  int64_t timestamp_us = 0;
  Vec3 trans = Vec3::Zero();
  Vec3 rot = Vec3::Zero();
  Vec3 trans_std = Vec3::Ones();
  Vec3 rot_std = Vec3::Ones();
  bool valid = false;
};

/** Output of the lateral loop. It lives here rather than in the service header because it is a
 *  domain structure, and the conversion functions must not include a service for it — that would
 *  make the headers circular. */
struct LaneKeepOutput {
  int64_t timestamp_us = 0;
  int64_t capture_ts_us = 0;
  int64_t vision_ts_us = 0;
  int64_t chassis_ts_us = 0;
  int64_t publish_ts_us = 0;
  double steer_rad = 0.0;
  double steer_norm = 0.0;
  double max_steer_rad = 0.0;
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

    bool assist_allowed = false;
    bool assist_known = false;

    bool lane_anchored = false;
    bool lanelines_active = true;
    double road_roll_deg = 0.0;
    std::string kappa_solver;
    double pid_p = 0.0;
    double pid_i = 0.0;
    double pid_f = 0.0;
    double lane_width_m = 0.0;
    double lane_offset_m = 0.0;
    double center_force_m = 0.0;
    double p_lane_blend_scale = 0.0;
    double p_camera_offset_m = 0.0;
    double p_center_force_gain = 0.0;
  } dbg;
};

struct CameraCalibrationState {
  int64_t timestamp_us = 0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double camera_height_m = 1.22;
  double fx = 930.0;
  double fy = 930.0;
  double cx = 640.0;
  double cy = 360.0;
  bool calibration_success = false;
  int n_updates = 0;
  double vp_u = 0.0;
  double vp_v = 0.0;
  bool has_vp = false;
  int cal_percent = 0;
  int cal_status = 0;
};

}  // namespace adas
