#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "utils/math_utils.h"

namespace adas {

namespace topics {
inline constexpr const char* kVehicleChassis = "vehicle/chassis";
inline constexpr const char* kVisionLanes = "vision/lanes";
inline constexpr const char* kVisionPath = "vision/path";
inline constexpr const char* kGpsLocation = "sensors/gps/location";
inline constexpr const char* kImu = "sensors/imu";
inline constexpr const char* kImuRaw = "sensors/imu_raw";
inline constexpr const char* kImuYaw = "sensors/imu_yaw";
inline constexpr const char* kLaneKeep = "control/lane_keep";
inline constexpr const char* kLaneKeepDebug = "control/lane_keep_debug";
inline constexpr const char* kLocalizationPose = "localization/pose";
inline constexpr const char* kSteerCommand = "controls/steer";
inline constexpr const char* kCameraCalib = "calibration/camera";
inline constexpr const char* kCameraCalibDebug = "calibration/camera_debug";
inline constexpr const char* kCalibLaneUv = "calibration/lane_uv";
inline constexpr const char* kCameraOdometry = "model/camera_odometry";
inline constexpr const char* kVisionModelLong = "vision/model_long";
inline constexpr const char* kLongPlan = "control/long_plan";
inline constexpr const char* kSafetyWarn = "safety/warn";
inline constexpr const char* kTrafficDetections = "vision/traffic_dets";
inline constexpr const char* kTrafficVision = "traffic/state";
inline constexpr const char* kVehicleState = "vehicle/state";
inline constexpr const char* kCanRx = "can/rx";
inline constexpr const char* kPandaHealth = "panda/health";
inline constexpr const char* kMiddlewareStats = "middleware/stats";
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
  std::vector<double> plan_yaw;       // rad
  std::vector<double> plan_yaw_rate;  // rad/s

  bool lane_anchored = false;
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
