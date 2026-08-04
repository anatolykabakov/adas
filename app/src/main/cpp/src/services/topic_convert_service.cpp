#include "services/topic_convert_service.h"

#include <algorithm>

#include "utils/logger.h"
#include "utils/topic_convert.h"

namespace adas {

TopicConvertService::TopicConvertService(Config config) : config_(config), steer_ratio_(config.steer_ratio)
{
  path_cfg_.min_lane_prob = 0.3f;
  path_cfg_.lane_blend_scale = config.path_lane_blend_scale;
  path_cfg_.camera_offset_m = config.path_camera_offset_m;
  path_cfg_.lane_std_good_m = config.lane_std_good_m;
  path_cfg_.lane_std_bad_m = config.lane_std_bad_m;
  path_cfg_.lane_width_min_m = config.lane_width_min_m;
  path_cfg_.lane_width_max_m = config.lane_width_max_m;
  path_cfg_.cam_y_left_m = config.cam_y_left_m;
  path_cfg_.center_force_gain = config.center_force_gain;
  path_cfg_.center_force_max_m = config.center_force_max_m;
  path_cfg_.center_force_turn_scale = config.center_force_turn_scale;
}

void TopicConvertService::setLaneBlendScale(double scale)
{
  const double v = std::clamp(scale, 0.0, 1.0);
  if (v == path_cfg_.lane_blend_scale)
    return;
  LOGI("TopicConvert: lane blend %.2f → %.2f", path_cfg_.lane_blend_scale, v);
  path_cfg_.lane_blend_scale = v;
  config_.path_lane_blend_scale = v;
}

void TopicConvertService::configure()
{
  subscribe<ai::flow::adas::ZMQMessage>(topics::kVisionLanes,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onVisionLanes(m); });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kVehicleState,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onVehicleState(m); });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kImu, [this](const ai::flow::adas::ZMQMessage& m) { onImu(m); });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kGpsLocation,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onGps(m); });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kCameraOdometry,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onCameraOdometry(m); });
  registerParameters();
  LOGI("TopicConvertService: lanes/state/imu/gps/cam_odo → path/chassis/imu_raw/gps(ENU)/cam_odo");
}

void TopicConvertService::registerParameters()
{
  registerParameter<double>(
      "path_lane_blend_scale", [this](const double& v) { setLaneBlendScale(v); },
      [this] { return path_cfg_.lane_blend_scale; });
  registerParameter<double>(
      "path_camera_offset_m",
      [this](const double& v) {
        path_cfg_.camera_offset_m = v;
        config_.path_camera_offset_m = v;
      },
      [this] { return path_cfg_.camera_offset_m; });
  registerParameter<double>(
      "center_force_gain",
      [this](const double& v) {
        path_cfg_.center_force_gain = std::max(0.0, v);
        config_.center_force_gain = path_cfg_.center_force_gain;
      },
      [this] { return path_cfg_.center_force_gain; });
  registerParameter<double>(
      "lane_std_good_m",
      [this](const double& v) {
        path_cfg_.lane_std_good_m = v;
        config_.lane_std_good_m = v;
      },
      [this] { return path_cfg_.lane_std_good_m; });
  registerParameter<double>(
      "lane_std_bad_m",
      [this](const double& v) {
        path_cfg_.lane_std_bad_m = v;
        config_.lane_std_bad_m = v;
      },
      [this] { return path_cfg_.lane_std_bad_m; });
  registerParameter<double>(
      "cam_y_left_m",
      [this](const double& v) {
        path_cfg_.cam_y_left_m = v;
        config_.cam_y_left_m = v;
      },
      [this] { return path_cfg_.cam_y_left_m; });
  registerParameter<double>(
      "steer_ratio",
      [this](const double& v) {
        steer_ratio_ = std::max(v, 1e-3);
        config_.steer_ratio = steer_ratio_;
      },
      [this] { return steer_ratio_; });
}

void TopicConvertService::reset()
{
  gps_proj_.reset();
  fusion_.reset();
}

void TopicConvertService::onVisionLanes(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_lane_lines()) {
    LOGW("vision/lanes without lane_lines payload");
    return;
  }
  auto path = laneLinesToPath(msg.lane_lines(), path_cfg_, &fusion_);
  LOGI("vision/lanes → path n=%zu frame=%d blend=%.2f cam_off=%.2f anchored=%d w=%.2f off=%+.2f center=%+.2f",
       path.polyline.size(), path.frame_id, path_cfg_.lane_blend_scale, path_cfg_.camera_offset_m,
       path.lane_anchored ? 1 : 0, path.lane_width_m, path.lane_offset_m, path.center_force_m);
  publish(topics::kVisionPath, path);
}

void TopicConvertService::onVehicleState(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_car_state())
    return;
  publish(topics::kVehicleChassis, carStateToChassis(msg.car_state(), steer_ratio_));
}

void TopicConvertService::onImu(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_imu_data())
    return;
  publish(topics::kImuRaw, imuToRaw(msg.imu_data()));
}

void TopicConvertService::onGps(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_gps_location())
    return;
  const auto& g = msg.gps_location();
  const int64_t t_us = g.timestamp() * 1000;
  const bool ok_fix =
      g.fix_type() != ai::flow::adas::GPSLocation::NO_FIX && g.fix_type() != ai::flow::adas::GPSLocation::TIME_ONLY;
  auto sample = gps_proj_.project(t_us, g.latitude(), g.longitude(), ok_fix, g.speed(), g.bearing());
  if (!sample.valid)
    return;
  publish(topics::kGpsLocation, sample);
}

void TopicConvertService::onLaneUv(const ai::flow::adas::ZMQMessage&) {}

void TopicConvertService::onCameraOdometry(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_camera_odometry())
    return;
  auto sample = cameraOdometryToSample(msg.camera_odometry());
  if (!sample.valid)
    return;
  publish(topics::kCameraOdometry, sample);
}

}  // namespace adas
