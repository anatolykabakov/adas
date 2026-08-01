#pragma once

#include "messages.pb.h"
#include "utils/adas_topics.h"

namespace adas {

struct LanePathConfig {
  float min_lane_prob = 0.3f;

  double lane_blend_scale = 1.0;

  double camera_offset_m = 0.0;

  double lane_std_good_m = 0.2;
  double lane_std_bad_m = 1.5;

  double lane_width_min_m = 2.6;
  double lane_width_max_m = 4.6;

  double cam_y_left_m = 0.0;

  double center_force_gain = 0.0;
  double center_force_max_m = 0.8;

  double center_force_turn_scale = 0.7;
  double center_force_typical_width_m = 3.4;
};

struct LaneFusionState {
  double lane_width_m = 0.0;
  bool inited = false;
  void reset() { *this = LaneFusionState{}; }
};

LanePathMsg laneLinesToPath(const ai::flow::adas::LaneLines& ll, const LanePathConfig& cfg = {},
                            LaneFusionState* state = nullptr);

ChassisSample carStateToChassis(const ai::flow::adas::CarState& cs, double steer_ratio = 15.7);

RawImuSample imuToRaw(const ai::flow::adas::IMUData& imu);

CameraOdometrySample cameraOdometryToSample(const ai::flow::adas::CameraOdometry& odom);

}  // namespace adas
