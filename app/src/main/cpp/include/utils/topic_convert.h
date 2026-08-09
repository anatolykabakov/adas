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

  /** Range over which lane σ is summarised into a blending confidence.
   *
   *  Was a fixed 5–40 m. Measured on run 2026_08_06_00_36_42, worst-line σ by band:
   *
   *  | segment | 5–20 m | 5–40 m | 20–40 m | 40–80 m |
   *  |---|---|---|---|---|
   *  | straight | 0.11 | 0.14 | 0.20 | 0.37 |
   *  | left arc | 0.58 | 0.89 | 1.49 | 4.32 |
   *  | right arc | 0.34 | 0.50 | 0.78 | 1.66 |
   *
   *  σ roughly doubles from the near half to the far half and quadruples beyond 40 m, because on a
   *  bend the inner line leaves the frame and its far samples are extrapolation. Summarising over
   *  5–40 m therefore lets "I have not seen that far" veto a line whose near half is fine, and the
   *  1.5 m cut-off then rejects 20 % of left-arc frames instead of 5 %.
   *
   *  Raising `lane_std_bad_m` does not fix this: swept offline over the same run, 2.0 leaves right-arc
   *  blending at exactly 0.00 (σ there is 2.48) and 2.5 buys 0.05 m. The window was the wrong lever.
   *
   *  20 m is the near field the command actually acts on; the far part of the reference is recomputed
   *  every frame long before the car reaches it. */
  double lane_std_range_m = 20.0;

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
