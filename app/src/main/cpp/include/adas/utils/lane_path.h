#pragma once

#include "adas/utils/adas_topics.h"
#include "messages.pb.h"

namespace adas {
struct LanePathConfig {
  float min_lane_prob = 0.3f;

  double lane_blend_scale = 1.0;

  double camera_offset_m = 0.0;

  double lane_std_good_m = 0.2;
  double lane_std_bad_m = 1.5;

  double lane_std_range_m = 20.0;

  bool lane_mode_hysteresis = true;
  double lane_mode_off_prob = 0.3;
  double lane_mode_on_prob = 0.5;

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
  bool lanelines_active = true;
  void reset() { *this = LaneFusionState{}; }
};

LanePathMsg laneLinesToPath(const adas::proto::LaneLines& ll, const LanePathConfig& cfg = {},
                            LaneFusionState* state = nullptr);

}  // namespace adas
