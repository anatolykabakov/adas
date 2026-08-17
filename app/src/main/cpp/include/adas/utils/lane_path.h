#pragma once

#include "adas/utils/adas_topics.h"
#include "messages.pb.h"

namespace adas {
struct LanePathConfig {
  float min_lane_prob = 0.3f;  ///< Lane-line confidence below which the line is ignored.

  double lane_blend_scale = 1.0;  ///< How strongly the lane centre pulls the path against the model plan.

  double camera_offset_m = 0.0;  ///< Camera offset used when measuring the in-lane position [m].

  double lane_std_good_m = 0.2;  ///< Line spread treated as fully trustworthy [m].
  double lane_std_bad_m = 1.5;   ///< Line spread treated as useless [m]; between the two the weight ramps.

  double lane_std_range_m = 20.0;  ///< Distance ahead over which that spread is measured [m].

  double lane_mode_off_prob = 0.3;  ///< Confidence at which lane following switches off (hysteresis low edge).
  double lane_mode_on_prob = 0.5;   ///< Confidence at which it switches on (hysteresis high edge).

  double lane_width_min_m = 2.6;  ///< Narrower than this and the measured width is rejected [m].
  double lane_width_max_m = 4.6;  ///< Wider than this and it is rejected [m].

  double cam_y_left_m = 0.0;  ///< Camera offset from the centreline [m], positive left.

  double center_force_gain = 0.0;   ///< Gain pulling the path towards the lane centre; 0 disables it.
  double center_force_max_m = 0.8;  ///< Cap on that pull [m].

  double center_force_turn_scale = 0.7;       ///< How much the pull is reduced in a turn, where cutting in is natural.
  double center_force_typical_width_m = 3.4;  ///< Lane width the pull is normalised against [m].
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
