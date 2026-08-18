#pragma once

#include "adas/utils/adas_topics.h"
#include "messages.pb.h"

namespace adas {
/** Knobs loaded from config / the phone.
 *
 *  Stock comma `LanePlanner.get_d_path` only uses the camera offset. The rest are kept so the
 *  JSON schema and runtime registry stay stable; they do not change the mix. */
struct LanePathConfig {
  float min_lane_prob = 0.3f;

  double lane_blend_scale = 1.0;

  double camera_offset_m = 0.0;  ///< Added to the fused path, device Y right+ [m]. Same role as comma CAMERA_OFFSET.

  double lane_std_good_m = 0.2;
  double lane_std_bad_m = 1.5;
  double lane_std_range_m = 20.0;
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

/** Persistent filters from comma `LanePlanner`: width estimate (rc=9.95) and certainty (rc=0.95). */
struct LaneFusionState {
  double width_est_m = 3.7;
  double width_certainty = 1.0;
  double lane_width_m = 3.7;
  bool inited = true;
  bool lanelines_active = true;
  void reset() { *this = LaneFusionState{}; }
};

/** Fuse host lane lines into the model plan. Port of comma/openpilot `LanePlanner.get_d_path`.
 *
 *  \param v_ego ego speed [m/s], used for the speed-based fallback width and the look-ahead
 *               width check at 0 / 1.5 / 3.0 s. */
LanePathMsg laneLinesToPath(const adas::proto::LaneLines& ll, const LanePathConfig& cfg = {},
                            LaneFusionState* state = nullptr, double v_ego = 0.0);

}  // namespace adas
