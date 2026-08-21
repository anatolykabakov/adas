#pragma once

#include "adas/utils/adas_topics.h"
#include "messages.pb.h"

namespace adas {
/** Knobs loaded from config / the phone.
 *
 *  Stock comma `LanePlanner.get_d_path` only uses the camera offset. The rest are kept so the
 *  JSON schema and runtime registry stay stable; they do not change the mix. */
struct LanePathConfig {
  float min_lane_prob = 0.3f;  ///< Below this the model is not claiming a lane line.

  double lane_blend_scale = 1.0;  ///< How much the lane centre pulls against the model plan; 0 disables.

  double camera_offset_m = 0.0;  ///< Added to the fused path, device Y right+ [m]. Same role as comma CAMERA_OFFSET.

  double lane_std_good_m = 0.2;     ///< Line sigma [m] treated as fully trustworthy.
  double lane_std_bad_m = 1.5;      ///< Line sigma [m] treated as useless.
  double lane_std_range_m = 20.0;   ///< Distance ahead over which sigma is taken [m].
  double lane_mode_off_prob = 0.3;  ///< Lane probability below which lane mode disengages.
  double lane_mode_on_prob = 0.5;   ///< Lane probability above which it engages (hysteresis).
  double lane_width_min_m = 2.6;    ///< Clamp on the estimated lane width [m].
  double lane_width_max_m = 4.6;    ///< Upper clamp on the width [m].
  double cam_y_left_m = 0.0;        ///< Camera offset from the centreline [m], positive left.

  double center_force_gain = 0.0;             ///< Pull toward the lane centre; 0 disables the correction.
  double center_force_max_m = 0.8;            ///< Clamp on that pull [m].
  double center_force_turn_scale = 0.7;       ///< Pull reduction in turns.
  double center_force_typical_width_m = 3.4;  ///< Width assumed when only one line is visible [m].
};

/** Persistent filters from comma `LanePlanner`: width estimate (rc=9.95) and certainty (rc=0.95). */
struct LaneFusionState {
  double width_est_m = 3.7;
  double width_certainty = 1.0;
  double lane_width_m = 3.7;
  bool inited = true;
  bool lanelines_active = true;
  /// Forget the fusion state.
  void reset() { *this = LaneFusionState{}; }
};

/** Fuse host lane lines into the model plan. Port of comma/openpilot `LanePlanner.get_d_path`.
 *
 *  \param v_ego ego speed [m/s], used for the speed-based fallback width and the look-ahead
 *               width check at 0 / 1.5 / 3.0 s. */
LanePathMsg laneLinesToPath(const adas::proto::LaneLines& ll, const LanePathConfig& cfg = {},
                            LaneFusionState* state = nullptr, double v_ego = 0.0);

}  // namespace adas
