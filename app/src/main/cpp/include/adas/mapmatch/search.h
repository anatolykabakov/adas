#pragma once

#include <cstdint>
#include <vector>

#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/track.h"

namespace adas {
namespace mapmatch {
struct SearchConfig {
  double turn_tol_deg = 25.0;        ///< Heading tolerance when matching a turn [deg].
  double dist_tol_rel = 0.15;        ///< Relative tolerance on a section length.
  double dist_tol_abs_m = 45.0;      ///< Absolute tolerance on a section length [m].
  double dist_tol_min_m = 90.0;      ///< Floor under that tolerance for short sections [m].
  bool check_first_straight = true;  ///< Require the first straight to match before expanding a candidate.
  bool verbose = false;              ///< Log every candidate; for debugging a failed match, unusable on a drive.
  double straight_max_deg = 30.0;    ///< Total turning still counted as straight [deg].
  double turn_max_len_m = 150.0;     ///< Longer than this and it is a curve, not a turn [m].
  double turn_start_min_deg = 12.0;  ///< Turning needed before a turn is considered started [deg].
  int beam_width = 400;              ///< Candidates kept per expansion step.
  int max_candidates = 60;           ///< Candidates returned.
  double min_seed_edge_m = 20.0;     ///< Shorter edges are not used to seed a search [m].
  double max_overshoot_rel = 1.5;    ///< How far past the expected length a candidate may run before it is dropped.
  double heading_sigma_deg = 12.0;   ///< Assumed heading noise [deg].
  double heading_step_m = 25.0;      ///< Spacing at which heading is compared along an edge [m].
};

struct RouteCandidate {
  std::vector<std::uint32_t> dir_edges;
  double cost = 0.0;
  double length_m = 0.0;
  double start_x_m = 0.0;
  double start_y_m = 0.0;
  double start_heading_rad = 0.0;
  int matched_turns = 0;
};

std::vector<RouteCandidate> searchRoutes(const RoadMap& map, const Track& track, const SearchConfig& cfg = {});

}  // namespace mapmatch
}  // namespace adas
