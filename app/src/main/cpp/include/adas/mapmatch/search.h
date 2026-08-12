#pragma once

#include <cstdint>
#include <vector>

#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/track.h"

namespace adas {
namespace mapmatch {
struct SearchConfig {
  double turn_tol_deg = 25.0;
  double dist_tol_rel = 0.15;
  double dist_tol_abs_m = 45.0;
  double dist_tol_min_m = 90.0;
  bool check_first_straight = true;
  bool verbose = false;
  double straight_max_deg = 30.0;
  double turn_max_len_m = 150.0;
  double turn_start_min_deg = 12.0;
  int beam_width = 400;
  int max_candidates = 60;
  double min_seed_edge_m = 20.0;
  double max_overshoot_rel = 1.5;
  double heading_sigma_deg = 12.0;
  double heading_step_m = 25.0;
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
