#pragma once

#include <cstdint>
#include <vector>

#include "adas/mapmatch/road_map.h"

namespace adas {
namespace mapmatch {

/**
 * Route search over heading increments on fixed-length path windows.
 *
 * <p>Neither the whole heading profile nor a turn-and-leg chain can be matched directly: gyro bias
 * (0.15 deg/s measured) drifts the profile by hundreds of degrees over a drive, and a gradual exit that
 * odometry traces as an arc appears as a single kink on the map. A heading increment over 250 m is free
 * of both — the drift is under 3 deg and an arc accumulates the same degrees as the kink.
 */
struct WindowSearchConfig {
  double window_m = 250.0;  ///< window length along the path
  double tol_deg = 4.0;     ///< disagreement within tolerance is not penalised
  double clip_deg = 45.0;   ///< penalty ceiling for one window
  int beam = 4000;          ///< states kept per layer
  int per_edge = 3;         ///< states per edge
  double cell_m = 300.0;    ///< cell size for spatial diversity
  int per_cell = 6;         ///< states per cell, or the beam collapses into one area
  double defer_deg = 90.0;  ///< below this much information, no cost-based pruning
  int defer_beam = 60000;   ///< but the layer is still bounded, or growth is explosive
  int max_expand = 64;      ///< safety stop on growth inside a window
  bool verbose = false;
};

struct WindowRoute {
  std::vector<std::uint32_t> dir_edges;
  double cost = 0.0;
};

/**
 * @param window_deg per-window heading increments of the track, degrees
 * @return routes carried to the end of the profile, best first
 */
std::vector<WindowRoute> searchByWindows(const RoadMap& map, const std::vector<double>& window_deg,
                                         const WindowSearchConfig& cfg);

}  // namespace mapmatch
}  // namespace adas
