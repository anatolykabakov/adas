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
  double window_m = 250.0;  ///< Length of the window matched at a time [m].
  double tol_deg = 4.0;     ///< Heading tolerance inside the window [deg].
  double clip_deg = 45.0;   ///< Per-sample heading difference is clipped here, so one outlier cannot dominate [deg].
  int beam = 4000;          ///< Candidates kept per step.
  int per_edge = 3;         ///< Candidates kept per edge, so one street cannot fill the beam.
  double cell_m = 300.0;    ///< Grid cell for spreading candidates over space [m].
  int per_cell = 6;         ///< Candidates kept per cell.
  double defer_deg = 90.0;  ///< Above this ambiguity the decision is deferred to a wider beam [deg].
  int defer_beam = 60000;   ///< Beam used for those deferred cases.
  int max_expand = 64;      ///< Expansion steps before the search gives up.
  bool verbose = false;     ///< Log every candidate; debugging only.
};

struct WindowRoute {
  std::vector<std::uint32_t> dir_edges;
  double cost = 0.0;
};

/**
 * \param window_deg per-window heading increments of the track, degrees
 * \return routes carried to the end of the profile, best first
 */
std::vector<WindowRoute> searchByWindows(const RoadMap& map, const std::vector<double>& window_deg,
                                         const WindowSearchConfig& cfg);

}  // namespace mapmatch
}  // namespace adas
