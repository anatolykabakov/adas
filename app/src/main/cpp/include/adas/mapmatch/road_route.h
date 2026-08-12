#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "adas/mapmatch/dir_edge.h"
#include "adas/mapmatch/road_map.h"

namespace adas {
namespace mapmatch {
/** Road curvature ahead from the OSM graph — the map half of what dragonpilot's `selfdrive/mapd` does.
 *
 *  The chain is: snap the car onto a directed edge, grow a route forward by "keep going straight, prefer the
 *  same road name", resample that centreline by arc length, differentiate heading to get curvature, and cut
 *  the result into turn sections with a speed each. Thresholds are dragonpilot's, so numbers here are
 *  comparable with `liveMapData`: turn if |kappa| >= 0.002 1/m, speed = sqrt(2.6 / kappa).
 *
 *  What this is *not*: a navigation route. There is no destination — the walk is a guess at where the road
 *  goes, which is exactly what dragonpilot does, and it is wrong the moment the driver takes an exit. It is
 *  useful anyway because the failure is visible: the route diverges from the driven track, and the logged
 *  `match_dist_m` says so.
 *
 *  Accuracy is bounded by the map, not by the maths. `Moscow.osm.admap` is a 46k-node graph of the road
 *  network drawn as single centrelines: a dual carriageway is one line, a junction is one node, and lane
 *  geometry does not exist. Curvature from it is the curvature of the *road*, good to roughly the OSM
 *  drawing error (metres), and it is not a substitute for the vision path — see `utils/curvature_preview.h`
 *  for the one the controller actually uses. */
struct RouteConfig {
  double horizon_m = 2000.0;

  double match_search_m = 120.0;
  double max_match_dist_m = 40.0;
  double max_match_heading_deg = 60.0;
  double heading_weight_m_per_rad = 40.0;

  double step_m = 5.0;
  double window_m = 25.0;

  double turn_kappa = 0.002;
  double max_lat_acc = 2.6;
  double min_section_m = 100.0;
  double max_curv_deviation = 2.0;
  double max_curv_split_arc_deg = 90.0;

  double straight_max_deg = 60.0;
};

/** A stretch of road ahead whose curvature exceeds `turn_kappa`, with the speed that holds
 *  `max_lat_acc` through its sharpest point. Distances are along the route from the car. */
struct TurnSection {
  double start_m = 0.0;
  double end_m = 0.0;
  double kappa = 0.0;
  double speed_mps = 0.0;
  int sign = 0;
};

struct RouteAhead {
  bool matched = false;
  double match_dist_m = -1.0;
  double heading_delta_rad = 0.0;
  std::uint32_t dir_edge = kNoEdge;
  std::string road_name;

  double x_m = 0.0;
  double y_m = 0.0;

  std::vector<std::uint32_t> dir_edges;

  std::vector<double> s_m;
  std::vector<double> x_m_pts;
  std::vector<double> y_m_pts;
  std::vector<double> kappa;

  std::vector<TurnSection> turns;
  double length_m = 0.0;
  double node_spacing_m = 0.0;
};

/** Snap (`x`, `y`, `yaw_rad`) onto the graph and grow the route ahead. Position in map-frame metres, heading
 *  in ENU radians (0 = east, counter-clockwise), i.e. the convention `LocalizationPose::yaw` uses. */
RouteAhead buildRouteAhead(const RoadMap& map, double x, double y, double yaw_rad, const RouteConfig& cfg = {});

/** Match only, without growing a route. Returns `kNoEdge` if nothing acceptable is within
 *  `cfg.match_search_m`. On success writes the snapped point, its distance and the heading error. */
std::uint32_t matchDirectedEdge(const RoadMap& map, double x, double y, double yaw_rad, const RouteConfig& cfg,
                                double* snap_x, double* snap_y, double* dist_m, double* heading_delta_rad,
                                double* s_into_edge_m);

/** Heading of a polyline as a function of arc length: one sample per segment, placed at the segment's
 *  midpoint, with the angle unwrapped so it can be interpolated.
 *
 *  The midpoint is the whole trick. On a circle, the chord between two points is parallel to the tangent at
 *  the arc midpoint *exactly* — so for a polyline inscribed in a curve, (midpoint, chord heading) samples
 *  the true heading profile with no error from how far apart the nodes are. Interpolating between those
 *  samples recovers theta(s), and its slope is the curvature. */
struct HeadingProfile {
  std::vector<double> s_m;
  std::vector<double> theta_rad;
  double length_m = 0.0;
};

HeadingProfile headingProfile(const std::vector<double>& xs, const std::vector<double>& ys);

/** Curvature at each `query_s`, 1/m, positive left: the heading change across a `window_m` span centred on
 *  the query, divided by that span. Near the ends the window shrinks to what exists.
 *
 *  `src_x`/`src_y` must be the *map* geometry, not a resampled copy of it: resampling turns the heading
 *  profile back into a staircase and reintroduces exactly the node-spacing sensitivity this avoids. Measured
 *  on a 250 m arc, this returns 1/R to within 0.01 % at 2 m node spacing and 0.11 % at 40 m — where taking
 *  the heading difference between resampled points instead ripples by 12 %.
 *
 *  This replaces dragonpilot's "insert a node every 15 m where they are more than 50 m apart, then fit a
 *  spline". That interpolates geometry the map does not have; this reads the geometry the map does have. */
std::vector<double> curvatureAlong(const std::vector<double>& src_x, const std::vector<double>& src_y,
                                   const std::vector<double>& query_s, double window_m);

/** Cut a curvature profile into turn sections. `s` and `kappa` must be the same length. */
std::vector<TurnSection> turnSections(const std::vector<double>& s, const std::vector<double>& kappa,
                                      const RouteConfig& cfg);

/** Resample a polyline at fixed arc-length steps. This is what replaces dragonpilot's "insert points every
 *  15 m where nodes are more than 50 m apart, then spline": a fixed step plus the heading window below give
 *  the same node-spacing independence without carrying a spline fitter onto the phone. */
void resamplePolyline(const std::vector<double>& xs, const std::vector<double>& ys, double step_m,
                      std::vector<double>& out_s, std::vector<double>& out_x, std::vector<double>& out_y);

}  // namespace mapmatch
}  // namespace adas
