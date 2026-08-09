#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mapmatch/dir_edge.h"
#include "mapmatch/road_map.h"

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
  /** How far ahead to grow the route. 2 km at 25 m/s is 80 s of preview — far past anything the controller
   *  can use, and deliberately so: this is a logging and analysis service first. */
  double horizon_m = 2000.0;

  /** Map matching. `max_match_dist_m` is generous because the graph carries one centreline for a dual
   *  carriageway, so being 15-20 m off the drawn line is normal, not a mismatch. */
  double match_search_m = 120.0;
  double max_match_dist_m = 40.0;
  double max_match_heading_deg = 60.0;
  /** Metres of lateral error traded against one radian of heading error when picking the matched edge.
   *  At the default, 30 deg of heading error costs as much as 21 m of distance — enough to prefer the road
   *  we are actually driving along over a closer one crossing it. */
  double heading_weight_m_per_rad = 40.0;

  /** Curvature evaluation. `step_m` is where curvature is sampled (dragonpilot's spline evaluation step);
   *  `window_m` is the arc length the heading change is divided by.
   *
   *  Both are independent of how coarsely the map is drawn, which matters here: median node spacing on
   *  `Moscow.osm.admap` is 16 m but p90 is 67 m, and a third of all segments are longer than this window.
   *  See `curvatureAlong` for why that is not a problem. */
  double step_m = 5.0;
  double window_m = 25.0;

  /** Turn sections — dragonpilot's `_TURN_CURVATURE_THRESHOLD`, `_MAX_LAT_ACC`, `_MIN_SPEED_SECTION_LENGTH`,
   *  `_MAX_CURV_DEVIATION_FOR_SPLIT` and `_MAX_CURV_SPLIT_ARC_ANGLE`. */
  double turn_kappa = 0.002;
  double max_lat_acc = 2.6;
  double min_section_m = 100.0;
  double max_curv_deviation = 2.0;
  double max_curv_split_arc_deg = 90.0;

  /** A continuation is "straight enough" to be a candidate for the name/ref tie-break below this. Above it,
   *  the straightest branch wins outright. */
  double straight_max_deg = 60.0;
};

/** A stretch of road ahead whose curvature exceeds `turn_kappa`, with the speed that holds
 *  `max_lat_acc` through its sharpest point. Distances are along the route from the car. */
struct TurnSection {
  double start_m = 0.0;
  double end_m = 0.0;
  double kappa = 0.0;     // peak |curvature|, 1/m
  double speed_mps = 0.0; // sqrt(max_lat_acc / kappa)
  int sign = 0;           // +1 left, -1 right
};

struct RouteAhead {
  bool matched = false;
  /** Perpendicular distance from the car to the matched centreline. The honesty check on everything else:
   *  a route built from a 35 m match is a route on a road we are probably not on. */
  double match_dist_m = -1.0;
  double heading_delta_rad = 0.0;
  std::uint32_t dir_edge = kNoEdge;
  std::string road_name;

  /** Snapped position, map frame (east, north) metres. */
  double x_m = 0.0;
  double y_m = 0.0;

  std::vector<std::uint32_t> dir_edges;

  /** Centreline ahead, resampled every `step_m` starting at the snapped position. `s_m[0] == 0`. */
  std::vector<double> s_m;
  std::vector<double> x_m_pts;
  std::vector<double> y_m_pts;
  /** Curvature at each sample, 1/m, positive left. */
  std::vector<double> kappa;

  std::vector<TurnSection> turns;
  double length_m = 0.0;
  /** Median spacing of the map nodes the route was built from. Not used by anything here — logged because
   *  it is the resolution the curvature actually came from, and a 200 m median means the road ahead is three
   *  straight lines in the map whatever the curvature profile says. */
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
  std::vector<double> theta_rad;  // unwrapped
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
