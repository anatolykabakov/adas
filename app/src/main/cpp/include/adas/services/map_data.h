#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/road_route.h"
#include "messages.pb.h"
#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"

namespace adas {
namespace services {
/**
 * \brief Matches the pose onto the road graph and publishes the road ahead.
 *
 * \details Holds the compact OSM map, finds the edge under the car, and builds a route ahead along it,
 * so downstream consumers get speed limits, curvature and street names without knowing anything about
 * map formats. The local road geometry around the car goes out on a slower cadence than the match
 * itself, since it costs a bounding-box query over the graph.
 *
 * A match is only attempted on a trustworthy pose: a stale fix or a heading built from noise puts the
 * car on a parallel street, which is worse than reporting no match at all.
 */
class MapData : public adas::middleware::Service {
public:
  struct Config {
    /// Prebuilt map file; a relative path resolves against the app data directory.
    std::string map_path = "Moscow.osm.admap";

    double update_hz = 2.0;  ///< Match rate [Hz].

    double local_map_period_s = 5.0;    ///< How often the local road geometry is published [s]; it costs a bbox query.
    double local_map_radius_m = 400.0;  ///< Radius of that local geometry [m].

    double min_speed_mps = 1.5;  ///< Below this the heading is held rather than taken from the pose [m/s].

    double max_pose_gap_m = 150.0;  ///< Pose-to-fix disagreement above which the match is not trusted [m].

    double max_fix_age_s = 30.0;  ///< A fix older than this stops matching [s].

    mapmatch::RouteConfig route{};  ///< How the route ahead is built from the graph.
  };

  MapData() : MapData(Config{}) {}
  explicit MapData(Config config);

  std::string_view getName() const override { return "map_data"; }

  void configure() override;
  void reset() override;

  const Config& config() const { return config_; }
  /// False when the map file was missing or unreadable; the service then publishes an unmatched state.
  bool mapLoaded() const { return map_loaded_; }
  /// The loaded road graph, for consumers that need geometry this service does not publish.
  const mapmatch::RoadMap& map() const { return map_; }
  /// The route built on the last successful match. Stale after a match failure — check `matched`.
  const mapmatch::RouteAhead& lastRoute() const { return route_; }

private:
  void onGpsData(const adas::proto::GPSData& payload);
  void onPose(const adas::proto::LocalizationPose& payload);
  void onTick();
  /// Heading to match against, with the last trustworthy one held at low speed: GNSS course is noise
  /// below walking pace, and a route built on noise jumps between parallel roads.
  double yawWithHold(double yaw, double speed_mps);
  /// One line every 600 ticks: match rate, last match distance, route build time.
  void logProgress(double build_ms) const;

  bool loadMap();
  bool currentPosition(double& x, double& y, double& yaw) const;

  Config config_;
  mapmatch::RoadMap map_;
  bool map_loaded_ = false;
  std::string map_resolved_path_;

  bool have_anchor_ = false;
  double anchor_map_x_ = 0.0, anchor_map_y_ = 0.0;
  double anchor_pose_x_ = 0.0, anchor_pose_y_ = 0.0;
  bool anchor_has_pose_ = false;
  double last_lat_ = 0.0, last_lon_ = 0.0;
  int64_t last_fix_us_ = 0;
  double last_gap_m_ = 0.0;

  bool have_pose_ = false;
  double pose_x_ = 0.0, pose_y_ = 0.0, pose_yaw_ = 0.0, pose_v_ = 0.0;
  double gps_speed_mps_ = 0.0, gps_bearing_deg_ = 0.0;
  bool gps_course_valid_ = false;

  double last_good_yaw_ = 0.0;
  bool have_yaw_ = false;

  mapmatch::RouteAhead route_;
  int64_t last_local_map_us_ = 0;
  uint64_t ticks_ = 0;
  uint64_t matched_ticks_ = 0;
};

}  // namespace services

}  // namespace adas
