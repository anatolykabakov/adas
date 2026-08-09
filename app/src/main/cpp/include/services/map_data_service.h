#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mapmatch/road_map.h"
#include "mapmatch/road_route.h"
#include "messages.pb.h"
#include "middleware/middleware.hpp"
#include "utils/adas_topics.h"

namespace adas {

/** Road curvature ahead from the OSM map, published on `map/local` and recorded in the bag.
 *
 *  This is dragonpilot's `selfdrive/mapd` with the parts that do not apply here removed. dragonpilot queries
 *  Overpass over the network for a 3 km circle and rebuilds its way collection when the car nears the edge of
 *  it; we already ship the whole of Moscow as a 4.9 MB `ADASMAP1` file that loads in one read
 *  (`docs/MAPMATCH.md`), so there is no query, no thread, and no cache — just a lookup. What is kept is the
 *  chain that turns a road graph into speeds: snap, walk forward, curvature, turn sections,
 *  `v = sqrt(a_lat / kappa)`.
 *
 *  Nothing consumes it yet, by design. The output goes into the bag so a run can be examined offline
 *  (`app/src/main/scripts/bag_map_data.py`) and compared against what the vision-based curvature preview
 *  said at the same moment. Wiring map speeds into `long_plan` is a decision that needs that comparison
 *  first: the map is a single centreline drawn by hand, and on the runs recorded so far it matches the
 *  driven road to within a few metres in some places and 100 m in others.
 *
 *  Position comes from two sources on purpose. The map needs geodetic coordinates, and only the raw GPS
 *  message carries them; but GPS on these runs arrives at 0.1-1 Hz and jumps, while `localization/pose` is
 *  a 100 Hz fused estimate in a local frame with an origin this service does not know. So each fix anchors
 *  the map frame, and pose deltas carry the position between fixes. The drift that accumulates in between is
 *  measured, not assumed: `pose_gps_gap_m` is the error at the moment the next fix arrives. */
class MapDataService : public adas::Service {
public:
  struct Config {
    /** Map file. On Android this is overwritten by `nativeStart` with the absolute path of the unpacked
     *  asset; the name here is what Java looks up in the APK (`map.path` in `config.json` — one key, read by
     *  both sides, so they cannot drift). If relative and not overridden, the service falls back to a few
     *  known locations. Empty disables it without removing it from the graph. */
    std::string map_path = "Moscow.osm.admap";

    /** Rate the route is rebuilt and published at. dragonpilot runs its mapd at 1 Hz; 2 Hz here costs a
     *  route build (0.02 ms) plus 5.2 kB of bag per tick, and makes the logged track smooth enough to plot.
     *  Measured cost at the defaults is about 45 MB/hour — the same class as `vision/model_long`, and an
     *  order of magnitude under `localization/pose`. */
    double update_hz = 2.0;

    /** The surrounding graph is 15 kB — three times the route — and does not change between ticks, so it
     *  rides along only this often. Set the period to 0 to attach it to every message. */
    double local_map_period_s = 5.0;
    double local_map_radius_m = 400.0;

    /** Below this speed the heading is not trustworthy enough to pick a direction of travel, so the last
     *  good one is reused — the same reason dragonpilot stops updating its route near a full stop. */
    double min_speed_mps = 1.5;

    /** A fix further than this from where the pose said we were is treated as the pose having drifted, not
     *  as a bad fix: the anchor is reset. Only affects logging of `pose_gps_gap_m` and the reset itself. */
    double max_pose_gap_m = 150.0;

    /** Age past which a GPS fix no longer anchors anything and the service stops publishing. At 0.1 Hz
     *  fixes, the default tolerates a two-fix gap. */
    double max_fix_age_s = 30.0;

    mapmatch::RouteConfig route{};
  };

  MapDataService() : MapDataService(Config{}) {}
  explicit MapDataService(Config config);

  std::string_view getName() const override { return "map_data"; }

  void configure() override;
  void reset() override;

  const Config& config() const { return config_; }
  bool mapLoaded() const { return map_loaded_; }
  const mapmatch::RoadMap& map() const { return map_; }
  const mapmatch::RouteAhead& lastRoute() const { return route_; }

private:
  void onGpsData(const ai::flow::adas::ZMQMessage& msg);
  void onPose(const ai::flow::adas::ZMQMessage& msg);
  void onTick();

  bool loadMap();
  bool currentPosition(double& x, double& y, double& yaw) const;
  void fillLocalMap(ai::flow::adas::MapLocalState& out, double x, double y);

  Config config_;
  mapmatch::RoadMap map_;
  bool map_loaded_ = false;
  std::string map_resolved_path_;

  // Anchor: map-frame position of the last fix, and the pose reading at that instant.
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

}  // namespace adas
