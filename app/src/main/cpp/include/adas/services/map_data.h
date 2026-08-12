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
class MapData : public adas::middleware::Service {
public:
  struct Config {
    std::string map_path = "Moscow.osm.admap";

    double update_hz = 2.0;

    double local_map_period_s = 5.0;
    double local_map_radius_m = 400.0;

    double min_speed_mps = 1.5;

    double max_pose_gap_m = 150.0;

    double max_fix_age_s = 30.0;

    mapmatch::RouteConfig route{};
  };

  MapData() : MapData(Config{}) {}
  explicit MapData(Config config);

  std::string_view getName() const override { return "map_data"; }

  void configure() override;
  void reset() override;

  const Config& config() const { return config_; }
  bool mapLoaded() const { return map_loaded_; }
  const mapmatch::RoadMap& map() const { return map_; }
  const mapmatch::RouteAhead& lastRoute() const { return route_; }

private:
  void onGpsData(const adas::proto::GPSData& payload);
  void onPose(const adas::proto::LocalizationPose& payload);
  void onTick();

  bool loadMap();
  bool currentPosition(double& x, double& y, double& yaw) const;
  void fillLocalMap(adas::proto::MapLocalState& out, double x, double y);

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
