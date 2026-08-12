#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "adas/adas_app.h"
#include "adas/mapmatch/road_map.h"
#include "messages.pb.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/map_data.h"
#include "adas/utils/adas_topics.h"

namespace {
#ifndef ADAS_TEST_MAP_FILE
#define ADAS_TEST_MAP_FILE ""
#endif

bool haveMap()
{
  const std::string path = ADAS_TEST_MAP_FILE;
  if (path.empty())
    return false;
  std::ifstream in(path, std::ios::binary);
  return in.good();
}

/** Collects whatever the service publishes so the test can assert on it. */
class Collector : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "collector"; }

  void configure() override
  {
    subscribe<adas::proto::MapLocalState>(adas::topics::kMapLocal,
                                          [this](const adas::proto::MapLocalState& m) { msgs.push_back(m); });
  }

  std::vector<adas::proto::MapLocalState> msgs;
};

class Injector : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "injector"; }
  void configure() override {}

  void gps(int64_t t_ms, double lat, double lon, double speed, double bearing)
  {
    adas::proto::GPSData msg;
    auto* g = &msg;
    g->set_timestamp(t_ms);
    g->set_latitude(lat);
    g->set_longitude(lon);
    g->set_speed(static_cast<float>(speed));
    g->set_bearing(static_cast<float>(bearing));
    publish(adas::topics::kGpsData, msg);
  }

  void pose(int64_t t_ms, double x, double y, double yaw, double v)
  {
    adas::proto::LocalizationPose msg;
    auto* p = &msg;
    p->set_timestamp(t_ms);
    p->set_x(x);
    p->set_y(y);
    p->set_yaw(yaw);
    p->set_v(v);
    publish(adas::topics::kLocalizationPose, msg);
  }
};

/** `middleware::Manager::step()` drains inboxes before firing timers, so a message published from a timer reaches
 *  its subscriber on the *next* step. Every assertion here is about what a timer published, so always pump
 *  twice. */
void pump(adas::middleware::Manager& bus, uint64_t t_us)
{
  bus.setTime(t_us);
  bus.step();
  bus.step();
}

}  // namespace

// it.
TEST(MapData, PublishesMatchedRouteFromGpsAndPose)
{
  if (!haveMap())
    GTEST_SKIP() << "no map file at " ADAS_TEST_MAP_FILE;

  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);

  adas::services::MapData::Config cfg;
  cfg.map_path = ADAS_TEST_MAP_FILE;
  cfg.update_hz = 10.0;
  cfg.local_map_period_s = 0.0;
  cfg.local_map_radius_m = 300.0;

  auto svc = bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();
  ASSERT_TRUE(svc->mapLoaded());

  pump(bus, 1'000'000);

  const double lat = 55.70946103, lon = 37.73241634;
  inj->pose(1000, 0.0, 0.0, 2.97, 20.0);
  inj->gps(1000, lat, lon, 20.0, 30.0);
  bus.step();

  pump(bus, 1'200'000);

  ASSERT_FALSE(col->msgs.empty());
  const auto& m = col->msgs.back();
  EXPECT_TRUE(m.map_loaded());
  ASSERT_TRUE(m.matched()) << "match distance " << m.match_dist_m();
  EXPECT_LT(m.match_dist_m(), 40.0);
  EXPECT_GT(m.route_x_size(), 100) << "a 2 km route at 5 m steps";
  EXPECT_EQ(m.route_x_size(), m.route_kappa_size());
  EXPECT_GT(m.route_length_m(), 500.0);
  EXPECT_FALSE(m.road_name().empty());
  EXPECT_TRUE(m.has_local_map());
  EXPECT_GT(m.local_edges_size(), 0);
  EXPECT_NEAR(m.lat(), lat, 1e-6);
  EXPECT_NEAR(m.lon(), lon, 1e-6);
}

// drive while everything else looks healthy.
TEST(ShippedConfig, TheMapServiceIsOnAndNamesAnAssetThatExists)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  EXPECT_TRUE(cfg.feature_flags.enable_map_data) << "shipped config has map_data off";
  ASSERT_FALSE(cfg.map_data.map_path.empty());
  EXPECT_NE(cfg.map_data.map_path.front(), '/') << "map.path names an APK asset, not a device path";

  const std::string dir = std::string(path).substr(0, std::string(path).find_last_of('/') + 1);
  const std::string asset = dir + cfg.map_data.map_path;
  std::ifstream in(asset, std::ios::binary);
  if (!in.good()) {
    GTEST_SKIP() << asset << " not built yet — `gradle syncRoadMap` copies it from maps/";
  }

  adas::mapmatch::RoadMap m;
  ASSERT_TRUE(m.load(asset)) << asset << " is not a loadable ADASMAP1 file";
  EXPECT_GT(m.edgeCount(), 1000u);
}

// them: the route is what rides on every message, the surrounding graph only every `local_map_period_s`.
TEST(MapData, LoggedMessageStaysSmallEnoughToRecord)
{
  if (!haveMap())
    GTEST_SKIP() << "no map file at " ADAS_TEST_MAP_FILE;

  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = ADAS_TEST_MAP_FILE;
  cfg.update_hz = 10.0;
  cfg.local_map_period_s = 1e9;

  bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();

  pump(bus, 1'000'000);
  inj->pose(1000, 0.0, 0.0, 2.97, 20.0);
  inj->gps(1000, 55.70946103, 37.73241634, 20.0, 30.0);
  bus.step();
  pump(bus, 1'200'000);

  ASSERT_TRUE(col->msgs.back().matched());
  const std::size_t route_only = col->msgs.back().ByteSizeLong();

  adas::middleware::Manager bus2(adas::middleware::Manager::Mode::Simulated);
  cfg.local_map_period_s = 0.0;
  cfg.local_map_radius_m = 400.0;
  bus2.registerService<adas::services::MapData>(cfg);
  auto inj2 = bus2.registerService<Injector>();
  auto col2 = bus2.registerService<Collector>();

  pump(bus2, 1'000'000);
  inj2->pose(1000, 0.0, 0.0, 2.97, 20.0);
  inj2->gps(1000, 55.70946103, 37.73241634, 20.0, 30.0);
  bus2.step();
  pump(bus2, 1'200'000);

  ASSERT_TRUE(col2->msgs.back().matched());
  const std::size_t with_map = col2->msgs.back().ByteSizeLong();

  std::printf("[  BAG COST ] route-only %zu B, +local map %zu B (%d edges)\n", route_only, with_map,
              col2->msgs.back().local_edges_size());

  // service would have to earn its place before it is left on.
  EXPECT_LT(route_only, 8u * 1024u);
  EXPECT_LT(with_map, 64u * 1024u);
}

// is indistinguishable from a dead service, and this is a logging service above all.
TEST(MapData, PublishesUnmatchedWithoutAFix)
{
  if (!haveMap())
    GTEST_SKIP() << "no map file at " ADAS_TEST_MAP_FILE;

  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = ADAS_TEST_MAP_FILE;
  cfg.update_hz = 10.0;

  bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();

  pump(bus, 1'000'000);
  inj->pose(1000, 0.0, 0.0, 1.0, 15.0);
  pump(bus, 1'200'000);

  ASSERT_FALSE(col->msgs.empty());
  EXPECT_TRUE(col->msgs.back().map_loaded());
  EXPECT_FALSE(col->msgs.back().matched());
}

// route would be built somewhere the car has long left.
TEST(MapData, StopsMatchingWhenTheFixGoesStale)
{
  if (!haveMap())
    GTEST_SKIP() << "no map file at " ADAS_TEST_MAP_FILE;

  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = ADAS_TEST_MAP_FILE;
  cfg.update_hz = 10.0;
  cfg.max_fix_age_s = 5.0;

  bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();

  pump(bus, 10'000'000);
  inj->pose(10'000, 0.0, 0.0, 2.97, 20.0);
  inj->gps(10'000, 55.70946103, 37.73241634, 20.0, 30.0);
  pump(bus, 10'200'000);
  ASSERT_TRUE(col->msgs.back().matched());

  col->msgs.clear();
  pump(bus, 20'000'000);
  ASSERT_FALSE(col->msgs.empty());
  EXPECT_FALSE(col->msgs.back().matched());
}
