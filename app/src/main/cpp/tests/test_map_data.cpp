#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "adas/adas_app.h"
#include "messages.pb.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/map_data.h"
#include "adas/utils/adas_topics.h"

namespace {
#ifndef ADAS_TEST_MAP_FILE
#define ADAS_TEST_MAP_FILE ""
#endif
// data/test_road.admap: a 24 kB ADASMAP1 map written for these tests, so they do not depend on the shipped
// 4 MB one. It holds "Test Highway" — 2.6 km starting at the loader's default frame anchor and pointing
// north: 1.2 km straight, a 600 m right-hand arc of radius 400 m, then 800 m straight — plus eight parallel
// service roads either side, which is what gives the local-map slice a realistic number of edges.
constexpr double kFixtureLat = 55.75569634;  // 200 m along the road, 2400 m of it still ahead
constexpr double kFixtureLon = 37.62080000;
constexpr double kFixtureYaw = 1.5708;  // ENU, from east — the road heads north here
constexpr char kFixtureRoadName[] = "Test Highway";

/** \brief The fixture is committed next to the tests, so its absence is a broken checkout, not a reason to
 *         skip: a skipped map test reads like a passing one. */
std::string fixtureMap()
{
  const std::string path = ADAS_TEST_MAP_FILE;
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in.good()) << "test fixture missing: " << path;
  return path;
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
  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);

  adas::services::MapData::Config cfg;
  cfg.map_path = fixtureMap();
  cfg.update_hz = 10.0;
  cfg.local_map_period_s = 0.0;
  cfg.local_map_radius_m = 300.0;

  auto svc = bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();
  ASSERT_TRUE(svc->mapLoaded());

  pump(bus, 1'000'000);

  inj->pose(1000, 0.0, 0.0, kFixtureYaw, 20.0);
  inj->gps(1000, kFixtureLat, kFixtureLon, 20.0, 0.0);
  bus.step();

  pump(bus, 1'200'000);

  ASSERT_FALSE(col->msgs.empty());
  const auto& m = col->msgs.back();
  EXPECT_TRUE(m.map_loaded());
  ASSERT_TRUE(m.matched()) << "match distance " << m.match_dist_m();
  EXPECT_LT(m.match_dist_m(), 40.0);
  EXPECT_GT(m.route_x_size(), 100) << "2400 m of road ahead at 5 m steps";
  EXPECT_EQ(m.route_x_size(), m.route_kappa_size());
  EXPECT_GT(m.route_length_m(), 500.0);
  EXPECT_EQ(m.road_name(), kFixtureRoadName);
  EXPECT_TRUE(m.has_local_map());
  EXPECT_GT(m.local_edges_size(), 0);
  EXPECT_NEAR(m.lat(), kFixtureLat, 1e-6);
  EXPECT_NEAR(m.lon(), kFixtureLon, 1e-6);

  // The fixture bends right at 1.2 km with a radius of 400 m, so the route must see roughly 1/400 = 0.0025
  // 1/m somewhere ahead. A route that is curvature-blind would pass every assertion above.
  double kappa_max = 0.0;
  for (int i = 0; i < m.route_kappa_size(); ++i)
    kappa_max = std::max(kappa_max, std::abs(static_cast<double>(m.route_kappa(i))));
  EXPECT_GT(kappa_max, 0.0015) << "the arc in the fixture is not showing up in route_kappa";
}

// drive while everything else looks healthy.
//
// What this asserts is the config, not the file: whether the shipped asset is really there is a property of
// the checkout (it is a git-lfs object), and the loader itself is exercised by the MapData tests above
// against their own fixture.
TEST(ShippedConfig, TheMapServiceIsOnAndNamesAnAsset)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  EXPECT_TRUE(cfg.feature_flags.enable_map_data) << "shipped config has map_data off";
  ASSERT_FALSE(cfg.map_data.map_path.empty());
  EXPECT_NE(cfg.map_data.map_path.front(), '/') << "map.path names an APK asset, not a device path";
}

// them: the route is what rides on every message, the surrounding graph only every `local_map_period_s`.
TEST(MapData, LoggedMessageStaysSmallEnoughToRecord)
{
  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = fixtureMap();
  cfg.update_hz = 10.0;
  cfg.local_map_period_s = 1e9;

  bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();

  pump(bus, 1'000'000);
  inj->pose(1000, 0.0, 0.0, kFixtureYaw, 20.0);
  inj->gps(1000, kFixtureLat, kFixtureLon, 20.0, 0.0);
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
  inj2->pose(1000, 0.0, 0.0, kFixtureYaw, 20.0);
  inj2->gps(1000, kFixtureLat, kFixtureLon, 20.0, 0.0);
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
  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = fixtureMap();
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
  adas::middleware::Manager bus(adas::middleware::Manager::Mode::Simulated);
  adas::services::MapData::Config cfg;
  cfg.map_path = fixtureMap();
  cfg.update_hz = 10.0;
  cfg.max_fix_age_s = 5.0;

  bus.registerService<adas::services::MapData>(cfg);
  auto inj = bus.registerService<Injector>();
  auto col = bus.registerService<Collector>();

  pump(bus, 10'000'000);
  inj->pose(10'000, 0.0, 0.0, kFixtureYaw, 20.0);
  inj->gps(10'000, kFixtureLat, kFixtureLon, 20.0, 0.0);
  pump(bus, 10'200'000);
  ASSERT_TRUE(col->msgs.back().matched());

  col->msgs.clear();
  pump(bus, 20'000'000);
  ASSERT_FALSE(col->msgs.empty());
  EXPECT_FALSE(col->msgs.back().matched());
}
