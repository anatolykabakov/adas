#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <vector>

#include <gtest/gtest.h>
#include <json/json.h>

#include "adas/adas_app.h"
#include "adas/lateral/limits.h"
#include "adas/utils/adas_config.h"
#include "adas/utils/lat_control_pid.h"
#include "adas/utils/proto_convert.h"
#include "adas/lateral/angle_control.h"
#include "adas/lateral/pp_planner.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/values.h"

using adas::LatControlPid;
using adas::PidController;
using adas::Vec2;
using volkswagen::applyDriverSteerTorqueLimits;
using volkswagen::CarControllerParams;

TEST(PidController, ProportionalResponse)
{
  PidController pid(0.5, 0.0, 0.0, 50.0);
  EXPECT_NEAR(pid.update(10.0), 1.0, 1e-9);
  pid.reset();
  EXPECT_NEAR(pid.update(1.0), 0.5, 1e-9);
}

TEST(PidController, IntegratorAntiWindupAndOverrideUnwind)
{
  PidController pid(0.0, 10.0, 0.0, 50.0);

  for (int i = 0; i < 100; ++i)
    pid.update(1.0);
  EXPECT_NEAR(pid.control(), 1.0, 1e-9);
  const double i_sat = pid.i();
  EXPECT_GT(i_sat, 0.0);

  for (int i = 0; i < 20; ++i)
    pid.update(1.0, 0.0, true);
  EXPECT_LT(std::abs(pid.i()), std::abs(i_sat));
}

TEST(LatControlPid, InactiveResets)
{
  LatControlPid lat(0.6, 0.0, 0.0, 50.0);
  auto on = lat.update(true, 20.0, 0.0, 5.0, false);
  EXPECT_TRUE(on.active);
  EXPECT_GT(on.steer_norm, 0.0);

  auto off = lat.update(false, 20.0, 0.0, 5.0, false);
  EXPECT_FALSE(off.active);
  EXPECT_NEAR(off.steer_norm, 0.0, 1e-12);
}

TEST(LatControlPid, FeedforwardUsesVSquared)
{
  // Floor explicitly off: checks that the feedforward shape is unchanged, SWA * v squared.
  LatControlPid lat(0.0, 0.0, 0.001, 50.0, 0.0);

  auto r = lat.update(true, 10.0, 10.0, 10.0, false);
  EXPECT_NEAR(r.angle_error_deg, 0.0, 1e-12);
  EXPECT_NEAR(r.steer_norm, 1.0, 1e-9);
  EXPECT_NEAR(r.f, 1.0, 1e-9);
}

TEST(LatControlPid, FeedforwardFloorAddsSpeedIndependentDemand)
{
  const double floor_mps = 10.0;
  LatControlPid lat(0.0, 0.0, 0.001, 50.0, floor_mps);

  auto slow = lat.update(true, 10.0, 10.0, 0.0, false);
  EXPECT_NEAR(slow.f, 0.001 * 10.0 * floor_mps * floor_mps, 1e-9);
  EXPECT_GT(slow.f, 0.0);

  lat.reset();
  auto at_floor = lat.update(true, 1.0, 1.0, floor_mps, false);
  EXPECT_NEAR(at_floor.f, 2.0 * 0.001 * 1.0 * floor_mps * floor_mps, 1e-9);

  lat.reset();
  auto fast = lat.update(true, 1.0, 1.0, 25.0, false);
  EXPECT_NEAR(fast.f, 0.001 * 1.0 * (625.0 + 100.0), 1e-9);
  EXPECT_LT((fast.f - 0.001 * 625.0) / fast.f, 0.20);
}

TEST(LatControlPid, FeedforwardMatchesMeasuredDemandAcrossSpeeds)
{
  struct Point {
    double v_mps;
    double measured_norm_per_deg;
  };
  const Point points[] = {{6.0, 0.0185}, {10.0, 0.0265}, {15.0, 0.0465}, {21.0, 0.0765}};

  LatControlPid lat(0.0, 0.0, 0.00015, 50.0, adas::kFeedforwardFloorMps);
  for (const auto& pt : points) {
    lat.reset();
    auto r = lat.update(true, 1.0, 1.0, pt.v_mps, false);
    const double rel = std::abs(r.f - pt.measured_norm_per_deg) / pt.measured_norm_per_deg;
    EXPECT_LT(rel, 0.35) << "v = " << pt.v_mps << " m/s: feedforward " << r.f << ", measured "
                         << pt.measured_norm_per_deg;
  }
}

TEST(AdasConfigLoad, CameraYawRateSourceIsOffAndInvertible)
{
  const std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/adas_cam_yaw_test."
                                                                                                 "json";
  {
    std::ofstream f(path);
    f << R"({"localization": {"use_camera_odometry": true}})";
  }
  bool ok = false;
  auto cfg = AdasApp::Config::loadFromFile(path, &ok);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(cfg.localization.sources.camera_odometry);
  std::remove(path.c_str());

  {
    std::ofstream f(path);
    f << R"({"localization": {"use_camera_odometry": false}})";
  }
  cfg = AdasApp::Config::loadFromFile(path, &ok);
  EXPECT_TRUE(ok);
  EXPECT_FALSE(cfg.localization.sources.camera_odometry);
  std::remove(path.c_str());
}

TEST(AdasConfigLoad, FeedforwardFloorComesFromConfig)
{
  const std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/adas_ff_floor_test."
                                                                                                 "json";
  {
    std::ofstream f(path);
    f << R"({"vehicle": {"lat_pid_kf": 0.00021, "lat_pid_ff_floor_mps": 7.5}})";
  }
  bool ok = false;
  const auto cfg = AdasApp::Config::loadFromFile(path, &ok);
  EXPECT_TRUE(ok);
  EXPECT_NEAR(cfg.lane_keep.pid_kf, 0.00021, 1e-12);
  EXPECT_NEAR(cfg.lane_keep.pid_ff_floor_mps, 7.5, 1e-12);
  std::remove(path.c_str());
}

TEST(AdasConfigLoad, FeedforwardDefaultsMatchUpstream)
{
  AdasApp::Config cfg;
  EXPECT_NEAR(cfg.lane_keep.pid_kf, 6e-5, 1e-12);
  EXPECT_NEAR(cfg.lane_keep.pid_ff_floor_mps, 0.0, 1e-12);
}

TEST(LatControlPid, TheFittedGainAsksSeveralTimesMoreThanUpstream)
{
  LatControlPid upstream(0.0, 0.0, 6e-5, 50.0, 0.0);
  LatControlPid fitted(0.0, 0.0, 0.00015, 50.0, adas::kFeedforwardFloorMps);

  for (double v : {6.0, 10.0, 15.0, 21.0}) {
    upstream.reset();
    fitted.reset();
    const double up = upstream.update(true, 1.0, 1.0, v, false).f;
    const double fit = fitted.update(true, 1.0, 1.0, v, false).f;
    EXPECT_GT(fit, 2.0 * up) << "v = " << v;
  }
}

namespace {
// The same settings PurePursuit(0.4, 2.636, 1.4, 3.0, 20.0) had.
adas::lateral::PpPlanner::Config ppConfig()
{
  adas::lateral::PpPlanner::Config c;
  c.k_dd = 0.4;
  c.waypoint_shift = 1.4;
  c.ld_min = 3.0;
  c.ld_max = 20.0;
  c.vehicle.wheelbase_m = 2.636;
  return c;
}

adas::lateral::Input straightAt(double y, double speed_mps)
{
  adas::lateral::Input in;
  in.speed_mps = speed_mps;
  for (int i = 0; i <= 20; ++i)
    in.polyline_ego.push_back({static_cast<double>(i), y});
  return in;
}

}  // namespace

TEST(PpPlanner, StraightCenterlineNearZeroSteer)
{
  adas::lateral::PpPlanner planner(ppConfig());
  const auto r = planner.update(straightAt(0.0, 10.0));
  ASSERT_TRUE(r.has_target);
  EXPECT_NEAR(r.steer_rad, 0.0, 1e-6);
  EXPECT_NEAR(r.lookahead_m, 4.0, 1e-9);
}

TEST(PpPlanner, LeftOffsetProducesPositiveSteer)
{
  adas::lateral::PpPlanner planner(ppConfig());
  const auto r = planner.update(straightAt(1.5, 10.0));
  ASSERT_TRUE(r.has_target);
  EXPECT_GT(r.steer_rad, 0.0);
}

TEST(PpPlanner, EmptyPolylineNoTarget)
{
  adas::lateral::PpPlanner planner(adas::lateral::PpPlanner::Config{});
  adas::lateral::Input in;
  in.speed_mps = 5.0;
  const auto r = planner.update(in);
  EXPECT_FALSE(r.has_target);
  EXPECT_NEAR(r.steer_rad, 0.0, 1e-12);
  EXPECT_EQ("no_polyline", r.status);
}

TEST(SteerTorqueLimits, FirstStepFromZeroMatchesPandaRateUp)
{
  EXPECT_EQ(applyDriverSteerTorqueLimits(300, 0.f, 0), CarControllerParams::STEER_DELTA_UP);
  EXPECT_EQ(applyDriverSteerTorqueLimits(-300, 0.f, 0), -CarControllerParams::STEER_DELTA_UP);
}

TEST(SteerTorqueLimits, RateUpWhilePositive)
{
  const int last = 40;
  EXPECT_EQ(applyDriverSteerTorqueLimits(300, 0.f, last), last + CarControllerParams::STEER_DELTA_UP);
  EXPECT_EQ(applyDriverSteerTorqueLimits(0, 0.f, last), last - CarControllerParams::STEER_DELTA_DOWN);
}

TEST(SteerTorqueLimits, AbsoluteCapAtSteerMax)
{
  EXPECT_EQ(applyDriverSteerTorqueLimits(300, 0.f, 296), 300);
  EXPECT_EQ(applyDriverSteerTorqueLimits(400, 0.f, 300), 300);
}

TEST(SteerTorqueLimits, ConstantsMatchPandaSafetyContract)
{
  EXPECT_EQ(CarControllerParams::STEER_DELTA_UP, 4);
  EXPECT_EQ(CarControllerParams::STEER_DELTA_DOWN, 10);
  EXPECT_EQ(CarControllerParams::STEER_MAX, 300);
  EXPECT_EQ(CarControllerParams::STEER_DRIVER_ALLOWANCE, 80);
}

namespace {
adas::LanePathConfig pathCfg(double blend, double camera_offset_m)
{
  adas::LanePathConfig cfg;
  cfg.lane_blend_scale = blend;
  cfg.camera_offset_m = camera_offset_m;
  return cfg;
}

}  // namespace

TEST(TopicConvert, CameraOffsetShiftsPathRight)
{
  adas::proto::LaneLines ll;
  for (int i = 0; i < 33; ++i) {
    const double x = i * 3.0;
    ll.add_x(x);
    ll.add_plan_x(x);
    ll.add_plan_y(0.0);
  }
  for (int lane = 0; lane < 4; ++lane) {
    auto* l = ll.add_lanes();
    l->set_prob(lane == 1 || lane == 2 ? 0.99f : 0.1f);
    for (int i = 0; i < 33; ++i)
      l->add_y(lane == 1 ? -1.6 : 1.6);
  }

  const auto plain = adas::laneLinesToPath(ll, pathCfg(0.0, 0.0));
  const auto shifted = adas::laneLinesToPath(ll, pathCfg(0.0, 0.08));
  ASSERT_GE(plain.polyline.size(), 2u);
  ASSERT_EQ(plain.polyline.size(), shifted.polyline.size());
  ASSERT_EQ(plain.plan_poly.size(), shifted.plan_poly.size());

  for (size_t i = 0; i < plain.polyline.size(); ++i) {
    EXPECT_NEAR(shifted.polyline[i].y() - plain.polyline[i].y(), 0.08, 1e-9);
    EXPECT_NEAR(shifted.polyline[i].x(), plain.polyline[i].x(), 1e-9);
  }
  for (size_t i = 0; i < plain.plan_poly.size(); ++i)
    EXPECT_NEAR(shifted.plan_poly[i].y() - plain.plan_poly[i].y(), 0.08, 1e-9);

  const auto fused = adas::laneLinesToPath(ll, pathCfg(1.0, 0.08));
  const auto fused_plain = adas::laneLinesToPath(ll, pathCfg(1.0, 0.0));
  ASSERT_EQ(fused.polyline.size(), fused_plain.polyline.size());
  ASSERT_GE(fused.polyline.size(), 2u);
  EXPECT_NEAR(fused.polyline[0].y() - fused_plain.polyline[0].y(), 0.08, 1e-9);
}

TEST(TopicConvert, LaneBlendRequiresBothHostLines)
{
  auto make = [](bool left_ok, bool right_ok, double width) {
    adas::proto::LaneLines ll;
    for (int i = 0; i < 33; ++i) {
      const double x = i * 3.0;
      ll.add_x(x);
      ll.add_plan_x(x);
      ll.add_plan_y(1.0);
    }
    for (int lane = 0; lane < 4; ++lane) {
      auto* l = ll.add_lanes();
      const bool near_left = lane == 1, near_right = lane == 2;
      float prob = 0.05f;
      if (near_left)
        prob = left_ok ? 0.99f : 0.05f;
      if (near_right)
        prob = right_ok ? 0.99f : 0.05f;
      l->set_prob(prob);
      for (int i = 0; i < 33; ++i)
        l->add_y(near_left ? -width / 2 : (near_right ? width / 2 : (lane == 0 ? -width : width)));
    }
    return ll;
  };
  const double blend = 1.0, off = 0.0;
  const double plan_y = 1.0;

  const auto both = adas::laneLinesToPath(make(true, true, 3.2), pathCfg(blend, off));
  ASSERT_GE(both.polyline.size(), 2u);
  EXPECT_LT(both.polyline[1].y(), plan_y - 0.3);

  for (auto ll : {make(true, false, 3.2), make(false, true, 3.2)}) {
    const auto one = adas::laneLinesToPath(ll, pathCfg(blend, off));
    ASSERT_GE(one.polyline.size(), 2u);
    EXPECT_NEAR(one.polyline[1].y(), plan_y, 1e-9);
  }

  for (double w : {2.0, 5.0}) {
    const auto bad = adas::laneLinesToPath(make(true, true, w), pathCfg(blend, off));
    ASSERT_GE(bad.polyline.size(), 2u);
    EXPECT_NEAR(bad.polyline[1].y(), plan_y, 1e-9);
  }
}

namespace {
adas::proto::LaneLines twoLineFrame(float sigma_m, float lane_width_m = 3.5f)
{
  adas::proto::LaneLines ll;
  auto* left = ll.add_lanes();
  (void)left;
  auto* l = ll.add_lanes();
  auto* r = ll.add_lanes();
  l->set_prob(0.9f);
  r->set_prob(0.9f);
  for (int i = 0; i < 33; ++i) {
    const float x = 1.0f * i * 1.5f;
    ll.add_x(x);
    ll.add_plan_x(x);
    ll.add_plan_y(0.4f);
    l->add_y(-0.5f * lane_width_m);
    r->add_y(+0.5f * lane_width_m);
    if (sigma_m > 0.0f) {
      l->add_y_std(sigma_m);
      r->add_y_std(sigma_m);
    }
  }
  return ll;
}

double pathYAt10m(const adas::LanePathMsg& msg)
{
  for (const auto& p : msg.polyline)
    if (p.x() >= 10.0)
      return p.y();
  return msg.polyline.empty() ? 0.0 : msg.polyline.back().y();
}

}  // namespace

TEST(LaneBlend, ConfidentLinesPullThePathTowardTheLaneCentre)
{
  const auto path = adas::laneLinesToPath(twoLineFrame(0.15f), pathCfg(1.0, 0.0));
  EXPECT_TRUE(path.lane_anchored);
  EXPECT_NEAR(0.0, pathYAt10m(path), 0.05);
}

TEST(LaneBlend, UnsureLinesAreIgnored)
{
  const auto path = adas::laneLinesToPath(twoLineFrame(1.6f), pathCfg(1.0, 0.0));
  EXPECT_FALSE(path.lane_anchored);
  EXPECT_NEAR(0.4, pathYAt10m(path), 0.05);
}

TEST(LaneBlend, MiddlingSigmaBlendsPartially)
{
  const auto path = adas::laneLinesToPath(twoLineFrame(1.0f), pathCfg(1.0, 0.0));
  const double y = pathYAt10m(path);
  EXPECT_GT(y, 0.05);
  EXPECT_LT(y, 0.35);
}

TEST(LaneBlend, ArcGradeSigmaStillAnchors)
{
  for (float sigma : {0.6f, 0.95f}) {
    const auto path = adas::laneLinesToPath(twoLineFrame(sigma), pathCfg(1.0, 0.0));
    EXPECT_TRUE(path.lane_anchored) << "sigma " << sigma;

    EXPECT_LT(pathYAt10m(path), 0.3) << "sigma " << sigma;
  }
}

TEST(LaneBlend, FramesWithoutSigmasAreNotPenalised)
{
  const auto path = adas::laneLinesToPath(twoLineFrame(0.0f), pathCfg(1.0, 0.0));
  EXPECT_TRUE(path.lane_anchored);
  EXPECT_NEAR(0.0, pathYAt10m(path), 0.05);
}

namespace {
adas::proto::LaneLines offsetLaneFrame(double centre_y, double kappa = 0.0, double width = 3.3)
{
  adas::proto::LaneLines ll;
  ll.add_lanes();
  auto* l = ll.add_lanes();
  auto* r = ll.add_lanes();
  l->set_prob(0.95f);
  r->set_prob(0.95f);
  for (int i = 0; i < 33; ++i) {
    const double x = i * 1.5;
    const double c = centre_y + 0.5 * kappa * x * x;
    ll.add_x(x);
    ll.add_plan_x(x);
    ll.add_plan_y(c);
    l->add_y(c - 0.5 * width);
    r->add_y(c + 0.5 * width);
  }
  return ll;
}

adas::LanePathConfig centringCfg(double gain = 0.7)
{
  adas::LanePathConfig cfg;
  cfg.lane_blend_scale = 1.0;
  cfg.center_force_gain = gain;
  return cfg;
}

}  // namespace

TEST(CenterForce, ShiftsTheReferenceTowardTheLaneCentre)
{
  const auto off = adas::laneLinesToPath(offsetLaneFrame(0.5), centringCfg(0.0));
  const auto on = adas::laneLinesToPath(offsetLaneFrame(0.5), centringCfg(0.7));
  ASSERT_TRUE(on.lane_anchored);
  EXPECT_NEAR(0.5, on.lane_offset_m, 0.02);
  EXPECT_GT(on.center_force_m, 0.0);
  EXPECT_NEAR(0.0, off.center_force_m, 1e-12);

  EXPECT_NEAR(0.36, on.center_force_m, 0.03);
  for (size_t i = 0; i < on.polyline.size(); ++i)
    EXPECT_NEAR(on.polyline[i].y() - off.polyline[i].y(), on.center_force_m, 1e-9);
}

TEST(CenterForce, MirrorsForTheOtherSideAndVanishesWhenCentred)
{
  const auto right = adas::laneLinesToPath(offsetLaneFrame(-0.5), centringCfg());
  EXPECT_LT(right.center_force_m, 0.0);
  const auto centred = adas::laneLinesToPath(offsetLaneFrame(0.0), centringCfg());
  EXPECT_NEAR(0.0, centred.center_force_m, 0.01);
}

TEST(CenterForce, IsClampedLikeUpstream)
{
  adas::LanePathConfig cfg = centringCfg(0.7);
  cfg.center_force_max_m = 0.8;
  const auto path = adas::laneLinesToPath(offsetLaneFrame(3.0), cfg);
  EXPECT_NEAR(0.8, path.center_force_m, 1e-9);
}

TEST(CenterForce, DampedWhenItPushesIntoTheTurn)
{
  const auto same = adas::laneLinesToPath(offsetLaneFrame(0.5, +0.005), centringCfg());
  const auto opposite = adas::laneLinesToPath(offsetLaneFrame(0.5, -0.005), centringCfg());
  ASSERT_GT(opposite.center_force_m, 0.0);
  EXPECT_NEAR(0.7, same.center_force_m / opposite.center_force_m, 0.05);
}

TEST(CenterForce, OffWithoutPaintToMeasureAgainst)
{
  auto ll = offsetLaneFrame(0.5);
  ll.mutable_lanes(2)->set_prob(0.05f);
  const auto path = adas::laneLinesToPath(ll, centringCfg());
  EXPECT_FALSE(path.lane_anchored);
  EXPECT_NEAR(0.0, path.center_force_m, 1e-12);
}

TEST(CenterForce, ReadsTheOffsetAtTheCarNotAhead)
{
  const double kappa = 0.006;
  const auto path = adas::laneLinesToPath(offsetLaneFrame(0.0, kappa), centringCfg());
  ASSERT_TRUE(path.lane_anchored);

  EXPECT_NEAR(0.0, path.lane_offset_m, 0.02);
  EXPECT_NEAR(0.0, path.center_force_m, 0.02);
}

TEST(LaneWidthFilter, SingleWideFrameDoesNotDropTheAnchor)
{
  adas::LanePathConfig cfg = pathCfg(1.0, 0.0);
  adas::LaneFusionState state;
  for (int i = 0; i < 20; ++i) {
    const auto path = adas::laneLinesToPath(offsetLaneFrame(0.0, 0.0, 3.9), cfg, &state);
    ASSERT_TRUE(path.lane_anchored) << "settling frame " << i;
  }
  const auto spike = adas::laneLinesToPath(offsetLaneFrame(0.0, 0.0, 6.0), cfg, &state);
  EXPECT_TRUE(spike.lane_anchored);
  EXPECT_NEAR(3.9, spike.lane_width_m, 0.05);

  const auto unfiltered = adas::laneLinesToPath(offsetLaneFrame(0.0, 0.0, 6.0), cfg);
  EXPECT_FALSE(unfiltered.lane_anchored);
}

TEST(LaneWidthFilter, FollowsARealWidthChange)
{
  adas::LanePathConfig cfg = pathCfg(1.0, 0.0);
  adas::LaneFusionState state;
  adas::laneLinesToPath(offsetLaneFrame(0.0, 0.0, 3.0), cfg, &state);
  EXPECT_NEAR(3.0, state.lane_width_m, 1e-6);
  for (int i = 0; i < 400; ++i)
    adas::laneLinesToPath(offsetLaneFrame(0.0, 0.0, 3.6), cfg, &state);
  EXPECT_NEAR(3.6, state.lane_width_m, 0.05);
}

TEST(AdasConfigFile, ShippedJsonReachesTheLanePathKnobs)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  Json::Value root;
  std::ifstream in(path);
  ASSERT_TRUE(in.good());
  ASSERT_TRUE(Json::parseFromStream(Json::CharReaderBuilder{}, in, &root, nullptr));
  const Json::Value& veh = root["vehicle"];
  ASSERT_TRUE(veh.isObject());

  EXPECT_DOUBLE_EQ(veh["path_lane_blend_scale"].asDouble(), cfg.lane_keep.lane_path.lane_blend_scale);
  EXPECT_DOUBLE_EQ(veh["path_camera_offset_m"].asDouble(), cfg.lane_keep.lane_path.camera_offset_m);
  EXPECT_DOUBLE_EQ(veh["lane_std_good_m"].asDouble(), cfg.lane_keep.lane_path.lane_std_good_m);
  EXPECT_DOUBLE_EQ(veh["lane_std_bad_m"].asDouble(), cfg.lane_keep.lane_path.lane_std_bad_m);
  EXPECT_DOUBLE_EQ(veh["lane_width_min_m"].asDouble(), cfg.lane_keep.lane_path.lane_width_min_m);
  EXPECT_DOUBLE_EQ(veh["lane_width_max_m"].asDouble(), cfg.lane_keep.lane_path.lane_width_max_m);
  EXPECT_DOUBLE_EQ(veh["center_force_gain"].asDouble(), cfg.lane_keep.lane_path.center_force_gain);
  EXPECT_DOUBLE_EQ(veh["center_force_max_m"].asDouble(), cfg.lane_keep.lane_path.center_force_max_m);
  EXPECT_DOUBLE_EQ(veh["center_force_turn_scale"].asDouble(), cfg.lane_keep.lane_path.center_force_turn_scale);

  EXPECT_DOUBLE_EQ(root["calibration"]["camera"]["position_m"]["y_left"].asDouble(),
                   cfg.lane_keep.lane_path.cam_y_left_m);
}

TEST(LaneBlendRuntimeKnob, TakesEffectAndIsClamped)
{
  adas::LanePathConfig cfg;
  cfg.lane_blend_scale = 0.6;

  std::map<std::string, std::function<void(double)>> setters;
  adas::registerLanePathParameters(cfg, [&setters](const char* name, auto setter, auto) { setters[name] = setter; });
  const auto set = [&setters](const std::string& name, double v) { setters.at(name)(v); };

  set("path_lane_blend_scale", 1.0);
  EXPECT_DOUBLE_EQ(cfg.lane_blend_scale, 1.0);
  set("path_lane_blend_scale", 1.4);
  EXPECT_DOUBLE_EQ(cfg.lane_blend_scale, 1.0) << "above 1.0 is meaningless";
  set("path_lane_blend_scale", -0.2);
  EXPECT_DOUBLE_EQ(cfg.lane_blend_scale, 0.0) << "negative weight would invert the path";

  set("center_force_gain", -1.0);
  EXPECT_DOUBLE_EQ(cfg.center_force_gain, 0.0) << "negative centring would push away from the centre";
}

namespace {
constexpr double kStdRangePlanY = 1.0;

/** Straight lane centred on 0, plan offset by `kStdRangePlanY`, σ growing with range. */
adas::proto::LaneLines laneFrameWithStd(double std_near, double std_far, double split_x = 20.0)
{
  adas::proto::LaneLines ll;
  for (int lane = 0; lane < 4; ++lane) {
    auto* l = ll.add_lanes();
    l->set_prob(lane == 1 || lane == 2 ? 0.99f : 0.05f);
  }
  for (int i = 0; i < 33; ++i) {
    const double x = i * 1.5;
    ll.add_x(x);
    ll.add_plan_x(x);
    ll.add_plan_y(kStdRangePlanY);
    const double sigma = x <= split_x ? std_near : std_far;
    for (int lane = 0; lane < 4; ++lane) {
      auto* l = ll.mutable_lanes(lane);
      const double half = 1.65;
      l->add_y(lane == 1 ? -half : (lane == 2 ? half : (lane == 0 ? -3.3 : 3.3)));
      l->add_y_std(sigma);
    }
  }
  return ll;
}

adas::LanePathConfig stdRangeCfg(double range_m)
{
  adas::LanePathConfig cfg = pathCfg(1.0, 0.0);
  cfg.lane_std_good_m = 0.3;
  cfg.lane_std_bad_m = 1.5;
  cfg.lane_std_range_m = range_m;
  return cfg;
}

double referenceY(const adas::proto::LaneLines& ll, const adas::LanePathConfig& cfg)
{
  const auto out = adas::laneLinesToPath(ll, cfg);
  return out.polyline.size() >= 2 ? out.polyline[1].y() : std::nan("");
}

}  // namespace

TEST(LaneStdRange, NearWindowKeepsALineWhoseFarHalfIsUnobserved)
{
  const auto ll = laneFrameWithStd(0.3, 2.5);

  const double near_y = referenceY(ll, stdRangeCfg(20.0));
  const double far_y = referenceY(ll, stdRangeCfg(40.0));

  EXPECT_LT(near_y, 0.5 * kStdRangePlanY) << "near window should trust the observed half";
  EXPECT_NEAR(far_y, kStdRangePlanY, 0.05) << "old window let unobserved range veto the line";
}

TEST(LaneStdRange, ABadNearHalfIsStillRejected)
{
  const auto ll = laneFrameWithStd(2.0, 2.0);
  EXPECT_NEAR(referenceY(ll, stdRangeCfg(20.0)), kStdRangePlanY, 0.05);
}

TEST(LaneStdRange, AGoodLineIsUnaffectedByTheWindow)
{
  const auto ll = laneFrameWithStd(0.15, 0.2);
  EXPECT_NEAR(referenceY(ll, stdRangeCfg(20.0)), referenceY(ll, stdRangeCfg(40.0)), 1e-9);
}

TEST(ShippedConfig, LaneStdRangeIsTheNearField)
{
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(ADAS_SHIPPED_CONFIG_JSON, &ok);
  ASSERT_TRUE(ok);
  EXPECT_GT(cfg.lane_keep.lane_path.lane_std_range_m, 5.0);
  EXPECT_LE(cfg.lane_keep.lane_path.lane_std_range_m, 25.0) << "past ~25 m sigma is dominated by extrapolation, see "
                                                               "the "
                                                               "measured table";
}

namespace {
adas::proto::LaneLines frameWithProbs(float lp, float rp)
{
  adas::proto::LaneLines ll = twoLineFrame(0.1f);
  ll.mutable_lanes(1)->set_prob(lp);
  ll.mutable_lanes(2)->set_prob(rp);
  return ll;
}

}  // namespace

TEST(DpParity, LanelessHysteresisFollowsUpstreamThresholds)
{
  adas::LanePathConfig cfg;
  adas::LaneFusionState state;
  auto run = [&](float lp, float rp) {
    auto ll = frameWithProbs(lp, rp);
    return adas::laneLinesToPath(ll, cfg, &state).lanelines_active;
  };

  EXPECT_TRUE(run(0.9f, 0.9f)) << "both confident — lane mode";
  EXPECT_TRUE(run(0.35f, 0.35f)) << "between 0.3 and 0.5 the hysteresis keeps the previous mode";
  EXPECT_FALSE(run(0.2f, 0.2f)) << "both below 0.3 — fall back to the model plan alone";
  EXPECT_FALSE(run(0.45f, 0.45f)) << "coming back takes more than being above 0.3";
  EXPECT_TRUE(run(0.55f, 0.1f)) << "one above 0.5 is enough: theirs is an OR, not an AND";
}

TEST(DpParity, LanelessModeStopsTheBlendEntirely)
{
  adas::LanePathConfig cfg;
  cfg.camera_offset_m = 0.0;
  adas::LaneFusionState state;

  auto confident = frameWithProbs(0.9f, 0.9f);
  const double with_lines = pathYAt10m(adas::laneLinesToPath(confident, cfg, &state));
  auto weak = frameWithProbs(0.1f, 0.1f);
  const double laneless = pathYAt10m(adas::laneLinesToPath(weak, cfg, &state));

  EXPECT_NEAR(laneless, 0.4, 1e-6) << "laneless is exactly the model plan, with no trace of the lines";
  EXPECT_NE(with_lines, laneless);
}
TEST(DpParity, RollCompensationMatchesUpstreamFormula)
{
  const double sf = 0.0015, v = 20.0, roll = 0.05;
  EXPECT_NEAR(adas::rollCompensationCurvature(roll, v, sf), 9.81 * roll / (1.0 / sf - v * v), 1e-12);
  EXPECT_DOUBLE_EQ(adas::rollCompensationCurvature(roll, v, 0.0), 0.0) << "with no slip there is nothing to subtract";
  EXPECT_GT(adas::rollCompensationCurvature(0.05, v, sf), 0.0) << "a bank to the right pulls to the right";
  EXPECT_LT(adas::rollCompensationCurvature(-0.05, v, sf), 0.0);
}

// ---------------------------------------------------------------- lateral::AngleControl
//
// The angle loop without middleware — what moved into the Control service. Tested here because
// bringing an application up for angle arithmetic buys nothing, same as longplan::compute and
// safety::SafetyPlanner.

namespace {

adas::lateral::AngleControl::Config ctlConfig()
{
  adas::lateral::AngleControl::Config c;
  c.pid_kp = 0.6;
  c.pid_ki = 0.2;
  c.pid_kf = 6e-5;
  c.steer_ratio = 15.6;
  c.steer_sign = -1.0;
  c.max_steer_deg = 20.0;
  c.tire_stiffness_factor = 1.0;
  return c;
}

adas::ChassisSample chassisAt(double v, double swa_deg, bool pressed = false)
{
  adas::ChassisSample ch;
  ch.speed_mps = v;
  ch.steering_angle_deg = swa_deg;
  ch.steering_pressed = pressed;
  return ch;
}

}  // namespace

TEST(AngleControl, RoadWheelAngleBecomesSteeringWheelAngleWithSignAndRatio)
{
  adas::lateral::AngleControl ctl(ctlConfig());
  ctl.setSetpointFromSteer(0.01);
  // The CAN sign is negative, so a positive road-wheel angle yields a negative steering-wheel angle.
  EXPECT_NEAR(ctl.desiredSwaDeg(), -0.01 * 180.0 / M_PI * 15.6, 1e-9);
}

TEST(AngleControl, LearnedOffsetIsAddedNotSubtracted)
{
  auto cfg = ctlConfig();
  adas::lateral::AngleControl ctl(cfg);
  ctl.setLearnedParams(true, 1.0, 15.6, 0.8);
  ctl.setSetpointFromSteer(0.0);
  // The estimator solved delta = (SWA - offset) / ratio; the inverse relation adds the wheel zero back.
  EXPECT_NEAR(ctl.desiredSwaDeg(), 0.8, 1e-9);
}

TEST(AngleControl, LosingValidityWalksParametersBackToConfigured)
{
  auto cfg = ctlConfig();
  adas::lateral::AngleControl ctl(cfg);
  ctl.setLearnedParams(true, 1.4, 17.0, 0.5);
  EXPECT_TRUE(ctl.usingLearnedParams());
  EXPECT_NEAR(ctl.effectiveSteerRatio(), 17.0, 1e-9);

  ctl.setLearnedParams(false, 1.4, 17.0, 0.5);
  EXPECT_FALSE(ctl.usingLearnedParams());
  EXPECT_NEAR(ctl.effectiveSteerRatio(), 15.6, 1e-9) << "losing validity rolls back rather than freezing";
  EXPECT_NEAR(ctl.effectiveAngleOffsetDeg(), 0.0, 1e-9);
}

TEST(AngleControl, ConfiguredRatioSurvivesLearnedOne)
{
  auto cfg = ctlConfig();
  adas::lateral::AngleControl ctl(cfg);
  ctl.setLearnedParams(true, 1.0, 17.0, 0.0);
  // A measured steering-wheel angle is converted with the configured ratio: substituting the learned
  // one would quietly change what counts as the input.
  EXPECT_NEAR(ctl.steerRatio(), 15.6, 1e-9);
  EXPECT_NEAR(ctl.effectiveSteerRatio(), 17.0, 1e-9);
}

TEST(AngleControl, ClearSetpointZeroesTheCommandAndTheIntegrator)
{
  adas::lateral::AngleControl ctl(ctlConfig());
  // 0.001 rad of road-wheel angle is about 0.9 deg at the wheel: an error inside the linear region,
  // otherwise anti-windup legitimately stops the integrator and there would be nothing to check.
  ctl.setSetpointFromSteer(0.001);
  for (int i = 0; i < 50; ++i)
    ctl.update(true, chassisAt(20.0, 0.0));
  ASSERT_GT(std::abs(ctl.update(true, chassisAt(20.0, 0.0)).i), 1e-6) << "the integrator should have accumulated";

  ctl.clearSetpoint();
  EXPECT_NEAR(ctl.desiredSwaDeg(), 0.0, 1e-12);
  EXPECT_NEAR(ctl.update(false, chassisAt(20.0, 0.0)).i, 0.0, 1e-12);
}

TEST(AngleControl, IntegratorRateDoesNotDependOnCallCadence)
{
  // The parity property: upstream's PIDController(rate=100) is fixed at construction. So the same
  // number of calls yields the same integral, however often they are made.
  auto run = [](int n) {
    adas::lateral::AngleControl ctl(ctlConfig());
    ctl.setSetpointFromSteer(0.001);
    double last = 0.0;
    for (int i = 0; i < n; ++i)
      last = ctl.update(true, chassisAt(20.0, 0.0)).i;
    return last;
  };
  EXPECT_NEAR(run(10), run(10), 1e-12);
  EXPECT_GT(std::abs(run(20)), std::abs(run(10))) << "more calls, more integral";
}

TEST(AngleControl, SlewGuardLimitsTheStepAndRemembersTheLast)
{
  adas::lateral::AngleControl ctl(ctlConfig());
  ctl.setSlewConfig({/*limit_deg=*/1.0, /*max_lateral_jerk=*/1e9, /*rate_min_speed=*/2.0,
                     /*Lf=*/2.67, /*max_gap_frames=*/10});
  double steer = 0.0;
  EXPECT_FALSE(ctl.applySlew(steer, 20.0, 0.05, true)) << "the first frame has nothing to compare against";

  steer = 1.0;  // rad, far beyond the 1-degree ceiling
  EXPECT_TRUE(ctl.applySlew(steer, 20.0, 0.05, true));
  EXPECT_NEAR(steer, 1.0 * M_PI / 180.0, 1e-9);
}

TEST(AngleControl, DriverOverrideUnwindsTheIntegratorInsteadOfGrowingIt)
{
  adas::lateral::AngleControl ctl(ctlConfig());
  ctl.setSetpointFromSteer(0.001);
  for (int i = 0; i < 100; ++i)
    ctl.update(true, chassisAt(20.0, 0.0));
  const double wound = std::abs(ctl.update(true, chassisAt(20.0, 0.0)).i);

  for (int i = 0; i < 20; ++i)
    ctl.update(true, chassisAt(20.0, 0.0, /*pressed=*/true));
  EXPECT_LT(std::abs(ctl.update(true, chassisAt(20.0, 0.0, true)).i), wound);
}

/** The kinematic mode is what a simulated ego needs: MetaDrive's car turns as geometry says, and the
 *  Golf understeer model would over-turn every arc. The switch was lost once in a refactor and the
 *  Python bench died on a missing binding, so both ends are pinned here. */
TEST(VehicleModel, ClearingTheFlagLeavesTheKinematicModel)
{
  adas::lateral::VehicleParams v;
  v.wheelbase_m = 2.636;
  v.tire_stiffness_factor = 0.64;

  v.use_vehicle_model = true;
  EXPECT_NE(adas::lateral::slipFactorOrZero(v), 0.0) << "the slip model must contribute something";

  v.use_vehicle_model = false;
  EXPECT_EQ(adas::lateral::slipFactorOrZero(v), 0.0) << "no slip term means the kinematic bicycle model";
}
