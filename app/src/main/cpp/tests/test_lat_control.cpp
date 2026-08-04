#include <cmath>
#include <cstdlib>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <json/json.h>

#include "adas_app.h"
#include "utils/lat_control_pid.h"
#include "services/topic_convert_service.h"
#include "utils/topic_convert.h"
#include "utils/pure_pursuit.h"
#include "volkswagen/carcontroller.h"
#include "volkswagen/values.h"

using adas::LatControlPid;
using adas::PidController;
using adas::PurePursuit;
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
  LatControlPid lat(0.0, 0.0, 0.001, 50.0);

  auto r = lat.update(true, 10.0, 10.0, 10.0, false);
  EXPECT_NEAR(r.angle_error_deg, 0.0, 1e-12);
  EXPECT_NEAR(r.steer_norm, 1.0, 1e-9);
  EXPECT_NEAR(r.f, 1.0, 1e-9);
}

TEST(PurePursuit, StraightCenterlineNearZeroSteer)
{
  PurePursuit pp(0.4, 2.636, 1.4, 3.0, 20.0);
  std::vector<Vec2> poly;
  for (int i = 0; i <= 20; ++i)
    poly.push_back({static_cast<double>(i), 0.0});
  const auto r = pp.compute(poly, 10.0);
  ASSERT_TRUE(r.target_ego.has_value());
  EXPECT_NEAR(r.steer_rad, 0.0, 1e-6);
  EXPECT_NEAR(r.lookahead_m, 4.0, 1e-9);
}

TEST(PurePursuit, LeftOffsetProducesPositiveSteer)
{
  PurePursuit pp(0.4, 2.636, 1.4, 3.0, 20.0);
  std::vector<Vec2> poly;
  for (int i = 0; i <= 20; ++i)
    poly.push_back({static_cast<double>(i), 1.5});
  const auto r = pp.compute(poly, 10.0);
  ASSERT_TRUE(r.target_ego.has_value());
  EXPECT_GT(r.steer_rad, 0.0);
}

TEST(PurePursuit, EmptyPolylineNoTarget)
{
  PurePursuit pp;
  const auto r = pp.compute({}, 5.0);
  EXPECT_FALSE(r.target_ego.has_value());
  EXPECT_NEAR(r.steer_rad, 0.0, 1e-12);
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
  ai::flow::adas::LaneLines ll;
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
    ai::flow::adas::LaneLines ll;
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

ai::flow::adas::LaneLines twoLineFrame(float sigma_m, float lane_width_m = 3.5f)
{
  ai::flow::adas::LaneLines ll;
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

ai::flow::adas::LaneLines offsetLaneFrame(double centre_y, double kappa = 0.0, double width = 3.3)
{
  ai::flow::adas::LaneLines ll;
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

TEST(AdasConfigFile, ShippedJsonReachesTheTopicConvertKnobs)
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

  EXPECT_DOUBLE_EQ(veh["path_lane_blend_scale"].asDouble(), cfg.topic_convert.path_lane_blend_scale);
  EXPECT_DOUBLE_EQ(veh["path_camera_offset_m"].asDouble(), cfg.topic_convert.path_camera_offset_m);
  EXPECT_DOUBLE_EQ(veh["lane_std_good_m"].asDouble(), cfg.topic_convert.lane_std_good_m);
  EXPECT_DOUBLE_EQ(veh["lane_std_bad_m"].asDouble(), cfg.topic_convert.lane_std_bad_m);
  EXPECT_DOUBLE_EQ(veh["lane_width_min_m"].asDouble(), cfg.topic_convert.lane_width_min_m);
  EXPECT_DOUBLE_EQ(veh["lane_width_max_m"].asDouble(), cfg.topic_convert.lane_width_max_m);
  EXPECT_DOUBLE_EQ(veh["center_force_gain"].asDouble(), cfg.topic_convert.center_force_gain);
  EXPECT_DOUBLE_EQ(veh["center_force_max_m"].asDouble(), cfg.topic_convert.center_force_max_m);
  EXPECT_DOUBLE_EQ(veh["center_force_turn_scale"].asDouble(), cfg.topic_convert.center_force_turn_scale);

  EXPECT_DOUBLE_EQ(root["calibration"]["camera"]["position_m"]["y_left"].asDouble(), cfg.topic_convert.cam_y_left_m);
}

TEST(LaneBlendRuntimeKnob, TakesEffectAndIsClamped)
{
  adas::TopicConvertService::Config cfg;
  cfg.path_lane_blend_scale = 0.6;
  adas::TopicConvertService svc(cfg);
  EXPECT_DOUBLE_EQ(svc.config().path_lane_blend_scale, 0.6);

  svc.setLaneBlendScale(1.0);
  EXPECT_DOUBLE_EQ(svc.config().path_lane_blend_scale, 1.0);

  svc.setLaneBlendScale(1.4);
  EXPECT_DOUBLE_EQ(svc.config().path_lane_blend_scale, 1.0) << "above 1.0 is meaningless";
  svc.setLaneBlendScale(-0.2);
  EXPECT_DOUBLE_EQ(svc.config().path_lane_blend_scale, 0.0) << "negative weight would invert the path";
}
