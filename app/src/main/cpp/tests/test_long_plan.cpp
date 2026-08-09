#include <cmath>
#include <cstdlib>
#include <fstream>
#include <vector>

#include <json/json.h>

#include "adas/adas_app.h"

#include <gtest/gtest.h>

#include "adas/utils/curvature_preview.h"
#include "adas/utils/long_planner.hpp"

using adas::curvatureSpeedLimit;
using adas::maxCurvatureAhead;
using adas::Vec2;

namespace {

std::vector<Vec2> arc(double radius, double length_m, double step = 1.0)
{
  std::vector<Vec2> pts;
  for (double s = 0.0; s <= length_m; s += step) {
    const double th = s / radius;
    pts.push_back({radius * std::sin(th), radius * (1.0 - std::cos(th))});
  }
  return pts;
}

std::vector<Vec2> straight(double length_m, double step = 1.0)
{
  std::vector<Vec2> pts;
  for (double s = 0.0; s <= length_m; s += step)
    pts.push_back({s, 0.0});
  return pts;
}

}  // namespace

TEST(CurvaturePreview, StraightAsksForNoLimit)
{
  const double k = maxCurvatureAhead(straight(80.0), 10.0, 70.0);
  EXPECT_NEAR(0.0, k, 1e-4);
  EXPECT_DOUBLE_EQ(27.0, curvatureSpeedLimit(k, 1.8, 27.0));
}

TEST(CurvaturePreview, RecoversArcCurvature)
{
  for (const double radius : {120.0, 200.0, 400.0}) {
    const double k = maxCurvatureAhead(arc(radius, 90.0), 10.0, 80.0);
    EXPECT_NEAR(1.0 / radius, k, 0.15 / radius) << "radius " << radius;
  }
}

TEST(CurvaturePreview, SpeedLimitMatchesLateralAcceleration)
{
  EXPECT_NEAR(19.0, curvatureSpeedLimit(1.0 / 200.0, 1.8, 40.0), 0.2);
  EXPECT_NEAR(8.6, curvatureSpeedLimit(1.0 / 41.0, 1.8, 40.0), 0.2);
}

TEST(CurvaturePreview, IgnoresASingleBadSample)
{
  std::vector<Vec2> poly = straight(80.0);
  poly[40].y() += 0.5;
  const double k = maxCurvatureAhead(poly, 10.0, 70.0);
  EXPECT_LT(k, 1.0 / 500.0);
}

TEST(CurvaturePreview, TooShortAPathAsksForNothing)
{
  EXPECT_DOUBLE_EQ(0.0, maxCurvatureAhead(straight(8.0), 10.0, 70.0));
  EXPECT_DOUBLE_EQ(0.0, maxCurvatureAhead({}, 10.0, 70.0));
}

TEST(CurvaturePreview, LooksAtTheTightestPartAhead)
{
  std::vector<Vec2> poly = straight(40.0);
  const auto tail = arc(100.0, 50.0);
  for (const auto& p : tail)
    poly.push_back({40.0 + p.x(), p.y()});

  const double near_k = maxCurvatureAhead(poly, 5.0, 35.0);
  const double far_k = maxCurvatureAhead(poly, 5.0, 85.0);
  EXPECT_LT(near_k, 1.0 / 500.0);
  EXPECT_GT(far_k, 0.5 / 100.0);
}

// ── Plan rules ────────────────────────────────────────────────────────────────────────────────
//
// Every case below is a defect found on run 2026_08_06_00_36_42, where the plan ran the cruise
// buttons for the first time and asked for a set speed roughly 5 m/s below the actual speed for the
// whole drive: 715 down-tips and 690 up-tips in 28 minutes.

namespace {

adas::longplan::Config planConfig()
{
  adas::longplan::Config c;
  c.curv_enabled = false;  // curvature is exercised separately; keep the lead rules isolated
  return c;
}

adas::longplan::Input freeFlow(double v_ego, const std::vector<Vec2>* path = nullptr)
{
  adas::longplan::Input in;
  in.v_ego = v_ego;
  in.path = path;
  return in;
}

}  // namespace

TEST(LongPlan, FreeFlowHoldsCurrentSpeed)
{
  const auto plan = adas::longplan::compute(planConfig(), freeFlow(20.0));
  EXPECT_EQ(plan.source, "hold");
  EXPECT_NEAR(plan.v_target, 20.0, 1e-9);
  EXPECT_NEAR(plan.a_target, 0.0, 1e-9);
}

TEST(LongPlan, ModelPlanVelocityIsNotUsedAsAnAbsoluteTarget)
{
  // plan_v0 / v_ego measured at 0.678 on the run. If this source were live it would demand
  // -6 m/s at 20 m/s for no reason at all.
  auto in = freeFlow(20.0);
  in.plan_v.valid = true;
  in.plan_v.v_plan = 13.6;
  in.plan_v.pose_valid = true;
  in.plan_v.pose_vx = 17.8;

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_EQ(plan.source, "hold");
  EXPECT_NEAR(plan.v_target, 20.0, 1e-9);

  adas::longplan::Config on = planConfig();
  on.plan_v_enabled = true;
  const auto opted_in = adas::longplan::compute(on, in);
  EXPECT_EQ(opted_in.source, "plan_v");
  EXPECT_LT(opted_in.v_target, 20.0);
}

TEST(LongPlan, StationaryLeadIsNotASpeedTarget)
{
  // 31 % of the run's lead ticks reported a near-stopped object — parked cars at night. We cannot
  // brake, so matching one as a speed target only collapses the set speed.
  auto in = freeFlow(11.0);
  in.lead.prob = 0.99;
  in.lead.d_rel = 21.0;
  in.lead.v_lead = 0.2;

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_FALSE(plan.has_lead);
  EXPECT_EQ(plan.source, "hold");
  EXPECT_NEAR(plan.v_target, 11.0, 1e-9);
}

TEST(LongPlan, MovingLeadInOurLaneIsFollowed)
{
  // Slightly slower lead at the desired gap: the whole speed difference is inside the coast
  // envelope, so the target is the lead's speed outright.
  auto in = freeFlow(20.0);
  in.lead.prob = 0.9;
  in.lead.d_rel = 30.0;
  in.lead.v_lead = 19.5;

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_TRUE(plan.has_lead);
  EXPECT_EQ(plan.source, "lead");
  EXPECT_EQ(plan.status, "ok");
  EXPECT_NEAR(plan.v_target, 19.5, 1e-9);
}

TEST(LongPlan, MuchSlowerLeadWalksTheTargetDownAtTheCoastRate)
{
  // A 5 m/s deficit cannot be closed by lifting the throttle, so the target is not the lead's speed
  // but the speed reachable in one coast step. Re-running from the new `v_ego` each tick walks it
  // down; publishing the lead's speed directly is what pegged the cruise buttons.
  adas::longplan::Config cfg = planConfig();
  auto in = freeFlow(20.0);
  in.lead.prob = 0.9;
  in.lead.d_rel = 30.0;
  in.lead.v_lead = 15.0;

  const auto plan = adas::longplan::compute(cfg, in);
  EXPECT_TRUE(plan.has_lead);
  EXPECT_EQ(plan.status, "brake_needed");
  EXPECT_NEAR(plan.v_target, 20.0 + cfg.a_coast_ms2 * adas::longplan::kCoastHorizonS, 1e-6);
}

TEST(LongPlan, SettledBehindALeadAsksForNothing)
{
  // At the desired gap and matched speed the plan must be neutral, otherwise the button actuator
  // hunts around a target that never stops moving.
  auto in = freeFlow(20.0);
  in.lead.prob = 0.9;
  in.lead.v_lead = 20.0;
  in.lead.d_rel = 1.5 * 20.0;  // exactly t_follow

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_TRUE(plan.has_lead);
  EXPECT_EQ(plan.status, "ok");
  EXPECT_NEAR(plan.a_target, 0.0, 1e-9);
  EXPECT_NEAR(plan.v_target, 20.0, 1e-9);
}

TEST(LongPlan, FarSlowerLeadDoesNotAnnounceTheWholeDeficit)
{
  // The defect that produced most of the button traffic: a lead 100 m ahead and 5 m/s slower needs
  // no action yet, but `v_target = v_lead` published the full deficit and the actuator tipped down.
  auto in = freeFlow(20.0);
  in.lead.prob = 0.9;
  in.lead.d_rel = 100.0;
  in.lead.v_lead = 15.0;

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_TRUE(plan.has_lead);
  EXPECT_GT(plan.a_target, 0.0);   // the gap is larger than we want, so no braking is due
  EXPECT_GE(plan.v_target, 20.0);  // and certainly not a request to drop 5 m/s
}

TEST(LongPlan, LeadOutsideOurLaneIsIgnored)
{
  const auto path = straight(80.0);
  auto in = freeFlow(20.0, &path);
  in.lead.prob = 0.9;
  in.lead.d_rel = 30.0;
  in.lead.v_lead = 8.0;
  in.lead.y_rel = 3.5;  // next lane over

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_FALSE(plan.lead_in_lane);
  EXPECT_FALSE(plan.has_lead);
  EXPECT_NEAR(plan.v_target, 20.0, 1e-9);
}

TEST(LongPlan, LeadOnAnArcStaysInOurLaneBecausePathIsTheReference)
{
  // Compared against the vehicle axis a lead 30 m into an R=200 m bend sits 2.2 m off and would be
  // discarded; compared against our own path it is where it should be.
  const auto path = arc(200.0, 80.0);
  auto in = freeFlow(20.0, &path);
  in.lead.prob = 0.9;
  in.lead.d_rel = 30.0;
  in.lead.v_lead = 15.0;
  in.lead.y_rel = adas::longplan::pathYAt(path, 30.0) + 0.4;

  const auto plan = adas::longplan::compute(planConfig(), in);
  EXPECT_TRUE(plan.lead_in_lane);
  EXPECT_TRUE(plan.has_lead);
}

TEST(LongPlan, BrakingIsCappedAtTheMeasuredCoastEnvelope)
{
  auto in = freeFlow(25.0);
  in.lead.prob = 0.99;
  in.lead.d_rel = 12.0;   // much closer than t_follow wants
  in.lead.v_lead = 10.0;  // and much slower

  const auto cfg = planConfig();
  const auto plan = adas::longplan::compute(cfg, in);
  EXPECT_EQ(plan.status, "brake_needed");
  EXPECT_GE(plan.a_target, cfg.a_coast_ms2 - 1e-9);
  // The target must stay reachable, otherwise the actuator tips the set speed down forever.
  EXPECT_NEAR(plan.v_target, 25.0 + cfg.a_coast_ms2 * adas::longplan::kCoastHorizonS, 1e-6);
}

TEST(LongPlan, AccelerationSideKeepsItsOwnLimit)
{
  auto in = freeFlow(10.0);
  in.lead.prob = 0.99;
  in.lead.d_rel = 100.0;
  in.lead.v_lead = 25.0;

  const auto cfg = planConfig();
  const auto plan = adas::longplan::compute(cfg, in);
  EXPECT_EQ(plan.status, "ok");
  EXPECT_NEAR(plan.a_target, cfg.a_max, 1e-9);
}

TEST(LongPlan, CurvatureLimiterIsMeasuredAgainstCurrentSpeedNotTheReducedTarget)
{
  // The published `v_curv` used to be capped by `v_target`, which made it a copy of it: the field
  // claimed the limiter was active in 79 % of ticks while it had changed the target in 0.5 %.
  const auto path = straight(200.0);
  adas::longplan::Config cfg = planConfig();
  cfg.curv_enabled = true;

  auto in = freeFlow(20.0, &path);
  in.lead.prob = 0.99;
  in.lead.d_rel = 40.0;
  in.lead.v_lead = 12.0;  // target pulled well below v_ego by the lead, not by curvature

  const auto plan = adas::longplan::compute(cfg, in);
  EXPECT_EQ(plan.source, "lead");  // no "+curv" — a straight road adds no limit
  EXPECT_GT(plan.v_curv, plan.v_target);
}

TEST(LongPlan, TightArcAheadLowersTheTarget)
{
  // R = 120 m, not tighter: `arc()` parameterises by heading, so at R = 60 m the polyline passes
  // 90 deg within the preview window and folds back on itself in x, which no x-indexed fit can read.
  const auto path = arc(120.0, 150.0);
  adas::longplan::Config cfg = planConfig();
  cfg.curv_enabled = true;

  const auto plan = adas::longplan::compute(cfg, freeFlow(20.0, &path));
  EXPECT_NE(plan.source.find("curv"), std::string::npos);
  EXPECT_LT(plan.v_target, 20.0);
  // sqrt(a_lat / kappa) at R = 120 m and 1.8 m/s^2 is 14.7 m/s, but reaching it from 20 m/s needs
  // (400 - 216) / (2 * 0.30) = 307 m of coasting while the preview is 80 m. So the honest answer for
  // this car is a coast-limited step plus a request for the driver, not the corner speed.
  EXPECT_EQ(plan.status, "brake_needed");
  EXPECT_NEAR(plan.v_target, 20.0 + cfg.a_coast_ms2 * adas::longplan::kCoastHorizonS, 1e-6);
  EXPECT_GT(plan.v_curv, 14.0);
  EXPECT_LT(plan.v_curv, 15.5);

  // Once slow enough that coasting can hold the corner speed, the limiter delivers it outright.
  const auto slow = adas::longplan::compute(cfg, freeFlow(15.0, &path));
  EXPECT_EQ(slow.status, "ok");
  EXPECT_NEAR(slow.v_target, std::sqrt(1.8 * 120.0), 1.0);
}

// ── Shipped config ────────────────────────────────────────────────────────────────────────────
//
// This suite exists because of a specific failure: on 2026-08-04 the FCW/AEB speed gate was raised
// 3 -> 8 m/s in `safety_planner.hpp`, but `assets/config.json` kept its own 3.0 and the file wins.
// The fix was therefore dead, and run 2026_08_06_00_36_42 reproduced the same three false warnings
// against a stationary object at 4.8-5.5 m/s. A header default is not a decision until the shipped
// config agrees with it.

TEST(ShippedConfig, LongitudinalKnobsReachTheService)
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
  const Json::Value& lp = root["long_plan"];
  ASSERT_TRUE(lp.isObject());

  EXPECT_DOUBLE_EQ(lp["a_coast_ms2"].asDouble(), cfg.long_plan.a_coast_ms2);
  EXPECT_DOUBLE_EQ(lp["lead_max_offset_m"].asDouble(), cfg.long_plan.lead_max_offset_m);
  EXPECT_DOUBLE_EQ(lp["lead_min_speed_ms"].asDouble(), cfg.long_plan.lead_min_speed_ms);
  EXPECT_EQ(lp["plan_v_enabled"].asBool(), cfg.long_plan.plan_v_enabled);
}

TEST(ShippedConfig, DoesNotPromiseBrakingThisCarCannotDeliver)
{
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(ADAS_SHIPPED_CONFIG_JSON, &ok);
  ASSERT_TRUE(ok);
  // No radar, no ACC, no brake-by-wire: the plan may not ask for more than coasting gives, and the
  // measured envelope is around -0.3 m/s^2.
  EXPECT_LE(cfg.long_plan.a_coast_ms2, 0.0);
  EXPECT_GE(cfg.long_plan.a_coast_ms2, -1.0) << "stronger than anything measured on this car";
  EXPECT_FALSE(cfg.long_plan.plan_v_enabled) << "plan_v0/v_ego measured at 0.678 — not a speed target";
}

TEST(ShippedConfig, WarningGatesMatchTheDecisionsTakenOnRoad)
{
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(ADAS_SHIPPED_CONFIG_JSON, &ok);
  ASSERT_TRUE(ok);
  const auto& p = cfg.safety_warn.planner;
  EXPECT_GE(p.warn_min_speed_ms, 8.0) << "stop-and-go false positives, runs 08-04 and 08-06";
  EXPECT_TRUE(p.ldw_suppress_on_lat_active) << "82 % of LDW frames were our own tracking error";
  EXPECT_TRUE(p.ldw_suppress_on_blinker);
}
