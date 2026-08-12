#include <cmath>
#include <fstream>
#include <vector>

#include <json/json.h>

#include <gtest/gtest.h>

#include "adas/lateral/flowpilot_mpc.hpp"
#include "adas/adas_app.h"
#include "messages.pb.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/lane_keep.h"
#include "adas/utils/path_lateral_state.h"
#include "adas/utils/proto_convert.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"
#include "adas/lateral/visionpilot_mpc.hpp"

using adas::estimatePathLateralState;
using adas::PathLateralState;
using adas::Vec2;
using adas::services::LaneKeep;
using visionpilot::build_kappa_schedule;
using visionpilot::LateralPlanner;

namespace {
std::vector<Vec2> straightLaneRightOffset(double offset_m)
{
  std::vector<Vec2> poly;
  for (int i = 2; i <= 40; ++i)
    poly.emplace_back(static_cast<double>(i), offset_m);
  return poly;
}

// Кормить kVisionPath значило бы проверять сервис против сообщения, которого он не слушает.
adas::proto::LaneLines lanesFromPolyline(const std::vector<Vec2>& poly, int64_t ts_us)
{
  adas::proto::LaneLines ll;
  ll.set_timestamp(ts_us / 1000);
  ll.set_capture_ts_ms(ts_us / 1000);
  for (const auto& pt : poly) {
    ll.add_plan_x(pt.x());
    ll.add_plan_y(pt.y());
  }
  return ll;
}

LaneKeep::Config mpcConfig()
{
  LaneKeep::Config c;
  c.controller = "mpc";
  return c;
}

}  // namespace

TEST(PathLateralState, StraightOffsetRightGivesNegativeCteInVpFrame)
{
  std::vector<Vec2> poly;
  for (int i = 2; i <= 30; ++i)
    poly.emplace_back(static_cast<double>(i), 0.5);

  const PathLateralState s = estimatePathLateralState(poly);
  ASSERT_TRUE(s.valid);
  EXPECT_NEAR(s.cte_m, -0.5, 0.05);
  EXPECT_NEAR(s.epsi_rad, 0.0, 0.05);
  EXPECT_NEAR(s.kappa, 0.0, 0.01);
}

TEST(VisionPilotMpc, FeedforwardMatchesAckermannOnCurve)
{
  const visionpilot::Params params;
  LateralPlanner mpc(params);
  const double Lf = 2.67;
  const double kappa = 0.02;
  const double v = 10.0;
  Eigen::Vector3d state;
  state << 0.0, 0.0, kappa;
  const auto ks = build_kappa_schedule(Lf, 0.0, kappa, 0.0, params.N);
  Eigen::VectorXd vs(static_cast<int>(params.N));
  vs.setConstant(v);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);

  const double ff = std::atan(Lf * kappa) + 0.0015 * v * v * kappa;
  EXPECT_NEAR(deltas[1], ff, 0.08);
}

TEST(VisionPilotMpc, CorrectsPositiveCte)
{
  const visionpilot::Params params;
  LateralPlanner mpc(params);
  const double Lf = 2.67;
  Eigen::Vector3d state;
  state << 0.8, 0.0, 0.0;
  const auto ks = build_kappa_schedule(Lf, 0.0, 0.0, 0.0, params.N);
  Eigen::VectorXd vs(static_cast<int>(params.N));
  vs.setConstant(8.0);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);

  EXPECT_NE(deltas[1], 0.0);
  EXPECT_GT(deltas[1], 0.0);
}

TEST(VisionPilotMpc, LowSpeedModerateCteDoesNotSaturate)
{
  const visionpilot::Params params;
  LateralPlanner mpc(params);
  const double Lf = 2.67;
  Eigen::Vector3d state;
  state << 0.6, 0.0, 0.0;
  const auto ks = build_kappa_schedule(Lf, 0.0, 0.0, 0.0, params.N);
  Eigen::VectorXd vs(static_cast<int>(params.N));
  vs.setConstant(1.5);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);
  EXPECT_GT(deltas[1], 0.0);
  EXPECT_LT(std::abs(deltas[1]), 0.40);
}

TEST(LaneKeepServiceMpc, LowSpeedClampNearPpLimit)
{
  LaneKeep::Config c = mpcConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  LaneKeep svc(c);
  const auto poly = straightLaneRightOffset(1.5);
  const auto out = svc.step(1.0, poly);
  ASSERT_EQ(out.status, "ok");

  EXPECT_LE(std::abs(out.steer_rad), 9.0 * M_PI / 180.0 + 1e-6);
}

TEST(LaneKeepServiceMpc, FrameDtComesFromMessageTimestamps)
{
  LaneKeep::Config c = mpcConfig();
  LaneKeep svc(c);

  auto frame = [](double offset_m, int64_t ts_us) {
    adas::LanePathMsg m;
    m.polyline = straightLaneRightOffset(offset_m);
    m.capture_ts_us = ts_us;
    m.timestamp_us = ts_us;
    return m;
  };

  auto flip_step_deg = [&](int64_t period_us) {
    LaneKeep s(c);
    int64_t ts = 1'000'000;
    auto prev = s.step(10.0, frame(2.0, ts));
    EXPECT_EQ(prev.status, "ok");
    for (int i = 0; i < 8; ++i) {
      ts += period_us;
      prev = s.step(10.0, frame(2.0, ts));
    }
    ts += period_us;
    const auto flipped = s.step(10.0, frame(-2.0, ts));
    EXPECT_EQ(flipped.status, "ok");
    return std::abs(flipped.steer_rad - prev.steer_rad) * 180.0 / M_PI;
  };

  auto bound = [&](double dt) { return (c.mpc_max_lateral_jerk / (10.0 * 10.0) * dt) * c.mpc_Lf * 180.0 / M_PI; };

  const double fast = flip_step_deg(50'000);
  const double slow = flip_step_deg(200'000);

  EXPECT_LE(fast, bound(0.06) + 1e-3);
  EXPECT_LE(slow, bound(0.2) + 1e-3);
  EXPECT_LE(slow, c.mpc_rate_limit_deg);
  EXPECT_GT(slow, 1.5 * fast);
}

TEST(LaneKeepServiceMpc, RateLimitBoundsFrameToFrameStep)
{
  LaneKeep::Config c = mpcConfig();
  LaneKeep svc(c);

  const auto o1 = svc.step(10.0, straightLaneRightOffset(0.8));
  ASSERT_EQ(o1.status, "ok");

  const auto o2 = svc.step(10.0, straightLaneRightOffset(-0.8));
  ASSERT_EQ(o2.status, "ok");
  const double step_deg = std::abs(o2.steer_rad - o1.steer_rad) * 180.0 / M_PI;

  const double expected_deg =
      (c.mpc_max_lateral_jerk / (10.0 * 10.0) * c.vision_nominal_dt_s) * c.mpc_Lf * 180.0 / M_PI;
  EXPECT_LE(step_deg, expected_deg + 1e-3);
  EXPECT_LT(step_deg, 3.0);
}

TEST(LaneKeepServiceMpc, KappaYawBlendDefaultsToPathOnly)
{
  LaneKeep::Config c = mpcConfig();
  EXPECT_DOUBLE_EQ(c.mpc_kappa_yaw_blend, 0.0);
  LaneKeep svc(c);

  std::vector<Vec2> poly;
  for (int i = 2; i <= 40; ++i) {
    const double x = static_cast<double>(i);
    poly.emplace_back(x, 0.002 * x * x);
  }
  const auto lat = estimatePathLateralState(poly);
  ASSERT_TRUE(lat.valid);

  const auto out = svc.step(8.0, poly);
  ASSERT_EQ(out.status, "ok");

  EXPECT_DOUBLE_EQ(out.curvature, lat.kappa);
}

TEST(LaneKeep, CamYLeftShiftsPathToVehicleFrame)
{
  LaneKeep::Config c = mpcConfig();
  c.cam_y_left_m = 0.5;
  LaneKeep svc(c);

  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, 0.0, 0.05);
  EXPECT_NEAR(out.steer_rad, 0.0, 0.05);
}

TEST(LaneKeep, CamYLeftZeroLeavesCteUnchanged)
{
  LaneKeep::Config c = mpcConfig();
  EXPECT_DOUBLE_EQ(c.cam_y_left_m, 0.0);
  LaneKeep svc(c);
  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, -0.5, 0.05);
}

TEST(LaneKeepServicePublish, SlewGuardConfigDefaultActive)
{
  LaneKeep::Config c;
  EXPECT_GT(c.steer_slew_limit_deg, 0.0);
  EXPECT_LE(c.steer_slew_limit_deg, 25.0);
}

TEST(LaneKeepServiceMpc, StraightModerateCteDoesNotRailAtSpeed)
{
  LaneKeep svc(mpcConfig());

  const auto slow = svc.step(4.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(slow.status, "ok");
  const auto fast = svc.step(20.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(fast.status, "ok");

  EXPECT_LT(std::abs(slow.steer_rad), 0.20);
  EXPECT_LT(std::abs(fast.steer_rad), 0.20);

  EXPECT_LT(std::abs(fast.steer_rad), std::abs(slow.steer_rad));
}

TEST(LaneKeepServiceMpc, RateLimitCeilingHoldsAtLowSpeed)
{
  LaneKeep::Config c = mpcConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  LaneKeep svc(c);
  const auto o1 = svc.step(1.0, straightLaneRightOffset(0.8));
  ASSERT_EQ(o1.status, "ok");
  const auto o2 = svc.step(1.0, straightLaneRightOffset(-0.8));
  ASSERT_EQ(o2.status, "ok");
  const double step_deg = std::abs(o2.steer_rad - o1.steer_rad) * 180.0 / M_PI;
  EXPECT_LE(step_deg, c.mpc_rate_limit_deg + 1e-3);
}

TEST(LaneKeepServiceFlowpilot, StraightOffsetSteersTowardCenter)
{
  LaneKeep::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeep svc(c);

  adas::LaneKeepOutput out;
  for (int i = 0; i < 8; ++i)
    out = svc.step(8.0, straightLaneRightOffset(0.6));
  ASSERT_EQ(out.status, "ok");
  ASSERT_EQ(out.controller, "fp");
  EXPECT_GT(out.steer_rad, 0.02);
  EXPECT_LT(std::abs(out.steer_rad), 0.45);
}

TEST(LaneKeepServiceFlowpilot, ControllerSwitchAcceptsFlowpilot)
{
  LaneKeep::Config c = mpcConfig();
  c.controller = "pp";
  LaneKeep svc(c);
  svc.setController("fp");
  EXPECT_TRUE(svc.useFlowpilot());
  const auto out = svc.step(12.0, straightLaneRightOffset(0.0));
  ASSERT_EQ(out.status, "ok");
  EXPECT_EQ(out.controller, "fp");
}

TEST(LaneKeepServiceFlowpilot, CurvedPathCommandsNonZero)
{
  LaneKeep::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeep svc(c);
  std::vector<Vec2> poly;
  for (int i = 2; i <= 40; ++i) {
    const double x = static_cast<double>(i);
    poly.emplace_back(x, 0.002 * x * x);
  }
  adas::LaneKeepOutput out;
  for (int i = 0; i < 10; ++i)
    out = svc.step(10.0, poly);
  ASSERT_EQ(out.status, "ok");

  EXPECT_GT(out.steer_rad, 0.005);
}

TEST(LaneKeepServiceMpc, LowSpeedGateHoldsZeroAndHasHysteresis)
{
  LaneKeep::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeep svc(c);
  const auto poly = straightLaneRightOffset(1.0);

  const auto creeping = svc.step(0.9, poly);
  EXPECT_EQ(creeping.status, "low_speed");
  EXPECT_DOUBLE_EQ(creeping.steer_rad, 0.0);
  EXPECT_FALSE(creeping.has_target);

  const auto band = svc.step(c.min_control_speed_mps + 0.1, poly);
  EXPECT_EQ(band.status, "low_speed");

  const auto rolling = svc.step(c.min_control_speed_mps + c.min_control_speed_hyst_mps + 0.5, poly);
  EXPECT_EQ(rolling.status, "ok");
  EXPECT_NE(rolling.steer_rad, 0.0);

  const auto slowing = svc.step(c.min_control_speed_mps - 0.1, poly);
  EXPECT_EQ(slowing.status, "low_speed");
}

namespace {
class SteerSink : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "steer_sink"; }

  void configure() override
  {
    subscribe<adas::proto::SteerCommand>(adas::topics::kSteerCommand, [this](const adas::proto::SteerCommand& m) {
      last_enabled = m.enabled();
      last_torque = m.torque_cnm();
      ++count;
    });
  }

  bool last_enabled = false;
  int last_torque = 0;
  int count = 0;
};

}  // namespace

TEST(LaneKeepStaleGate, OldReferenceDisablesTheCommand)
{
  LaneKeep::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.30;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  auto lk = mw.registerService<LaneKeep>(c);
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  int64_t now_us = 10'000'000;
  mw.setTime(now_us);

  auto feed = [&](int64_t capture_us) {
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), capture_us));
    adas::ChassisSample ch;
    ch.timestamp_us = capture_us;
    ch.speed_mps = 14.0;
    ch.steering_angle_deg = 0.0;
    mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
    mw.step();
  };

  feed(now_us);
  ASSERT_GT(sink->count, 0) << "controls/steer not published at all";
  EXPECT_TRUE(sink->last_enabled);

  feed(now_us - 1'000'000);
  EXPECT_FALSE(sink->last_enabled) << "must not steer on a stale path";
  EXPECT_EQ(0, sink->last_torque);

  now_us += 100'000;
  mw.setTime(now_us);
  feed(now_us);
  EXPECT_TRUE(sink->last_enabled);
}

TEST(LaneKeepStaleGate, ZeroDisablesTheGate)
{
  LaneKeep::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  mw.registerService<LaneKeep>(c);
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  mw.setTime(10'000'000);
  const int64_t old_us = 10'000'000 - 5'000'000;
  mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), old_us));
  adas::ChassisSample ch;
  ch.timestamp_us = old_us;
  ch.speed_mps = 14.0;
  mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
  mw.step();

  EXPECT_TRUE(sink->last_enabled) << "with gate off, prior behavior";
}

// Turn signal hands the wheel back: we have no lane-change planner, so holding the current lane
// while the driver crosses the line fights them. Signal presence verified on run
// 2026_08_04_21_00_18 (left 7 episodes, right 11).
TEST(LaneKeepBlinkerGate, TurnSignalClearsTheCommandAndResumesAfterDelay)
{
  LaneKeep::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;  // staleness gate off — this test is about the blinker only
  c.lka_suppress_on_blinker = true;
  c.lka_blinker_resume_delay_s = 0.2;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  mw.registerService<LaneKeep>(c);
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  // Time is driven explicitly: in simulated mode `now()` returns sim_us_, and without setTime the clock
  // stands at zero. Sleeping on the wall clock did not exercise the resume delay at all — the test
  // passed through a stuck status rather than through the timer.
  uint64_t sim_us = 1'000'000;
  auto feed = [&](bool left, bool right) {
    mw.setTime(sim_us);
    const int64_t ts = static_cast<int64_t>(sim_us);
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), ts));

    adas::ChassisSample ch;
    ch.timestamp_us = ts;
    ch.speed_mps = 14.0;
    ch.left_blinker = left;
    ch.right_blinker = right;
    mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
    mw.step();
  };

  feed(false, false);
  ASSERT_GT(sink->count, 0);
  EXPECT_TRUE(sink->last_enabled) << "no signal — we steer";

  feed(true, false);
  EXPECT_FALSE(sink->last_enabled) << "left signal on — wheel handed back";
  EXPECT_EQ(0, sink->last_torque);

  feed(false, true);
  EXPECT_FALSE(sink->last_enabled) << "right signal on — same";

  // Falling edge: still suppressed until the resume delay elapses.
  feed(false, false);
  EXPECT_FALSE(sink->last_enabled) << "signal just cancelled — the car is still crossing the line";
  sim_us += 250'000;
  feed(false, false);
  EXPECT_TRUE(sink->last_enabled) << "delay elapsed — steering again";
}

TEST(LaneKeepBlinkerGate, CanBeSwitchedOff)
{
  LaneKeep::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;
  c.lka_suppress_on_blinker = false;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  mw.registerService<LaneKeep>(c);
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  const int64_t ts = static_cast<int64_t>(mw.now());
  mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), ts));
  adas::ChassisSample ch;
  ch.timestamp_us = ts;
  ch.speed_mps = 14.0;
  ch.left_blinker = true;
  mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
  mw.step();

  EXPECT_TRUE(sink->last_enabled);
}

namespace {
struct AssistRig {
  explicit AssistRig(bool require_assist = true, double max_age_s = 0.5)
    : sink(std::make_shared<SteerSink>()), mw(adas::middleware::Manager::Mode::Simulated)
  {
    LaneKeep::Config c = mpcConfig();
    c.steer_output_enabled = true;
    c.min_control_speed_mps = 0.0;
    c.min_control_speed_hyst_mps = 0.0;
    c.lane_max_age_s = 0.0;
    c.lka_suppress_on_blinker = false;
    c.lat_require_assist = require_assist;
    c.assist_max_age_s = max_age_s;
    lk = mw.registerService<LaneKeep>(c);
    mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));
    mw.setTime(now_us);
  }

  void health(bool lat_allowed, bool controls_allowed_field = false)
  {
    health_t raw{};
    raw.controls_allowed_pkt = controls_allowed_field ? 1 : 0;
    raw.safety_mode_pkt = volkswagen::MqbSafetyConstants::kVolkswagen;
    auto msg = utils::createHealthMessage(raw, static_cast<int64_t>(mw.now() / 1000));
    msg.set_lat_actuation_allowed(lat_allowed);
    mw.publish(adas::topics::kPandaHealth, msg);
  }

  void tick()
  {
    now_us += 20'000;
    mw.setTime(now_us);
    // По шине ходит схема, поэтому тест подаёт то же, что подал бы транспорт.
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), now_us));

    adas::proto::CarState cs;
    cs.set_timestamp(now_us / 1000);
    cs.set_v_ego(14.0);
    cs.set_steering_angle_deg(0.0);
    mw.publish(adas::topics::kVehicleState, cs);
    mw.step();
  }

  std::shared_ptr<SteerSink> sink;
  adas::middleware::Manager mw;
  std::shared_ptr<LaneKeep> lk;
  int64_t now_us = 10'000'000;
};

}  // namespace

// Having never heard from a panda must not gate: that is a bag replay, the Python bindings or a bench
// run, and closing the gate there would silence the command in every offline harness we measure with.
TEST(LaneKeepAssistGate, NoPandaReportMeansNoGate)
{
  AssistRig r;
  r.tick();
  ASSERT_GT(r.sink->count, 0) << "controls/steer not published at all";
  EXPECT_TRUE(r.sink->last_enabled) << "no panda in the loop — the offline harness must still steer";
  EXPECT_FALSE(r.lk->last().dbg.assist_known) << "and it must say it does not know";
  EXPECT_FALSE(r.lk->last().dbg.assist_allowed);
}

TEST(LaneKeepAssistGate, WithheldTorqueClearsTheCommandAndComesBack)
{
  AssistRig r;
  r.health(true);
  r.tick();
  ASSERT_TRUE(r.sink->last_enabled) << "assist present — we steer";
  EXPECT_TRUE(r.lk->last().dbg.assist_known);
  EXPECT_TRUE(r.lk->last().dbg.assist_allowed);

  r.health(false);
  r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "panda is not passing torque — do not pretend to steer";
  EXPECT_EQ(0, r.sink->last_torque);
  EXPECT_EQ("no_assist", r.lk->last().status);
  EXPECT_FALSE(r.lk->last().dbg.assist_allowed);
  EXPECT_TRUE(r.lk->last().dbg.assist_known) << "absent is not the same as unknown";

  r.health(true);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "assist back — steering again";
  EXPECT_EQ("ok", r.lk->last().status) << "the status must clear, not only be set";
}

// The defect this task exists for. Two rigs see the same standing error and resume on the same tick
// spacing; they differ only in how long the torque was withheld beforehand. With the integrator reset
// the first resumed command is identical in both. Without it, the longer withholding accumulates and
// the wheel gets a step it never earned.
// The `require_assist = false` half is the control group, and it is what keeps this from being a test
// that passes for the wrong reason: it reproduces the pre-fix behaviour in the same file and shows the
// resumed command really does grow with the time spent withheld.
TEST(LaneKeepAssistGate, TheIntegratorDoesNotWindUpWhileTorqueIsWithheld)
{
  auto resumedTorque = [](int withheld_ticks, bool require_assist) {
    AssistRig r(require_assist);
    r.health(false);
    for (int i = 0; i < withheld_ticks; ++i)
      r.tick();
    if (require_assist)
      EXPECT_EQ(0, r.sink->last_torque) << "nothing should be commanded while the torque is withheld";
    r.health(true);
    r.tick();
    return r.sink->last_torque;
  };

  const int gated_5 = resumedTorque(5, true);
  const int gated_50 = resumedTorque(50, true);
  EXPECT_NE(0, gated_5) << "the comparison is vacuous if the resumed command is zero";
  EXPECT_EQ(gated_5, gated_50) << "torque on resumption grew with the time spent withheld — windup";

  const int ungated_5 = resumedTorque(5, false);
  const int ungated_50 = resumedTorque(50, false);
  EXPECT_GT(std::abs(ungated_50), std::abs(ungated_5)) << "control group: without the gate the integrator must wind "
                                                          "up, otherwise this test proves nothing";
  // Measured here: 5 withheld ticks resume at -209 cNm and 50 at -283, i.e. one second of pretending
  // to steer adds 74 cNm the wheel never earned. Gated, both resume at -203.
  EXPECT_GT(std::abs(ungated_50), std::abs(gated_50)) << "and the gate must be what prevents it";
}

// A panda that stops reporting is a panda that is not passing torque either, so an aged-out report
// closes the gate — unlike never having heard from one at all.
TEST(LaneKeepAssistGate, AnAgedOutReportClosesTheGate)
{
  AssistRig r(/*require_assist=*/true, /*max_age_s=*/0.2);
  r.health(true);
  r.tick();
  ASSERT_TRUE(r.sink->last_enabled);

  for (int i = 0; i < 20; ++i)  // 400 ms of ticks with no further health message
    r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "the last panda report aged out";
  EXPECT_FALSE(r.lk->last().dbg.assist_known);

  r.health(true);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "a fresh report reopens it";
}

// The knob is only worth having if the shipped config turns it on: the whole point is that no future
// measurement quietly mixes actuated and non-actuated frames again.
TEST(ShippedConfig, TheAssistGateIsOn)
{
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(ADAS_SHIPPED_CONFIG_JSON, &ok);
  ASSERT_TRUE(ok);
  EXPECT_TRUE(cfg.lane_keep.lat_require_assist);
  EXPECT_GT(cfg.lane_keep.assist_max_age_s, 0.2) << "below a couple of panda periods the gate would flap";
  EXPECT_LE(cfg.lane_keep.assist_max_age_s, 1.0);
}

// The gate must follow actuation, not `controls_allowed`. Always-on lateral (ALKA) makes the panda pass HCA
// torque with `controls_allowed` false — dragonpilot did exactly that in 64.3 % of a drive on this car — so a
// gate keyed on `controls_allowed` would withdraw the command precisely when the assist starts working.
TEST(LaneKeepAssistGate, FollowsActuationRatherThanControlsAllowed)
{
  AssistRig r;
  r.health(/*lat_allowed=*/true, /*controls_allowed_field=*/false);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "torque reaches the rack — steer, whatever controls_allowed says";
  EXPECT_TRUE(r.lk->last().dbg.assist_allowed);

  r.health(/*lat_allowed=*/false, /*controls_allowed_field=*/true);
  r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "controls_allowed alone must not reopen the gate";
}

// Reporting and gating are separate on purpose: with the gate off the recorded flag is still what lets
// offline analysis stop averaging actuated and non-actuated frames together.
TEST(LaneKeepAssistGate, CanBeSwitchedOffAndStillReports)
{
  AssistRig r(/*require_assist=*/false);
  r.health(false);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "gate off — prior behavior";
  EXPECT_TRUE(r.lk->last().dbg.assist_known);
  EXPECT_FALSE(r.lk->last().dbg.assist_allowed) << "but the truth is still recorded";
}

// Recomputing the setpoint between vision frames. The property that makes it safe to call at any rate is
// that `lagAdjustedCurvature` clamps against the *plan's* own kappa0, not against the previous output — so
// 100 Hz call site becomes a slow drift generator.
TEST(SetpointRecompute, RepeatedCallsAtOneSpeedDoNotRatchet)
{
  adas::flowpilot::LateralMpc mpc;
  EXPECT_FALSE(mpc.curvatureAtSpeed(14.0, 0.05).has_value()) << "nothing solved yet — nothing to recompute";

  std::vector<Vec2> curve;
  for (int i = 2; i <= 40; ++i) {
    const double x = static_cast<double>(i);
    curve.emplace_back(x, 0.004 * x * x);
  }
  const auto sol = mpc.update(14.0, 0.0, 2.67, curve, {}, {}, {}, 0.05);
  ASSERT_TRUE(sol.ok);

  const auto first = mpc.curvatureAtSpeed(14.0, 0.05);
  ASSERT_TRUE(first.has_value());
  for (int i = 0; i < 50; ++i) {
    const auto again = mpc.curvatureAtSpeed(14.0, 0.05);
    ASSERT_TRUE(again.has_value());
    EXPECT_DOUBLE_EQ(*again, *first) << "call " << i;
  }
  EXPECT_NEAR(*first, sol.desired_curvature, 1e-12) << "same speed as the solve must reproduce the solve";
}

TEST(SetpointRecompute, SpeedIsWhatMovesIt)
{
  adas::flowpilot::LateralMpc mpc;
  std::vector<Vec2> curve;
  for (int i = 2; i <= 40; ++i) {
    const double x = static_cast<double>(i);
    curve.emplace_back(x, 0.004 * x * x);
  }
  ASSERT_TRUE(mpc.update(14.0, 0.0, 2.67, curve, {}, {}, {}, 0.05).ok);

  const auto at14 = mpc.curvatureAtSpeed(14.0, 0.05);
  const auto at20 = mpc.curvatureAtSpeed(20.0, 0.05);
  ASSERT_TRUE(at14 && at20);
  EXPECT_NE(*at14, *at20) << "the whole point is that the setpoint follows the current speed between frames";

  mpc.reset();
  EXPECT_FALSE(mpc.curvatureAtSpeed(14.0, 0.05).has_value()) << "reset must forget the trajectory";
}

TEST(ShippedConfig, AtMostOneCommandChangeIsEnabledAtATime)
{
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(ADAS_SHIPPED_CONFIG_JSON, &ok);
  ASSERT_TRUE(ok);

  const std::vector<std::pair<const char*, bool>> command = {
      {"lat_recompute_setpoint", cfg.lane_keep.lat_recompute_setpoint},
      {"use_learned_params", cfg.lane_keep.use_learned_params},
      {"lane_mode_hysteresis", cfg.lane_keep.lane_path.lane_mode_hysteresis},
      {"roll_compensation", cfg.lane_keep.roll_compensation},
  };
  std::string on;
  for (const auto& [name, v] : command)
    if (v)
      on += std::string(on.empty() ? "" : ", ") + name;
  const size_t count =
      static_cast<size_t>(std::count_if(command.begin(), command.end(), [](const auto& kv) { return kv.second; }));
  if (cfg.lane_keep.dp_parity_pack) {
    EXPECT_GT(count, 1u) << "dp_parity_pack объявлен, а пакета нет — флаг лишний";
    GTEST_SKIP() << "объявленный пакет: " << on << " — разделить их этим заездом нельзя, и это принято";
  }
  EXPECT_LE(count, 1u) << "enabled together: " << on
                       << " — both change the command for the same reference, so the bag cannot separate them";
}

TEST(SetpointRecompute, TheReportedSetpointFollowsTheRecompute)
{
  auto steerAfterSpeedChange = [](bool recompute) {
    LaneKeep::Config c = mpcConfig();
    c.controller = "fp";
    c.steer_output_enabled = true;
    c.min_control_speed_mps = 0.0;
    c.min_control_speed_hyst_mps = 0.0;
    c.lane_max_age_s = 0.0;
    c.lka_suppress_on_blinker = false;
    c.lat_require_assist = false;
    c.lat_recompute_setpoint = recompute;

    adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
    auto lk = mw.registerService<LaneKeep>(c);
    int64_t t = 10'000'000;
    mw.setTime(t);

    std::vector<Vec2> curve;
    for (int i = 2; i <= 40; ++i) {
      const double x = static_cast<double>(i);
      curve.emplace_back(x, 0.004 * x * x);
    }
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(curve, t));
    adas::ChassisSample ch;
    ch.timestamp_us = t;
    ch.speed_mps = 12.0;
    mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
    mw.step();
    const double at_frame = lk->last().steer_rad;

    t += 20'000;
    mw.setTime(t);
    ch.timestamp_us = t;
    ch.speed_mps = 24.0;
    mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
    mw.step();
    return std::make_pair(at_frame, lk->last().steer_rad);
  };

  const auto off = steerAfterSpeedChange(false);
  EXPECT_DOUBLE_EQ(off.first, off.second) << "with the recompute off the setpoint is held, as it always was";

  const auto on = steerAfterSpeedChange(true);
  EXPECT_NE(on.first, on.second) << "with it on the reported setpoint must move, not just the internal one";
}

#include "adas/lateral/acados_lat_mpc.hpp"

TEST(AcadosLatMpc, LoadsAndSolves)
{
  adas::flowpilot::AcadosLatMpc mpc;
  ASSERT_TRUE(mpc.available()) << "библиотека не загрузилась — проверьте scripts/vendor_acados.py";
  EXPECT_EQ(adas::flowpilot::AcadosLatMpc::horizonNodes(), 16);
  EXPECT_NEAR(adas::flowpilot::AcadosLatMpc::nodeTime(16), 2.5, 1e-9) << "тот же горизонт, что у нашего решателя";
}

namespace {
struct ArcRefs {
  std::vector<double> y, psi, r;
};

ArcRefs arcRefs(double kappa, double v)
{
  const int n = adas::flowpilot::AcadosLatMpc::horizonNodes();
  ArcRefs a;
  for (int i = 0; i <= n; ++i) {
    const double s = v * adas::flowpilot::AcadosLatMpc::nodeTime(i);
    a.y.push_back(0.5 * kappa * s * s);
    a.psi.push_back(kappa * s);
    a.r.push_back(kappa * v);
  }
  return a;
}

}  // namespace

TEST(AcadosLatMpc, DISABLED_Bisect)
{
  adas::flowpilot::AcadosLatMpc mpc;
  ASSERT_TRUE(mpc.available());
  std::printf("создан\n");
  std::fflush(stdout);
  mpc.setWeights({});
  std::printf("веса заданы\n");
  std::fflush(stdout);
  const auto refs = arcRefs(0.01, 14.0);
  auto r = mpc.solve(14.0, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.2);
  std::printf("solve без reset: ok=%d status=%d kappa=%.5f\n", (int)r.ok, r.status, r.desired_curvature);
  std::fflush(stdout);
  mpc.reset();
  std::printf("reset прошёл\n");
  std::fflush(stdout);
  r = mpc.solve(14.0, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.2);
  std::printf("solve после reset: ok=%d status=%d kappa=%.5f\n", (int)r.ok, r.status, r.desired_curvature);
}

TEST(AcadosLatMpc, FollowsTheArcItIsGiven)
{
  adas::flowpilot::AcadosLatMpc mpc;
  ASSERT_TRUE(mpc.available());
  mpc.setWeights({});
  mpc.reset();

  const double v = 14.0, kappa = 0.01;
  const auto refs = arcRefs(kappa, v);
  adas::flowpilot::AcadosLatMpc::Result r;
  for (int i = 0; i < 5; ++i)
    r = mpc.solve(v, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.2);

  ASSERT_TRUE(r.ok) << "статус решателя " << r.status;
  EXPECT_GT(r.desired_curvature, 0.0) << "дуга влево — и кривизна влево";
  EXPECT_NEAR(r.desired_curvature, kappa, 0.5 * kappa) << "и по величине это та же дуга";

  const auto mirrored = arcRefs(-kappa, v);
  for (int i = 0; i < 5; ++i)
    r = mpc.solve(v, 0.0, 0.0, mirrored.y, mirrored.psi, mirrored.r, 0.2);
  EXPECT_LT(r.desired_curvature, 0.0) << "зеркальная дуга — зеркальный знак";
}

TEST(AcadosLatMpc, AgreesWithOurOwnSolverOnTheSameProblem)
{
  const double v = 14.0, kappa = 0.008;
  adas::flowpilot::AcadosLatMpc acados;
  ASSERT_TRUE(acados.available());
  acados.setWeights({});
  acados.reset();
  const auto refs = arcRefs(kappa, v);
  adas::flowpilot::AcadosLatMpc::Result ra;
  for (int i = 0; i < 8; ++i)
    ra = acados.solve(v, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.35);
  ASSERT_TRUE(ra.ok);

  std::vector<Vec2> arc;
  for (int i = 2; i <= 80; ++i) {
    const double x = static_cast<double>(i);
    arc.emplace_back(x, -0.5 * kappa * x * x);  // тот же поворот, но в device-кадре: y вправо
  }
  adas::flowpilot::LateralMpc ours;
  adas::flowpilot::LatMpcResult ro;
  for (int i = 0; i < 8; ++i)
    ro = ours.update(v, 0.0, 2.67, arc, {}, {}, {}, 0.05);
  ASSERT_TRUE(ro.ok);

  EXPECT_GT(ra.desired_curvature * ro.desired_curvature, 0.0)
      << "решатели обязаны хотеть одну сторону: acados " << ra.desired_curvature << ", наш " << ro.desired_curvature;
  EXPECT_NEAR(ra.desired_curvature, ro.desired_curvature, 0.5 * std::abs(kappa)) << "и близкую величину — постановка "
                                                                                    "задачи одна и та же";
}
