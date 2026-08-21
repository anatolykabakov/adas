#include <cmath>
#include <fstream>
#include <vector>

#include <json/json.h>

#include <gtest/gtest.h>

#include "adas/lateral/acados_lat_mpc.h"
#include "adas/lateral/flowpilot_mpc.h"
#include "adas/adas_app.h"
#include "messages.pb.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/planner.h"
#include "adas/services/control.h"
#include "adas/utils/path_lateral_state.h"
#include "adas/utils/proto_convert.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"

using adas::estimatePathLateralState;
using adas::PathLateralState;
using adas::Vec2;
using adas::services::Planner;

namespace {
std::vector<Vec2> straightLaneRightOffset(double offset_m)
{
  std::vector<Vec2> poly;
  for (int i = 2; i <= 40; ++i)
    poly.emplace_back(static_cast<double>(i), offset_m);
  return poly;
}

// Feeding kVisionPath would test the service against a message it does not subscribe to.
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

Planner::Config fpConfig()
{
  Planner::Config c;
  c.controller = "fp";
  c.fp_solver = "grad";  // детерминированный решатель: тесты меряют сервисные клампы, не капсулу
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

TEST(LaneKeepServiceFp, LowSpeedClampNearPpLimit)
{
  Planner::Config c = fpConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  Planner svc(c);
  const auto poly = straightLaneRightOffset(1.5);
  const auto out = svc.step(1.0, poly);
  ASSERT_EQ(out.status, "ok");

  EXPECT_LE(std::abs(out.steer_rad), 9.0 * M_PI / 180.0 + 1e-6);
}

TEST(LaneKeepServiceFp, FrameDtComesFromMessageTimestamps)
{
  Planner::Config c = fpConfig();
  Planner svc(c);

  auto frame = [](double offset_m, int64_t ts_us) {
    adas::LanePathMsg m;
    m.polyline = straightLaneRightOffset(offset_m);
    m.capture_ts_us = ts_us;
    m.timestamp_us = ts_us;
    return m;
  };

  auto flip_step_deg = [&](int64_t period_us) {
    Planner s(c);
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

TEST(LaneKeepServiceFp, RateLimitBoundsFrameToFrameStep)
{
  Planner::Config c = fpConfig();
  Planner svc(c);

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

TEST(Planner, CamYLeftShiftsPathToVehicleFrame)
{
  Planner::Config c = fpConfig();
  c.cam_y_left_m = 0.5;
  Planner svc(c);

  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, 0.0, 0.05);
  EXPECT_NEAR(out.steer_rad, 0.0, 0.05);
}

TEST(Planner, CamYLeftZeroLeavesCteUnchanged)
{
  Planner::Config c = fpConfig();
  EXPECT_DOUBLE_EQ(c.cam_y_left_m, 0.0);
  Planner svc(c);
  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, -0.5, 0.05);
}

TEST(LaneKeepServicePublish, SlewGuardConfigDefaultActive)
{
  Planner::Config c;
  EXPECT_GT(c.steer_slew_limit_deg, 0.0);
  EXPECT_LE(c.steer_slew_limit_deg, 25.0);
}

TEST(LaneKeepServiceFp, StraightModerateCteDoesNotRailAtSpeed)
{
  Planner svc(fpConfig());

  const auto slow = svc.step(4.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(slow.status, "ok");
  const auto fast = svc.step(20.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(fast.status, "ok");

  EXPECT_LT(std::abs(slow.steer_rad), 0.20);
  EXPECT_LT(std::abs(fast.steer_rad), 0.20);

  EXPECT_LT(std::abs(fast.steer_rad), std::abs(slow.steer_rad));
}

TEST(LaneKeepServiceFp, RateLimitCeilingHoldsAtLowSpeed)
{
  Planner::Config c = fpConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  Planner svc(c);
  const auto o1 = svc.step(1.0, straightLaneRightOffset(0.8));
  ASSERT_EQ(o1.status, "ok");
  const auto o2 = svc.step(1.0, straightLaneRightOffset(-0.8));
  ASSERT_EQ(o2.status, "ok");
  const double step_deg = std::abs(o2.steer_rad - o1.steer_rad) * 180.0 / M_PI;
  EXPECT_LE(step_deg, c.mpc_rate_limit_deg + 1e-3);
}

TEST(LaneKeepServiceFlowpilot, StraightOffsetSteersTowardCenter)
{
  Planner::Config c = fpConfig();
  c.controller = "fp";
  Planner svc(c);

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
  Planner::Config c = fpConfig();
  c.controller = "pp";
  Planner svc(c);
  svc.setController("fp");
  EXPECT_TRUE(svc.useFlowpilot());
  const auto out = svc.step(12.0, straightLaneRightOffset(0.0));
  ASSERT_EQ(out.status, "ok");
  EXPECT_EQ(out.controller, "fp");
}

TEST(LaneKeepServiceFlowpilot, CurvedPathCommandsNonZero)
{
  Planner::Config c = fpConfig();
  c.controller = "fp";
  Planner svc(c);
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

TEST(LaneKeepServiceFp, LowSpeedGateHoldsZeroAndHasHysteresis)
{
  Planner::Config c = fpConfig();
  c.controller = "fp";
  Planner svc(c);
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
/** A controller built from the planner's config. The gates and the PID moved into it, so a test about
 *  withdrawing the command has to bring up both services: the planner emits curvature, `Control`
 *  turns it into a command. */
adas::services::Control::Config controlFor(const Planner::Config& lk)
{
  adas::services::Control::Config c;
  c.ctl = {lk.pid_kp,      lk.pid_ki,     lk.pid_kf,        lk.pid_ff_floor_mps,     100.0,
           lk.steer_ratio, lk.steer_sign, lk.max_steer_deg, lk.tire_stiffness_factor};
  c.slew = {lk.steer_slew_limit_deg, lk.mpc_max_lateral_jerk, lk.mpc_rate_min_speed,
            lk.mpc_Lf > 1e-3 ? lk.mpc_Lf : lk.wheelbase_m, 10};
  c.wheelbase_m = lk.wheelbase_m;
  c.lf_m = lk.mpc_Lf > 1e-3 ? lk.mpc_Lf : lk.wheelbase_m;
  c.max_torque_cnm = lk.max_torque_cnm;
  c.lane_max_age_s = lk.lane_max_age_s;
  c.lka_blinker_resume_delay_s = lk.lka_blinker_resume_delay_s;
  c.assist_max_age_s = lk.assist_max_age_s;
  return c;
}

class SteerSink : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "steer_sink"; }

  void configure() override
  {
    subscribe<adas::proto::SteerCommand>(adas::topics::kSteerCommand, [this](const adas::proto::SteerCommand& m) {
      last_enabled = m.enabled();
      last_torque = m.torque_cnm();
      last_assist_allowed = m.assist_allowed();
      last_assist_known = m.assist_known();
      last_status = m.status();
      last_pid_i = m.pid_i();
      ++count;
    });
  }

  bool last_enabled = false;
  int last_torque = 0;
  // The gates and their status live in the controller now; the planner knows nothing about them, which
  // is why they are checked here.
  bool last_assist_allowed = false;
  bool last_assist_known = false;
  std::string last_status;
  double last_pid_i = 0.0;
  int count = 0;
};

}  // namespace

TEST(LaneKeepStaleGate, OldReferenceDisablesTheCommand)
{
  Planner::Config c = fpConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.30;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  auto lk = mw.registerService<Planner>(c);
  mw.registerService<adas::services::Control>(controlFor(c));
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
    // Virtual time advances on every publish: the controller's tick is fixed, and without moving the
    // clock the command is never recomputed. Two steps: the first delivers the plan and fires the
    // tick, the second delivers the command to the subscriber.
    mw.setTime(mw.now() + 20'000);
    mw.step();
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
  Planner::Config c = fpConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  mw.registerService<Planner>(c);
  mw.registerService<adas::services::Control>(controlFor(c));
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  mw.setTime(10'000'000);
  const int64_t old_us = 10'000'000 - 5'000'000;
  mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), old_us));
  adas::ChassisSample ch;
  ch.timestamp_us = old_us;
  ch.speed_mps = 14.0;
  mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
  // The controller's tick is fixed: without advancing virtual time it never fires, and the command
  // reaches the subscriber only on the next step — there are three services now.
  mw.setTime(mw.now() + 20'000);
  mw.step();
  mw.step();

  EXPECT_TRUE(sink->last_enabled) << "with gate off, prior behavior";
}

// Turn signal hands the wheel back: we have no lane-change planner, so holding the current lane
// while the driver crosses the line fights them. Signal presence verified on run
// 2026_08_04_21_00_18 (left 7 episodes, right 11).
TEST(LaneKeepBlinkerGate, TurnSignalClearsTheCommandAndResumesAfterDelay)
{
  Planner::Config c = fpConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;  // staleness gate off — this test is about the blinker only
  c.lka_blinker_resume_delay_s = 0.2;

  auto sink = std::make_shared<SteerSink>();
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  mw.registerService<Planner>(c);
  mw.registerService<adas::services::Control>(controlFor(c));
  mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));

  // Time is driven explicitly: in simulated mode `now()` returns sim_us_, and without setTime the clock
  // stands at zero. Sleeping on the wall clock did not exercise the resume delay at all — the test
  // passed through a stuck status rather than through the timer.
  uint64_t sim_us = 1'000'000;
  auto feed = [&](bool left, bool right) {
    // Time advances on every publish, or the controller's tick never fires: it is fixed, whereas the
    // gate used to be evaluated at the same instant as the plan and the clock could stand still.
    sim_us += 20'000;
    mw.setTime(sim_us);
    const int64_t ts = static_cast<int64_t>(sim_us);
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), ts));

    adas::ChassisSample ch;
    ch.timestamp_us = ts;
    ch.speed_mps = 14.0;
    ch.left_blinker = left;
    ch.right_blinker = right;
    mw.publish(adas::topics::kVehicleState, adas::carStateFromChassis(ch));
    // Two steps: the first delivers the plan and fires the tick, the second delivers the command.
    mw.step();
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

namespace {
struct AssistRig {
  explicit AssistRig(double max_age_s = 0.5)
    : sink(std::make_shared<SteerSink>()), mw(adas::middleware::Manager::Mode::Simulated)
  {
    Planner::Config c = fpConfig();
    c.min_control_speed_mps = 0.0;
    c.min_control_speed_hyst_mps = 0.0;
    c.lane_max_age_s = 0.0;
    c.assist_max_age_s = max_age_s;
    lk = mw.registerService<Planner>(c);
    mw.registerService<adas::services::Control>(controlFor(c));
    mw.registerService(std::static_pointer_cast<adas::middleware::Service>(sink));
    mw.setTime(now_us);
  }

  void health(bool lat_allowed, bool controls_allowed_field = false)
  {
    health_t raw{};
    raw.controls_allowed_pkt = controls_allowed_field ? 1 : 0;
    raw.safety_mode_pkt = volkswagen::MqbSafetyConstants::kVolkswagen;
    mw.publish(adas::topics::kPandaHealth,
               utils::createHealthMessage(raw, static_cast<int64_t>(mw.now() / 1000), /*ignition=*/true, lat_allowed));
  }

  void tick()
  {
    now_us += 20'000;
    mw.setTime(now_us);
    // The bus carries schema messages, so the test publishes exactly what the transport would.
    mw.publish(adas::topics::kVisionLanes, lanesFromPolyline(straightLaneRightOffset(0.4), now_us));

    adas::proto::CarState cs;
    cs.set_timestamp(now_us / 1000);
    cs.set_v_ego(14.0);
    cs.set_steering_angle_deg(0.0);
    mw.publish(adas::topics::kVehicleState, cs);
    // Virtual time advances on every publish: the controller tick is fixed.
    mw.step();
    mw.step();
  }

  std::shared_ptr<SteerSink> sink;
  adas::middleware::Manager mw;
  std::shared_ptr<Planner> lk;
  int64_t now_us = 10'000'000;
};

}  // namespace

// Having never heard from a panda must not gate: that is a bag replay, the Python bindings or a bench

TEST(LaneKeepAssistGate, NoPandaReportMeansNoGate)
{
  AssistRig r;
  r.tick();
  ASSERT_GT(r.sink->count, 0) << "controls/steer not published at all";
  EXPECT_TRUE(r.sink->last_enabled) << "no panda in the loop — the offline harness must still steer";
  EXPECT_FALSE(r.sink->last_assist_known) << "and it must say it does not know";
  EXPECT_FALSE(r.sink->last_assist_allowed);
}

TEST(LaneKeepAssistGate, WithheldTorqueClearsTheCommandAndComesBack)
{
  AssistRig r;
  r.health(true);
  r.tick();
  ASSERT_TRUE(r.sink->last_enabled) << "assist present — we steer";
  EXPECT_TRUE(r.sink->last_assist_known);
  EXPECT_TRUE(r.sink->last_assist_allowed);

  r.health(false);
  r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "panda is not passing torque — do not pretend to steer";
  EXPECT_EQ(0, r.sink->last_torque);
  EXPECT_EQ("no_assist", r.sink->last_status);
  EXPECT_FALSE(r.sink->last_assist_allowed);
  EXPECT_TRUE(r.sink->last_assist_known) << "absent is not the same as unknown";

  r.health(true);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "assist back — steering again";
  EXPECT_EQ("ok", r.sink->last_status) << "the status must clear, not only be set";
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
  auto resumedTorque = [](int withheld_ticks) {
    AssistRig r;
    r.health(false);
    for (int i = 0; i < withheld_ticks; ++i)
      r.tick();
    EXPECT_EQ(0, r.sink->last_torque) << "nothing should be commanded while the torque is withheld";
    r.health(true);
    r.tick();
    return r.sink->last_torque;
  };

  const int gated_5 = resumedTorque(5);
  const int gated_50 = resumedTorque(50);
  EXPECT_NE(0, gated_5) << "the comparison is vacuous if the resumed command is zero";
  EXPECT_EQ(gated_5, gated_50) << "torque on resumption grew with the time spent withheld — windup";
}

// A panda that stops reporting is a panda that is not passing torque either, so an aged-out report
// closes the gate — unlike never having heard from one at all.
TEST(LaneKeepAssistGate, AnAgedOutReportClosesTheGate)
{
  AssistRig r(/*max_age_s=*/0.2);
  r.health(true);
  r.tick();
  ASSERT_TRUE(r.sink->last_enabled);

  for (int i = 0; i < 20; ++i)  // 400 ms of ticks with no further health message
    r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "the last panda report aged out";
  EXPECT_FALSE(r.sink->last_assist_known);

  r.health(true);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "a fresh report reopens it";
}
TEST(LaneKeepAssistGate, FollowsActuationRatherThanControlsAllowed)
{
  AssistRig r;
  r.health(/*lat_allowed=*/true, /*controls_allowed_field=*/false);
  r.tick();
  EXPECT_TRUE(r.sink->last_enabled) << "torque reaches the rack — steer, whatever controls_allowed says";
  EXPECT_TRUE(r.sink->last_assist_allowed);

  r.health(/*lat_allowed=*/false, /*controls_allowed_field=*/true);
  r.tick();
  EXPECT_FALSE(r.sink->last_enabled) << "controls_allowed alone must not reopen the gate";
}
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

TEST(AcadosLatMpc, LoadsAndSolves)
{
  adas::flowpilot::AcadosLatMpc mpc;
  ASSERT_TRUE(mpc.available()) << "the library did not load — check scripts/vendor_acados.py";
  EXPECT_EQ(adas::flowpilot::AcadosLatMpc::horizonNodes(), 16);
  EXPECT_NEAR(adas::flowpilot::AcadosLatMpc::nodeTime(16), 2.5, 1e-9) << "the same horizon as our own solver";
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
  std::printf("created\n");
  std::fflush(stdout);
  mpc.setWeights({});
  std::printf("weights set\n");
  std::fflush(stdout);
  const auto refs = arcRefs(0.01, 14.0);
  auto r = mpc.solve(14.0, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.2);
  std::printf("solve without reset: ok=%d status=%d kappa=%.5f\n", (int)r.ok, r.status, r.desired_curvature);
  std::fflush(stdout);
  mpc.reset();
  std::printf("reset done\n");
  std::fflush(stdout);
  r = mpc.solve(14.0, 0.0, 0.0, refs.y, refs.psi, refs.r, 0.2);
  std::printf("solve after reset: ok=%d status=%d kappa=%.5f\n", (int)r.ok, r.status, r.desired_curvature);
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

  ASSERT_TRUE(r.ok) << "solver status " << r.status;
  EXPECT_GT(r.desired_curvature, 0.0) << "an arc to the left gives curvature to the left";
  EXPECT_NEAR(r.desired_curvature, kappa, 0.5 * kappa) << "and in magnitude it is the same arc";

  const auto mirrored = arcRefs(-kappa, v);
  for (int i = 0; i < 5; ++i)
    r = mpc.solve(v, 0.0, 0.0, mirrored.y, mirrored.psi, mirrored.r, 0.2);
  EXPECT_LT(r.desired_curvature, 0.0) << "a mirrored arc gives a mirrored sign";
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
    arc.emplace_back(x, -0.5 * kappa * x * x);  // the same turn, in the device frame: y to the right
  }
  adas::flowpilot::LateralMpc ours;
  adas::flowpilot::LatMpcResult ro;
  for (int i = 0; i < 8; ++i)
    ro = ours.update(v, 0.0, 2.67, arc, {}, {}, {}, 0.05);
  ASSERT_TRUE(ro.ok);

  EXPECT_GT(ra.desired_curvature * ro.desired_curvature, 0.0)
      << "the solvers must want the same side: acados " << ra.desired_curvature << ", ours " << ro.desired_curvature;
  EXPECT_NEAR(ra.desired_curvature, ro.desired_curvature, 0.5 * std::abs(kappa)) << "and a close magnitude — the "
                                                                                    "problem "
                                                                                    "is the same one";
}

TEST(LaneKeepTuning, RuntimeChangesReachThePlanner)
{
  Planner::Config c;
  c.controller = "fp";
  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  auto lk = mw.registerService<Planner>(c);

  lk->setFpSteerDelayS(0.5);
  EXPECT_DOUBLE_EQ(0.5, lk->fpPlannerConfig().steer_delay_s);

  lk->setFpSteeringRateWeight(123.0);
  EXPECT_DOUBLE_EQ(123.0, lk->fpPlannerConfig().steering_rate_weight);

  lk->setSteerSlewLimitDeg(3.0);
  EXPECT_DOUBLE_EQ(3.0, lk->fpPlannerConfig().steer_slew_limit_deg);

  lk->setMaxSteerDeg(11.0);
  EXPECT_NEAR(11.0 * M_PI / 180.0, lk->ppPlannerConfig().max_steer_rad, 1e-12);

  lk->setPurePursuit(0.9, 4.0, 21.0, 1.7);
  const auto pp = lk->ppPlannerConfig();
  EXPECT_DOUBLE_EQ(0.9, pp.k_dd);
  EXPECT_DOUBLE_EQ(4.0, pp.ld_min);
  EXPECT_DOUBLE_EQ(21.0, pp.ld_max);
  EXPECT_DOUBLE_EQ(1.7, pp.waypoint_shift);
}
