#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "messages.pb.h"
#include "middleware/middleware.hpp"
#include "services/lane_keep_service.h"
#include "utils/path_lateral_state.h"
#include "utils/protobuf_utils.h"
#include "visionpilot/lateral_planning.hpp"

using adas::estimatePathLateralState;
using adas::LaneKeepService;
using adas::PathLateralState;
using adas::Vec2;
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

LaneKeepService::Config mpcConfig()
{
  LaneKeepService::Config c;
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
  LateralPlanner mpc;
  const double Lf = 2.67;
  const double kappa = 0.02;
  const double v = 10.0;
  Eigen::Vector3d state;
  state << 0.0, 0.0, kappa;
  const auto ks = build_kappa_schedule(Lf, 0.0, kappa, 0.0);
  Eigen::VectorXd vs(static_cast<int>(visionpilot::N));
  vs.setConstant(v);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);

  const double ff = std::atan(Lf * kappa) + 0.0015 * v * v * kappa;
  EXPECT_NEAR(deltas[1], ff, 0.08);
}

TEST(VisionPilotMpc, CorrectsPositiveCte)
{
  LateralPlanner mpc;
  const double Lf = 2.67;
  Eigen::Vector3d state;
  state << 0.8, 0.0, 0.0;
  const auto ks = build_kappa_schedule(Lf, 0.0, 0.0, 0.0);
  Eigen::VectorXd vs(static_cast<int>(visionpilot::N));
  vs.setConstant(8.0);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);

  EXPECT_NE(deltas[1], 0.0);
  EXPECT_GT(deltas[1], 0.0);
}

TEST(VisionPilotMpc, LowSpeedModerateCteDoesNotSaturate)
{
  LateralPlanner mpc;
  const double Lf = 2.67;
  Eigen::Vector3d state;
  state << 0.6, 0.0, 0.0;
  const auto ks = build_kappa_schedule(Lf, 0.0, 0.0, 0.0);
  Eigen::VectorXd vs(static_cast<int>(visionpilot::N));
  vs.setConstant(1.5);
  const auto deltas = mpc.compute_steering(Lf, state, vs, ks);
  ASSERT_GE(deltas.size(), 2u);
  EXPECT_GT(deltas[1], 0.0);
  EXPECT_LT(std::abs(deltas[1]), 0.40);
}

TEST(LaneKeepServiceMpc, LowSpeedClampNearPpLimit)
{
  LaneKeepService::Config c = mpcConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  LaneKeepService svc(c);
  const auto poly = straightLaneRightOffset(1.5);
  const auto out = svc.step(1.0, poly);
  ASSERT_EQ(out.status, "ok");

  EXPECT_LE(std::abs(out.steer_rad), 9.0 * M_PI / 180.0 + 1e-6);
}

TEST(LaneKeepServiceMpc, FrameDtComesFromMessageTimestamps)
{
  LaneKeepService::Config c = mpcConfig();
  LaneKeepService svc(c);

  auto frame = [](double offset_m, int64_t ts_us) {
    adas::LanePathMsg m;
    m.polyline = straightLaneRightOffset(offset_m);
    m.capture_ts_us = ts_us;
    m.timestamp_us = ts_us;
    return m;
  };

  auto flip_step_deg = [&](int64_t period_us) {
    LaneKeepService s(c);
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
  LaneKeepService::Config c = mpcConfig();
  LaneKeepService svc(c);

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
  LaneKeepService::Config c = mpcConfig();
  EXPECT_DOUBLE_EQ(c.mpc_kappa_yaw_blend, 0.0);
  LaneKeepService svc(c);

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

TEST(LaneKeepService, CamYLeftShiftsPathToVehicleFrame)
{
  LaneKeepService::Config c = mpcConfig();
  c.cam_y_left_m = 0.5;
  LaneKeepService svc(c);

  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, 0.0, 0.05);
  EXPECT_NEAR(out.steer_rad, 0.0, 0.05);
}

TEST(LaneKeepService, CamYLeftZeroLeavesCteUnchanged)
{
  LaneKeepService::Config c = mpcConfig();
  EXPECT_DOUBLE_EQ(c.cam_y_left_m, 0.0);
  LaneKeepService svc(c);
  const auto out = svc.step(10.0, straightLaneRightOffset(0.5));
  ASSERT_EQ(out.status, "ok");
  EXPECT_NEAR(out.cte_m, -0.5, 0.05);
}

TEST(LaneKeepServicePublish, SlewGuardConfigDefaultActive)
{
  LaneKeepService::Config c;
  EXPECT_GT(c.steer_slew_limit_deg, 0.0);
  EXPECT_LE(c.steer_slew_limit_deg, 25.0);
}

TEST(LaneKeepServiceMpc, StraightModerateCteDoesNotRailAtSpeed)
{
  LaneKeepService svc(mpcConfig());

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
  LaneKeepService::Config c = mpcConfig();
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  LaneKeepService svc(c);
  const auto o1 = svc.step(1.0, straightLaneRightOffset(0.8));
  ASSERT_EQ(o1.status, "ok");
  const auto o2 = svc.step(1.0, straightLaneRightOffset(-0.8));
  ASSERT_EQ(o2.status, "ok");
  const double step_deg = std::abs(o2.steer_rad - o1.steer_rad) * 180.0 / M_PI;
  EXPECT_LE(step_deg, c.mpc_rate_limit_deg + 1e-3);
}

TEST(LaneKeepServiceFlowpilot, StraightOffsetSteersTowardCenter)
{
  LaneKeepService::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeepService svc(c);

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
  LaneKeepService::Config c = mpcConfig();
  c.controller = "pp";
  LaneKeepService svc(c);
  svc.setController("fp");
  EXPECT_TRUE(svc.useFlowpilot());
  const auto out = svc.step(12.0, straightLaneRightOffset(0.0));
  ASSERT_EQ(out.status, "ok");
  EXPECT_EQ(out.controller, "fp");
}

TEST(LaneKeepServiceFlowpilot, CurvedPathCommandsNonZero)
{
  LaneKeepService::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeepService svc(c);
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
  LaneKeepService::Config c = mpcConfig();
  c.controller = "fp";
  LaneKeepService svc(c);
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

class SteerSink : public adas::Service {
public:
  std::string_view getName() const override { return "steer_sink"; }

  void configure() override
  {
    subscribe<ai::flow::adas::ZMQMessage>(adas::topics::kSteerCommand, [this](const ai::flow::adas::ZMQMessage& m) {
      if (!m.has_steer_command())
        return;
      last_enabled = m.steer_command().enabled();
      last_torque = m.steer_command().torque_cnm();
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
  LaneKeepService::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.30;

  auto sink = std::make_shared<SteerSink>();
  adas::Middleware mw(adas::Middleware::Mode::Simulated);
  auto lk = mw.registerService<LaneKeepService>(c);
  mw.registerService(std::static_pointer_cast<adas::Service>(sink));

  const int64_t now_us = utils::getCurrentTimestamp() * 1000;

  auto feed = [&](int64_t capture_us) {
    adas::LanePathMsg path;
    path.polyline = straightLaneRightOffset(0.4);
    path.capture_ts_us = capture_us;
    path.timestamp_us = capture_us;
    mw.publish(adas::topics::kVisionPath, path);

    adas::ChassisSample ch;
    ch.timestamp_us = capture_us;
    ch.speed_mps = 14.0;
    ch.steering_angle_deg = 0.0;
    mw.publish(adas::topics::kVehicleChassis, ch);
    mw.step();
  };

  feed(now_us);
  ASSERT_GT(sink->count, 0) << "controls/steer not published at all";
  EXPECT_TRUE(sink->last_enabled);

  feed(now_us - 1'000'000);
  EXPECT_FALSE(sink->last_enabled) << "must not steer on a stale path";
  EXPECT_EQ(0, sink->last_torque);

  feed(utils::getCurrentTimestamp() * 1000);
  EXPECT_TRUE(sink->last_enabled);
}

TEST(LaneKeepStaleGate, ZeroDisablesTheGate)
{
  LaneKeepService::Config c = mpcConfig();
  c.steer_output_enabled = true;
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;

  auto sink = std::make_shared<SteerSink>();
  adas::Middleware mw(adas::Middleware::Mode::Simulated);
  mw.registerService<LaneKeepService>(c);
  mw.registerService(std::static_pointer_cast<adas::Service>(sink));

  const int64_t old_us = utils::getCurrentTimestamp() * 1000 - 5'000'000;
  adas::LanePathMsg path;
  path.polyline = straightLaneRightOffset(0.4);
  path.capture_ts_us = old_us;
  path.timestamp_us = old_us;
  mw.publish(adas::topics::kVisionPath, path);
  adas::ChassisSample ch;
  ch.timestamp_us = old_us;
  ch.speed_mps = 14.0;
  mw.publish(adas::topics::kVehicleChassis, ch);
  mw.step();

  EXPECT_TRUE(sink->last_enabled) << "with gate off, prior behavior";
}
