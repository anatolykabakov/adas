#include <gtest/gtest.h>

#include "adas/lateral/convert.h"
#include "adas/lateral/vp_planner.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/planner.h"
#include "adas/utils/lane_path.h"
#include "adas/utils/proto_convert.h"

using adas::services::Planner;

namespace {
std::vector<adas::Vec2> curveAt(int step)
{
  std::vector<adas::Vec2> poly;
  const double bend = 0.001 * static_cast<double>(step);
  const double offset = 0.3 * std::sin(0.4 * static_cast<double>(step));
  for (int i = 0; i <= 40; ++i) {
    const double x = static_cast<double>(i);
    poly.emplace_back(x, offset + bend * x * x);
  }
  return poly;
}

adas::proto::LaneLines lanesFrom(const std::vector<adas::Vec2>& poly, int64_t ts_us)
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

}  // namespace

TEST(VpPlannerWiring, ServiceRunsThePlannerItAdvertises)
{
  Planner::Config c;
  c.controller = "mpc";
  c.min_control_speed_mps = 0.0;
  c.min_control_speed_hyst_mps = 0.0;
  c.lane_max_age_s = 0.0;
  c.mpc_cte_ema_alpha = 0.5;
  c.mpc_epsi_ema_alpha = 0.7;
  c.mpc_kappa_ema_alpha = 0.4;
  c.mpc_kappa_yaw_blend = 0.3;
  c.mpc_kappa_yaw_min_speed = 3.0;

  adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
  auto lk = mw.registerService<Planner>(c);

  const double speed = 14.0;
  const double yaw_rate = static_cast<float>(0.02);
  const int64_t dt_us = static_cast<int64_t>(c.vision_nominal_dt_s * 1e6);

  adas::lateral::VpPlanner planner(lk->vpPlannerConfig());
  adas::LaneFusionState fusion;

  int64_t t = 10'000'000;
  for (int step = 0; step < 25; ++step) {
    t += dt_us;
    mw.setTime(t);

    const auto poly = curveAt(step);
    const auto ll = lanesFrom(poly, t);

    adas::proto::CarState cs;
    cs.set_timestamp(t / 1000);
    cs.set_v_ego(speed);
    cs.set_yaw_rate(yaw_rate);
    mw.publish(adas::topics::kVehicleState, cs);
    mw.publish(adas::topics::kVisionLanes, ll);
    mw.step();

    const auto path = adas::laneLinesToPath(ll, c.lane_path, &fusion, speed);
    const auto in = adas::lateral::inputFromMessages(path, speed, yaw_rate, true, c.vision_nominal_dt_s, c.cam_y_left_m,
                                                     adas::lateral::VehicleParams{});
    const auto out = planner.update(in);

    ASSERT_EQ("ok", lk->last().status) << "step " << step;
    EXPECT_NEAR(lk->last().steer_rad, out.steer_rad, 1e-12) << "step " << step;
    EXPECT_NEAR(lk->last().curvature, out.curvature, 1e-12) << "step " << step;
    EXPECT_NEAR(lk->last().cte_m, out.cte_m, 1e-12) << "step " << step;
    EXPECT_NEAR(lk->last().epsi_rad, out.epsi_rad, 1e-12) << "step " << step;
  }
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

  lk->setMpcEmaAlphas(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(0.5, lk->vpPlannerConfig().kappa_ema_alpha);
  EXPECT_DOUBLE_EQ(0.6, lk->vpPlannerConfig().epsi_ema_alpha);
  EXPECT_DOUBLE_EQ(0.7, lk->vpPlannerConfig().cte_ema_alpha);

  lk->setPurePursuit(0.9, 4.0, 21.0, 1.7);
  const auto pp = lk->ppPlannerConfig();
  EXPECT_DOUBLE_EQ(0.9, pp.k_dd);
  EXPECT_DOUBLE_EQ(4.0, pp.ld_min);
  EXPECT_DOUBLE_EQ(21.0, pp.ld_max);
  EXPECT_DOUBLE_EQ(1.7, pp.waypoint_shift);
}
