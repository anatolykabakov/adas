#include <algorithm>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "adas/lateral/pp_planner.h"

namespace {
struct Expected {
  double offset;
  double speed;
  double lookahead_m;
  double steer_rad;
  double target_x;
  double target_y;
};

constexpr Expected kReference[] = {
    {0.0, 5.0, 8.0, 0.0072148978643458341, 6.5995205121311944, 0.087587533315411054},
    {0.0, 14.0, 11.200000000000001, 0.0080833851653819427, 9.7983483776289031, 0.19233723834989855},
    {0.0, 25.0, 20.0, 0.0091139163778121406, 18.588041597330609, 0.69151507820246594},
    {0.4, 5.0, 8.0, 0.040112717247334957, 6.5851501308573734, 0.48721390340229176},
    {0.4, 14.0, 11.200000000000001, 0.024867308084677135, 9.7843536390449408, 0.59180543828370757},
    {0.4, 25.0, 20.0, 0.014367841632780194, 18.570264517232872, 1.0901995742752335},
    {-0.7, 5.0, 8.0, -0.05045397058799083, 6.5764789753176132, -0.6130115466417424},
    {-0.7, 14.0, 11.200000000000001, -0.021348611561186788, 9.7884716253410247, -0.50803807823704139},
    {-0.7, 25.0, 20.0, -0.00010016940806670254, 18.599998555959342, -0.0076001068590085593},
};

}  // namespace

TEST(PpPlannerPort, MatchesTheClassItReplaces)
{
  adas::lateral::PpPlanner::Config cfg;
  cfg.k_dd = 0.8;
  cfg.waypoint_shift = 1.4;
  cfg.ld_min = 8.0;
  cfg.ld_max = 25.0;
  cfg.ld_curv_gain = 0.0;
  cfg.vehicle.wheelbase_m = 2.636;
  cfg.max_steer_rad = 8.0 * M_PI / 180.0;
  adas::lateral::PpPlanner planner(cfg);

  for (const auto& want : kReference) {
    std::vector<adas::Vec2> poly;
    for (int i = 0; i < 40; ++i)
      poly.emplace_back(static_cast<double>(i), want.offset + 0.002 * i * i);

    adas::lateral::Input in;
    in.speed_mps = want.speed;
    in.polyline_ego = poly;
    in.vehicle = cfg.vehicle;

    const auto got = planner.update(in);
    const std::string where = "offset " + std::to_string(want.offset) + " speed " + std::to_string(want.speed);

    EXPECT_NEAR(got.lookahead_m, want.lookahead_m, 1e-12) << where;
    EXPECT_NEAR(got.dbg.pp_steer_raw_rad, want.steer_rad, 1e-12) << where;
    ASSERT_TRUE(got.has_target) << where;
    EXPECT_NEAR(got.target_x, want.target_x, 1e-12) << where;
    EXPECT_NEAR(got.target_y, want.target_y, 1e-12) << where;

    // A constant ceiling, as the service has on pure pursuit: the clip is part of the ported behaviour.
    EXPECT_NEAR(got.steer_rad, std::clamp(want.steer_rad, -cfg.max_steer_rad, cfg.max_steer_rad), 1e-12) << where;
  }
}
