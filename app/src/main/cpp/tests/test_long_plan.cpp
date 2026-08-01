#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "utils/curvature_preview.h"

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
