#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "adas/mapmatch/road_route.h"

using adas::mapmatch::curvatureAlong;
using adas::mapmatch::headingProfile;
using adas::mapmatch::resamplePolyline;
using adas::mapmatch::RouteConfig;
using adas::mapmatch::turnSections;

namespace {
/** Arc of radius R sampled every `step` metres of arc length, starting at the origin heading east.
 *  `sign` = +1 turns left. */
void arc(double radius, double length, double step, int sign, std::vector<double>& xs, std::vector<double>& ys,
         double x0 = 0.0, double y0 = 0.0, double theta0 = 0.0)
{
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    const double th = theta0 + sign * s / radius;
    const double cx = x0 - sign * radius * std::sin(theta0);
    const double cy = y0 + sign * radius * std::cos(theta0);
    xs.push_back(cx + sign * radius * std::sin(th));
    ys.push_back(cy - sign * radius * std::cos(th));
  }
}

void straight(double length, double step, std::vector<double>& xs, std::vector<double>& ys)
{
  const double x0 = xs.empty() ? 0.0 : xs.back();
  const double y0 = ys.empty() ? 0.0 : ys.back();
  double s = step;
  for (; s < length - 1e-9; s += step) {
    xs.push_back(x0 + s);
    ys.push_back(y0);
  }
  xs.push_back(x0 + length);
  ys.push_back(y0);
}

}  // namespace

// sign must say which way the road bends.
TEST(RoadRoute, CurvatureOfCircleIsOneOverRadius)
{
  std::vector<double> qs;
  for (double s = 0.0; s <= 400.0; s += 5.0)
    qs.push_back(s);

  for (const double radius : {80.0, 200.0, 500.0, 1000.0}) {
    std::vector<double> xs, ys;
    arc(radius, 400.0, 5.0, +1, xs, ys);
    const auto k = curvatureAlong(xs, ys, qs, 25.0);
    ASSERT_GE(k.size(), 20u);
    for (std::size_t i = 5; i + 5 < k.size(); ++i)
      EXPECT_NEAR(k[i], 1.0 / radius, 0.02 / radius) << "radius " << radius << " at i=" << i;
  }

  std::vector<double> xs, ys;
  arc(300.0, 400.0, 5.0, -1, xs, ys);
  const auto k = curvatureAlong(xs, ys, qs, 25.0);
  EXPECT_LT(k[k.size() / 2], 0.0) << "a right turn must give negative curvature";
  EXPECT_NEAR(std::abs(k[k.size() / 2]), 1.0 / 300.0, 0.02 / 300.0);
}

// midpoints makes the answer independent of that.
TEST(RoadRoute, CurvatureIsIndependentOfNodeSpacing)
{
  const double radius = 250.0;
  std::vector<double> qs;
  for (double s = 60.0; s <= 340.0; s += 5.0)
    qs.push_back(s);

  for (const double spacing : {2.0, 10.0, 40.0, 80.0}) {
    std::vector<double> xs, ys;
    arc(radius, 400.0, spacing, +1, xs, ys);
    const auto k = curvatureAlong(xs, ys, qs, 25.0);
    for (std::size_t i = 0; i < k.size(); ++i)
      EXPECT_NEAR(k[i], 1.0 / radius, 0.01 / radius) << "spacing " << spacing << " at s=" << qs[i];
  }
}

// `buildRouteAhead` (map geometry in, resampled grid only as query points) cannot be "simplified" away.
TEST(RoadRoute, ResampledGeometryLosesNodeSpacingIndependence)
{
  const double radius = 250.0;
  std::vector<double> xs, ys;
  arc(radius, 400.0, 40.0, +1, xs, ys);

  std::vector<double> s, x, y;
  resamplePolyline(xs, ys, 5.0, s, x, y);

  const auto from_map = curvatureAlong(xs, ys, s, 25.0);
  const auto from_resampled = curvatureAlong(x, y, s, 25.0);

  double worst_map = 0.0, worst_resampled = 0.0;
  for (std::size_t i = 12; i + 12 < s.size(); ++i) {
    worst_map = std::max(worst_map, std::abs(from_map[i] - 1.0 / radius));
    worst_resampled = std::max(worst_resampled, std::abs(from_resampled[i] - 1.0 / radius));
  }
  EXPECT_LT(worst_map, 0.01 / radius);
  EXPECT_GT(worst_resampled, 0.5 / radius);
}

TEST(RoadRoute, ResampleGivesFixedStepAndKeepsLength)
{
  std::vector<double> xs, ys;
  xs.push_back(0.0);
  ys.push_back(0.0);
  straight(1000.0, 137.0, xs, ys);

  std::vector<double> s, x, y;
  resamplePolyline(xs, ys, 5.0, s, x, y);
  ASSERT_GT(s.size(), 100u);
  EXPECT_NEAR(s.back(), 1000.0, 5.0);
  for (std::size_t i = 1; i < s.size(); ++i) {
    EXPECT_NEAR(s[i] - s[i - 1], 5.0, 1e-6);
    EXPECT_NEAR(std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]), 5.0, 1e-6);
  }
}

// v = sqrt(a_lat / kappa), and not merge an S-bend into one section.
TEST(RoadRoute, TurnSectionsSplitBySignAndCarrySpeed)
{
  std::vector<double> xs, ys;
  xs.push_back(0.0);
  ys.push_back(0.0);
  straight(200.0, 5.0, xs, ys);
  arc(200.0, 200.0, 5.0, +1, xs, ys, xs.back(), ys.back(), 0.0);
  arc(200.0, 200.0, 5.0, -1, xs, ys, xs.back(), ys.back(), 1.0);

  std::vector<double> s, x, y;
  resamplePolyline(xs, ys, 5.0, s, x, y);
  const auto k = curvatureAlong(xs, ys, s, 25.0);

  RouteConfig cfg;
  const auto sections = turnSections(s, k, cfg);
  ASSERT_EQ(sections.size(), 2u);

  EXPECT_EQ(sections[0].sign, +1);
  EXPECT_EQ(sections[1].sign, -1);
  EXPECT_GT(sections[0].start_m, 150.0) << "the first 200 m are straight";
  EXPECT_LT(sections[0].end_m, sections[1].start_m);

  for (const auto& sec : sections) {
    EXPECT_NEAR(sec.kappa, 1.0 / 200.0, 0.15 / 200.0);
    EXPECT_NEAR(sec.speed_mps, std::sqrt(cfg.max_lat_acc / sec.kappa), 0.5);
  }
}

TEST(RoadRoute, StraightRoadHasNoTurnSections)
{
  std::vector<double> xs, ys;
  xs.push_back(0.0);
  ys.push_back(0.0);
  straight(1500.0, 25.0, xs, ys);

  std::vector<double> s, x, y;
  resamplePolyline(xs, ys, 5.0, s, x, y);
  const auto k = curvatureAlong(xs, ys, s, 25.0);
  for (const double v : k)
    EXPECT_LT(std::abs(v), 1e-9);
  EXPECT_TRUE(turnSections(s, k, RouteConfig{}).empty());
}

// bend inherits the speed of the one kink in it.
TEST(RoadRoute, SharpPeakIsSplitOutOfALongGentleCurve)
{
  std::vector<double> xs, ys;
  xs.push_back(0.0);
  ys.push_back(0.0);
  arc(400.0, 400.0, 5.0, +1, xs, ys, 0.0, 0.0, 0.0);
  arc(90.0, 120.0, 5.0, +1, xs, ys, xs.back(), ys.back(), 1.0);
  arc(400.0, 400.0, 5.0, +1, xs, ys, xs.back(), ys.back(), 2.33);

  std::vector<double> s, x, y;
  resamplePolyline(xs, ys, 5.0, s, x, y);
  const auto k = curvatureAlong(xs, ys, s, 25.0);

  const auto sections = turnSections(s, k, RouteConfig{});
  ASSERT_GE(sections.size(), 2u) << "the tight corner must be isolated";

  const auto tight = std::max_element(sections.begin(), sections.end(),
                                      [](const auto& a, const auto& b) { return a.kappa < b.kappa; });
  EXPECT_GT(tight->kappa, 1.0 / 130.0);
  EXPECT_LT(tight->end_m - tight->start_m, 400.0) << "the sharp section must be local, not the whole curve";

  const auto gentle = std::min_element(sections.begin(), sections.end(),
                                       [](const auto& a, const auto& b) { return a.kappa < b.kappa; });
  EXPECT_LT(gentle->kappa, 0.6 * tight->kappa);
}
