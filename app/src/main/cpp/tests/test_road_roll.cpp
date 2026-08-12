#include <cmath>

#include <gtest/gtest.h>

#include "adas/utils/road_roll_estimator.h"

namespace {
constexpr double kG = 9.81;
constexpr double kDt = 1.0 / 15.0;

/** Accelerometer y a car would produce: kinematic lateral minus gravity's projection, plus the tilt the
 *  suspension adds. `yaw_rate_can` is ISO / left-positive, as the decoder delivers it. */
double lateralSpecificForce(double speed, double yaw_rate_can, double road_roll_deg, double body_gradient_deg_per_g)
{
  const double a_y = -speed * yaw_rate_can;
  const double body_deg = body_gradient_deg_per_g * (a_y / kG);
  const double tilt_deg = road_roll_deg + body_deg;
  return a_y - kG * std::sin(tilt_deg * M_PI / 180.0);
}

/** Drive a steady corner (or a straight, with yaw_rate 0) long enough for the filter to settle. */
adas::RoadRollEstimator settle(double road_roll_deg, double speed, double yaw_rate_can, double body_gradient = 3.0,
                               adas::RoadRollEstimator::Config cfg = {})
{
  adas::RoadRollEstimator est(cfg);
  const double f_y = lateralSpecificForce(speed, yaw_rate_can, road_roll_deg, body_gradient);
  for (int i = 0; i < 4000; ++i)
    est.update(speed, yaw_rate_can, f_y, kDt);
  return est;
}

}  // namespace

TEST(RoadRoll, FlatStraightRoadReadsZero)
{
  const auto est = settle(0.0, 20.0, 0.0);
  EXPECT_TRUE(est.valid());
  EXPECT_NEAR(est.rollDeg(), 0.0, 0.05);
}

TEST(RoadRoll, CamberOnAStraightIsRecovered)
{
  for (const double truth : {-1.5, -0.8, 0.8, 1.5}) {
    const auto est = settle(truth, 20.0, 0.0);
    EXPECT_NEAR(est.rollDeg(), truth, 0.05) << "truth " << truth;
  }
}

TEST(RoadRoll, BodyRollOnACornerIsSubtractedNotReported)
{
  const double speed = 15.0;
  const double yaw = -0.10;
  const auto est = settle(0.0, speed, yaw, 3.0);
  EXPECT_TRUE(est.valid());
  EXPECT_NEAR(est.rollDeg(), 0.0, 0.1);

  adas::RoadRollEstimator::Config raw;
  raw.body_roll_deg_per_g = 0.0;
  const auto uncorrected = settle(0.0, speed, yaw, 3.0, raw);
  EXPECT_NEAR(uncorrected.rollDeg(), 3.0 * (speed * -yaw) / kG, 0.1);
}

TEST(RoadRoll, TheSignConventionIsTheOneTheDecoderDelivers)
{
  const auto right_turn = settle(1.0, 18.0, -0.09);
  const auto left_turn = settle(1.0, 18.0, +0.09);
  EXPECT_NEAR(right_turn.rollDeg(), 1.0, 0.1);
  EXPECT_NEAR(left_turn.rollDeg(), 1.0, 0.1);
  EXPECT_NEAR(right_turn.rollDeg(), left_turn.rollDeg(), 0.05);
}

TEST(RoadRoll, CrawlingSpeedsAreRefused)
{
  adas::RoadRollEstimator est;
  for (int i = 0; i < 500; ++i)
    EXPECT_FALSE(est.update(3.0, 0.05, 0.0, kDt));
  EXPECT_FALSE(est.valid());
  EXPECT_DOUBLE_EQ(est.rollStdDeg(), adas::RoadRollEstimator::Config{}.unconverged_std_deg);
}

TEST(RoadRoll, TransientsAreRefused)
{
  adas::RoadRollEstimator est;
  int used = 0;
  double yaw = 0.0;
  for (int i = 0; i < 200; ++i) {
    yaw -= 0.01;
    const double f_y = lateralSpecificForce(20.0, yaw, 0.0, 3.0);
    used += est.update(20.0, yaw, f_y, kDt) ? 1 : 0;
  }
  EXPECT_LT(used, 20) << "a fast steering ramp should be mostly rejected";
}

TEST(RoadRoll, ABumpDoesNotMoveTheEstimate)
{
  auto est = settle(1.0, 20.0, 0.0);
  const double before = est.rollDeg();
  est.update(20.0, 0.0, -kG * std::sin(20.0 * M_PI / 180.0), kDt);
  EXPECT_NEAR(est.rollDeg(), before, 1e-9);
}

TEST(RoadRoll, UncertaintyStartsAtTheParamsdFallbackAndSettlesOnTheMeasuredFloor)
{
  adas::RoadRollEstimator est;
  EXPECT_DOUBLE_EQ(est.rollStdDeg(), 10.0) << "same value paramsd uses for 'no usable roll'";

  const auto settled = settle(0.5, 20.0, 0.0);
  // Never better than the 0.65 deg the data supports: past ten seconds the residual is the road itself.
  EXPECT_DOUBLE_EQ(settled.rollStdDeg(), 0.65);
  EXPECT_TRUE(settled.valid());
}

TEST(RoadRoll, ConvergesWithinTheTimeConstantNotInstantly)
{
  adas::RoadRollEstimator est;
  const double f_y = lateralSpecificForce(20.0, 0.0, 2.0, 3.0);
  est.update(20.0, 0.0, f_y, kDt);
  EXPECT_NEAR(est.rollDeg(), 2.0, 1e-9) << "first sample seeds the estimate";

  adas::RoadRollEstimator slow;
  const double flat = lateralSpecificForce(20.0, 0.0, 0.0, 3.0);
  for (int i = 0; i < 300; ++i)
    slow.update(20.0, 0.0, flat, kDt);
  const double step = lateralSpecificForce(20.0, 0.0, 2.0, 3.0);
  slow.update(20.0, 0.0, step, kDt);
  EXPECT_LT(slow.rollDeg(), 0.1) << "a single sample must not drag a converged estimate";
}
