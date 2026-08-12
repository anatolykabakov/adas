#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "adas/utils/speed_filter.h"

namespace {
constexpr double kDt = 0.01;
constexpr double kQuantum = 0.0075 / 3.6;

double quantise(double v) { return std::round(v / kQuantum) * kQuantum; }

}  // namespace

TEST(SpeedFilter, HoldsSteadySpeedWithoutInventingAcceleration)
{
  adas::SpeedFilter f;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.02);

  std::vector<double> accels;
  for (int i = 0; i < 3000; ++i) {
    f.update(quantise(20.0 + noise(rng)), kDt);
    if (i > 500)
      accels.push_back(f.accel());
  }
  EXPECT_NEAR(f.speed(), 20.0, 0.05);

  double worst = 0.0;
  for (const double a : accels)
    worst = std::max(worst, std::abs(a));
  // would mean the filter is still reading the quantiser.
  EXPECT_LT(worst, 0.5);
}

TEST(SpeedFilter, TracksARealAcceleration)
{
  adas::SpeedFilter f;
  const double a_true = 1.5;
  double v = 5.0;
  for (int i = 0; i < 400; ++i) {
    v += a_true * kDt;
    f.update(quantise(v), kDt);
  }
  EXPECT_NEAR(f.speed(), v, 0.1);
  EXPECT_NEAR(f.accel(), a_true, 0.3) << "must follow a real launch, not just smooth it away";
}

TEST(SpeedFilter, TracksBraking)
{
  adas::SpeedFilter f;
  double v = 20.0;
  for (int i = 0; i < 200; ++i)
    f.update(quantise(v), kDt);
  const double a_true = -3.0;
  for (int i = 0; i < 300; ++i) {
    v += a_true * kDt;
    f.update(quantise(v), kDt);
  }
  EXPECT_NEAR(f.accel(), a_true, 0.4);
}

TEST(SpeedFilter, SurvivesAVaryingFrameInterval)
{
  adas::SpeedFilter f;
  for (int i = 0; i < 2000; ++i)
    f.update(quantise(15.0), i % 2 ? 0.010 : 0.025);
  EXPECT_NEAR(f.speed(), 15.0, 0.05);
  EXPECT_LT(std::abs(f.accel()), 0.3);
}

TEST(SpeedFilter, ReseedsOnAJumpInsteadOfIntegratingIt)
{
  adas::SpeedFilter f;
  for (int i = 0; i < 500; ++i)
    f.update(quantise(10.0), kDt);
  ASSERT_LT(std::abs(f.accel()), 0.3);

  f.update(quantise(25.0), kDt);
  EXPECT_NEAR(f.speed(), 25.0, 0.05);
  EXPECT_NEAR(f.accel(), 0.0, 1e-9) << "a 15 m/s step in 10 ms is not 1500 m/s^2";
}

TEST(SpeedFilter, ReseedsAfterALongGap)
{
  adas::SpeedFilter f;
  for (int i = 0; i < 500; ++i)
    f.update(quantise(10.0), kDt);
  f.update(quantise(10.0), 3.0);
  EXPECT_NEAR(f.speed(), 10.0, 0.05);
  EXPECT_NEAR(f.accel(), 0.0, 1e-9);
}

TEST(SpeedFilter, WheelSpeedFactorScalesTheSignal)
{
  adas::SpeedFilter plain;
  EXPECT_DOUBLE_EQ(plain.config().wheel_speed_factor, 1.0);

  adas::SpeedFilter::Config cfg;
  cfg.wheel_speed_factor = 1.0 / 1.0118;
  adas::SpeedFilter corrected(cfg);
  for (int i = 0; i < 1000; ++i) {
    plain.update(20.236, kDt);
    corrected.update(20.236, kDt);
  }
  EXPECT_NEAR(plain.speed(), 20.236, 0.02);
  EXPECT_NEAR(corrected.speed(), 20.0, 0.02);
}

TEST(SpeedFilter, StartsFromTheFirstSampleNotFromZero)
{
  adas::SpeedFilter f;
  EXPECT_FALSE(f.ready());
  f.update(18.0, kDt);
  EXPECT_TRUE(f.ready());
  EXPECT_NEAR(f.speed(), 18.0, 1e-9);
  EXPECT_NEAR(f.accel(), 0.0, 1e-9);
}
