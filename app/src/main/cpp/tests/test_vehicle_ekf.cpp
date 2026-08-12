#include <cmath>

#include <gtest/gtest.h>

#include "adas/utils/vehicle_ekf.h"

namespace {
constexpr double kDt = 0.01;
constexpr double kSpeed = 22.0;
constexpr double kUndersteer = 0.54;

/** Run a steady arc for `seconds`, feeding a gyro that tells the truth and a steering angle that,
 *  through the kinematic model, claims 1/0.54 times as much. Returns final heading in rad. */
double driveArc(bool yaw_rate_is_state, double seconds, double steer_rad, bool feed_gyro = true)
{
  adas::VehicleEKF ekf(2.636);
  ekf.setYawRateIsAState(yaw_rate_is_state);
  ekf.reset(0.0, 0.0, 0.0, kSpeed, 0.0, 1.0, 0.05, 0.5, 0.05);

  const double model_rate = kSpeed * std::tan(steer_rad) / 2.636;
  const double true_rate = kUndersteer * model_rate;

  const int steps = static_cast<int>(seconds / kDt);
  for (int i = 0; i < steps; ++i) {
    ekf.predict(kSpeed, steer_rad, kDt);
    if (feed_gyro)
      ekf.updateImu(true_rate);
  }
  return ekf.yaw();
}

}  // namespace

TEST(VehicleEkfHeading, GyroWinsOverTheBicycleModel)
{
  const double steer = 0.04;
  const double model_rate = kSpeed * std::tan(steer) / 2.636;
  const double truth = kUndersteer * model_rate * 5.0;
  const double model_heading = model_rate * 5.0;

  const double with_state = driveArc(true, 5.0, steer);
  const double old_way = driveArc(false, 5.0, steer);

  // The gyro is the only honest witness here, so the new filter should land on it.
  EXPECT_NEAR(with_state, truth, 0.1 * std::abs(truth));
  // ... while the old one follows the model, which over-turns by 1/0.54.
  EXPECT_GT(std::abs(old_way), 1.5 * std::abs(truth));
  EXPECT_NEAR(old_way, model_heading, 0.15 * std::abs(model_heading));
}

TEST(VehicleEkfHeading, WithoutAGyroTheModelStillDrivesHeading)
{
  const double steer = 0.04;
  const double model_rate = kSpeed * std::tan(steer) / 2.636;
  const double model_heading = model_rate * 5.0;

  const double y = driveArc(true, 5.0, steer, false);
  EXPECT_GT(std::abs(y), 0.5 * std::abs(model_heading)) << "heading must not stall without a gyro";
  EXPECT_NEAR(y, model_heading, 0.3 * std::abs(model_heading));
}

TEST(VehicleEkfHeading, StraightLineHoldsHeadingEitherWay)
{
  for (const bool as_state : {false, true}) {
    adas::VehicleEKF ekf;
    ekf.setYawRateIsAState(as_state);
    ekf.reset(0.0, 0.0, 0.3, kSpeed, 0.0);
    for (int i = 0; i < 2000; ++i) {
      ekf.predict(kSpeed, 0.0, kDt);
      ekf.updateImu(0.0);
    }
    EXPECT_NEAR(ekf.yaw(), 0.3, 1e-3) << "as_state=" << as_state;
  }
}

TEST(VehicleEkfHeading, AMeasuredYawRateSurvivesTheNextPredict)
{
  auto run = [](bool as_state) {
    adas::VehicleEKF ekf;
    ekf.setYawRateIsAState(as_state);
    ekf.reset(0.0, 0.0, 0.0, kSpeed, 0.0, 1.0, 0.05, 0.5, 0.05);
    for (int i = 0; i < 200; ++i) {
      ekf.predict(kSpeed, 0.0, kDt);
      ekf.updateImu(0.1);
    }
    return ekf;
  };

  const adas::VehicleEKF now = run(true);
  const adas::VehicleEKF before = run(false);

  EXPECT_NEAR(now.yawRate(), 0.1, 0.01);
  EXPECT_NEAR(now.yaw(), 0.2, 0.03);
  EXPECT_LT(std::abs(before.yaw()), 0.05) << "old behaviour: the measurement never reached heading";
}

TEST(VehicleEkfHeading, ModelMeasurementIsCountedApartFromTheGyro)
{
  adas::VehicleEKF ekf;
  ekf.setYawRateIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, kSpeed);
  for (int i = 0; i < 100; ++i) {
    ekf.predict(kSpeed, 0.04, kDt);
    ekf.updateImu(0.05);
  }
  EXPECT_EQ(ekf.imu_update_count, 100);
  EXPECT_EQ(ekf.model_update_count, 100);
  EXPECT_EQ(ekf.cam_odo_update_count, 0);
}

TEST(VehicleEkfHeading, GpsCourseStillCorrectsHeading)
{
  adas::VehicleEKF ekf;
  ekf.setYawRateIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, kSpeed, 0.0, 1.0, 0.5);
  for (int i = 0; i < 50; ++i)
    ASSERT_TRUE(ekf.updateGpsYaw(0.4));
  EXPECT_NEAR(ekf.yaw(), 0.4, 0.02);
}

// wheel-speed scale exactly: `localization/pose.v` measured 1.011 against GNSS Doppler, raw CAN 1.012.

TEST(VehicleEkfSpeed, AGpsVelocityUpdateNowSurvivesOneTick)
{
  auto after_one_more_tick = [](bool as_state) {
    adas::VehicleEKF ekf;
    ekf.setSpeedIsAState(as_state);
    ekf.reset(0.0, 0.0, 0.0, 20.0, 0.0);
    for (int i = 0; i < 200; ++i)
      ekf.predict(20.24, 0.0, kDt);
    ekf.updateGpsVel(20.0, 0.0, 0.1 * 0.1);
    ekf.predict(20.24, 0.0, kDt);
    return ekf.v();
  };

  EXPECT_NEAR(after_one_more_tick(false), 20.24, 1e-9) << "old: predict reassigns, the correction is gone";
  EXPECT_LT(after_one_more_tick(true), 20.24 - 1e-3) << "new: the correction is still in the state";
}

TEST(VehicleEkfSpeed, ButTheWheelSpeedDragsItStraightBack)
{
  adas::VehicleEKF ekf;
  ekf.setSpeedIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, 20.0, 0.0);

  double right_after_gps = 0.0;
  for (int i = 0; i < 400; ++i) {
    ekf.predict(20.24, 0.0, kDt);
    if (i % 20 == 0) {
      ekf.updateGpsVel(20.0, 0.0, 0.1 * 0.1);
      right_after_gps = ekf.v();
    }
  }
  EXPECT_LT(right_after_gps, 20.20) << "GPS pulls it down when it speaks";
  EXPECT_NEAR(ekf.v(), 20.24, 1e-3) << "and the wheels have it back before GPS speaks again";
}

TEST(VehicleEkfSpeed, AtTheOldAssumedNoiseGpsVelocityWasDecorative)
{
  adas::VehicleEKF ekf;
  ekf.setSpeedIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, 20.0, 0.0);
  for (int i = 0; i < 200; ++i)
    ekf.predict(20.24, 0.0, kDt);
  ekf.updateGpsVel(20.0, 0.0, 1.0);
  EXPECT_NEAR(ekf.v(), 20.24, 5e-3) << "at R=1.0 the measurement barely registers";
}

TEST(VehicleEkfSpeed, TracksARealSpeedChange)
{
  adas::VehicleEKF ekf;
  ekf.setSpeedIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, 10.0, 0.0);
  double v = 10.0;
  for (int i = 0; i < 500; ++i) {
    v += 1.5 * kDt;
    ekf.predict(v, 0.0, kDt);
  }
  EXPECT_NEAR(ekf.v(), v, 0.2) << "a state must still follow the measurement it is fed";
}

TEST(VehicleEkfSpeed, CountersSeparateWheelFromGps)
{
  adas::VehicleEKF ekf;
  ekf.setSpeedIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, 15.0, 0.0);
  for (int i = 0; i < 100; ++i)
    ekf.predict(15.0, 0.0, kDt);
  EXPECT_EQ(ekf.wheel_speed_update_count, 100);
  EXPECT_EQ(ekf.gps_vel_update_count, 0);
}
