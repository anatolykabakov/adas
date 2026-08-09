#include <cmath>

#include <gtest/gtest.h>

#include "adas/utils/vehicle_ekf.h"

// Heading in `localization/pose` used to come almost entirely from the bicycle model: `predict`
// advanced it with `v·tan(δ)/L` and then overwrote the yaw-rate state with the same value, so the
// gyro reached heading only through cross-covariance — weight `dt·(1 − K₄) ≈ 0.12·dt` on our
// `Q₄₄ = 0.05²` and `R_imu = 0.02²`, i.e. heading = 0.88 · bicycle + 0.12 · measured.
//
// That is only safe if the bicycle model is right. Measured understeer on this car is
// `κ_fact/κ_kin` = 0.54 at 22 m/s, so it over-turns by 1.85×. On the road GPS heading hides the
// error; without GPS the heading runs away. These tests drive a steady arc where the gyro reports the
// truth and the steering angle implies 1.85× of it, which is the real situation.

namespace {

constexpr double kDt = 0.01;
constexpr double kSpeed = 22.0;
constexpr double kUndersteer = 0.54;  // κ_fact / κ_kin measured at this speed

/** Run a steady arc for `seconds`, feeding a gyro that tells the truth and a steering angle that,
 *  through the kinematic model, claims 1/0.54 times as much. Returns final heading in rad. */
double driveArc(bool yaw_rate_is_state, double seconds, double steer_rad, bool feed_gyro = true)
{
  adas::VehicleEKF ekf(/*wheelbase=*/2.636);
  ekf.setYawRateIsAState(yaw_rate_is_state);
  ekf.reset(0.0, 0.0, 0.0, kSpeed, 0.0, /*pos_unc=*/1.0, /*yaw_unc=*/0.05, /*v_unc=*/0.5,
            /*yaw_rate_unc=*/0.05);

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
  // 5 s, not 30: heading is normalised to [-pi, pi], so a longer arc wraps and the comparison stops
  // meaning anything. At 0.18 rad/s of truth, 5 s gives 0.9 rad and the model's 1.67 — both unwrapped.
  const double steer = 0.04;  // ~2.3 deg of road wheel
  const double model_rate = kSpeed * std::tan(steer) / 2.636;
  const double truth = kUndersteer * model_rate * 5.0;
  const double model_heading = model_rate * 5.0;

  const double with_state = driveArc(/*yaw_rate_is_state=*/true, 5.0, steer);
  const double old_way = driveArc(/*yaw_rate_is_state=*/false, 5.0, steer);

  // The gyro is the only honest witness here, so the new filter should land on it.
  EXPECT_NEAR(with_state, truth, 0.1 * std::abs(truth));
  // ... while the old one follows the model, which over-turns by 1/0.54.
  EXPECT_GT(std::abs(old_way), 1.5 * std::abs(truth));
  EXPECT_NEAR(old_way, model_heading, 0.15 * std::abs(model_heading));
}

TEST(VehicleEkfHeading, WithoutAGyroTheModelStillDrivesHeading)
{
  // The old overwrite handled the gyro-less case by accident. The replacement has to handle it on
  // purpose, or a bad IMU would freeze the heading instead of degrading to the model.
  const double steer = 0.04;
  const double model_rate = kSpeed * std::tan(steer) / 2.636;
  const double model_heading = model_rate * 5.0;

  const double y = driveArc(/*yaw_rate_is_state=*/true, 5.0, steer, /*feed_gyro=*/false);
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
  // The whole point of the change. With the wheel straight the bicycle model says "not turning" while
  // the gyro says 0.1 rad/s — a skid, a crowned road, a mis-zeroed steering sensor. Over 2 s the truth
  // is 0.2 rad of heading. The old filter wiped the state at the top of every predict, so heading
  // barely moved; only cross-covariance leaked anything through.
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
  // Whatever the yaw-rate handling, a GPS course fix must remain able to pull the heading back.
  adas::VehicleEKF ekf;
  ekf.setYawRateIsAState(true);
  ekf.reset(0.0, 0.0, 0.0, kSpeed, 0.0, 1.0, 0.5);
  for (int i = 0; i < 50; ++i)
    ASSERT_TRUE(ekf.updateGpsYaw(0.4));
  EXPECT_NEAR(ekf.yaw(), 0.4, 0.02);
}

// ── Speed as a state ──────────────────────────────────────────────────────────────────────────
//
// `predict` used to assign `state_(3) = v_measured` on every tick — about every 10 ms — while
// `updateGpsVel` runs at most every 0.2 s and only when the GPS course is valid. So the velocity update
// was overwritten roughly twenty times before the next GPS sample, and the fused speed inherited the
// wheel-speed scale exactly: `localization/pose.v` measured 1.011 against GNSS Doppler, raw CAN 1.012.

TEST(VehicleEkfSpeed, AGpsVelocityUpdateNowSurvivesOneTick)
{
  // The old mode was stranger than "the update does nothing": with nothing ever shrinking P(3,3), the
  // GPS gain was almost 1, so the update moved the speed *fully* to the GPS value — and then the next
  // `predict` assigned the wheel speed straight over it. Full correction, zero memory. What matters is
  // therefore not the value in the tick GPS arrives, but the value one tick later.
  auto after_one_more_tick = [](bool as_state) {
    adas::VehicleEKF ekf;
    ekf.setSpeedIsAState(as_state);
    ekf.reset(0.0, 0.0, 0.0, 20.0, 0.0);
    for (int i = 0; i < 200; ++i)  // settle
      ekf.predict(20.24, 0.0, kDt);
    ekf.updateGpsVel(20.0, 0.0, 0.1 * 0.1);
    ekf.predict(20.24, 0.0, kDt);  // the tick that used to erase it
    return ekf.v();
  };

  EXPECT_NEAR(after_one_more_tick(false), 20.24, 1e-9) << "old: predict reassigns, the correction is gone";
  EXPECT_LT(after_one_more_tick(true), 20.24 - 1e-3) << "new: the correction is still in the state";
}

TEST(VehicleEkfSpeed, ButTheWheelSpeedDragsItStraightBack)
{
  // And here is why making speed a state is *not* the fix for the 1.2 % scale, only the prerequisite for
  // one. The wheel measurement arrives every 10 ms with 0.1 m/s of assumed noise; GPS velocity arrives
  // every 0.2 s with the same. Twenty pulls against one, so between GPS samples the estimate returns to
  // the biased sensor. Tuning the noise cannot fix this: the bias is a constant on the frequent
  // measurement, and no pair of unbiased-noise assumptions separates a constant bias from the truth.
  // That needs the scale itself as a state — which is what `paramsd` does for the vehicle parameters.
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
  // The noise it used to be given, 1 m/s, made the update irrelevant even in the tick it arrived in.
  // Measured Doppler is an order of magnitude better than that: the residual against scale-corrected
  // wheel speed is 0.066-0.101 m/s on two runs, and that includes both sensors and the 1 Hz sampling.
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
