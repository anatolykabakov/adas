#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include "adas/adas_app.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/lane_keep.h"
#include "adas/services/localization.h"
#include "adas/utils/params_learner.h"

using adas::services::LaneKeep;

// The portable part of upstream's `paramsd`: learn tyre stiffness, steer ratio and the steering bias from
// steering angle, speed and yaw rate. It exists because one constant cannot represent the thing it stands
// for — measured on this car, `kappa_fact/kappa_kin` is 0.97 at 6–9 m/s, 0.80 at 12–15 and 0.54 at 21–26 —
// and because two independent estimates of it disagreed by a factor of two (our 0.54 against comma's
// learned 1.319 on the same car).
//
// Signs: everything inside the learner is z-down (positive yaw = right turn), and `chassis.yaw_rate` is ISO
// (left-positive), so the tests feed it the way the decoder delivers it and the learner negates.

namespace {

constexpr double kDt = 0.01;
constexpr double kG = 9.81;

/** Yaw rate a car with these parameters would report on CAN, i.e. ISO / left-positive. */
double canYawRate(double speed, double swa_deg, double truth_stiffness, double truth_ratio, double truth_offset_deg,
                  double roll_deg)
{
  adas::VehicleModelParams p;
  p.tire_stiffness_factor = truth_stiffness;
  const double slip = adas::slipFactor(p);
  const double delta = (swa_deg - truth_offset_deg) * M_PI / 180.0 / truth_ratio;
  const double kappa = adas::curvatureFromSteer(delta, speed, p.wheelbase_m, slip);
  const double yaw_zdown = speed * kappa + kG * std::sin(roll_deg * M_PI / 180.0) / speed;
  return -yaw_zdown;
}

/** How the road is banked through the corners a test drives. */
enum class Bank {
  kFlat,         //!< no bank at all
  kConstant,     //!< the same tilt everywhere, e.g. one lane of a crowned road
  kIntoTheTurn,  //!< what a designed road does: banked so the turn needs less lateral force
};

/** Drive a sequence of steady corners, alternating direction so the estimate cannot ride one bias. */
adas::ParamsLearner learn(double truth_stiffness, double truth_ratio = 15.7, double truth_offset_deg = 0.0,
                          double bank_deg = 0.0, Bank bank_kind = Bank::kFlat, bool feed_roll = true,
                          adas::ParamsLearner::Config cfg = {}, bool single_speed = false)
{
  // The shipped default is `use_roll = false` — measured, see the header — so the tests that are *about*
  // the roll term have to ask for it. Keeping the switch here rather than in each test means a test that
  // says `feed_roll=true` gets the banked-road path it is asking for.
  cfg.use_roll = feed_roll;
  adas::ParamsLearner est(cfg);

  // A dozen corners of different radius and speed, each held long enough to be steady.
  const double swas[] = {6.0, -6.0, 12.0, -12.0, 20.0, -20.0, 3.0, -3.0};
  const double speeds_multi[] = {12.0, 12.0, 15.0, 15.0, 22.0, 22.0, 25.0, 25.0};
  const double speeds_one[] = {18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0};
  const double* speeds = single_speed ? speeds_one : speeds_multi;
  for (int rep = 0; rep < 12; ++rep) {
    for (int c = 0; c < 8; ++c) {
      const double swa = swas[c];
      const double v = speeds[c];
      double roll = 0.0;
      if (bank_kind == Bank::kConstant)
        roll = bank_deg;
      else if (bank_kind == Bank::kIntoTheTurn)
        roll = swa > 0 ? bank_deg : -bank_deg;  // positive swa is a right turn in this frame

      const double yaw = canYawRate(v, swa, truth_stiffness, truth_ratio, truth_offset_deg, roll);
      const double roll_in = feed_roll ? roll : 0.0;
      const double roll_std = feed_roll ? 0.5 : 99.0;
      for (int i = 0; i < 60; ++i)  // 0.6 s per corner, jerk gate sees a step only on the first sample
        est.update(v, swa, yaw, roll_in, roll_std, kDt);
    }
  }
  return est;
}

}  // namespace

TEST(ParamsLearner, RecoversStiffnessFromSteadyCorners)
{
  for (const double truth : {0.45, 0.64, 0.90}) {
    const auto est = learn(truth);
    EXPECT_TRUE(est.valid()) << "truth " << truth;
    EXPECT_NEAR(est.stiffnessFactor(), truth, 0.06) << "truth " << truth;
  }
}

TEST(ParamsLearner, RecoversASteeringBias)
{
  // A bias we currently have no mechanism for at all; comma's learning reports +0.094 deg on this car.
  const auto est = learn(0.64, 15.7, /*truth_offset_deg=*/0.8);
  EXPECT_TRUE(est.valid());
  EXPECT_NEAR(est.angleOffsetDeg(), 0.8, 0.25);
}

TEST(ParamsLearner, KnowingTheRollRecoversTheTruthOnABankedRoad)
{
  const double truth = 0.64;
  for (const auto kind : {Bank::kConstant, Bank::kIntoTheTurn}) {
    const auto informed = learn(truth, 15.7, 0.0, 1.5, kind, /*feed_roll=*/true);
    EXPECT_NEAR(informed.stiffnessFactor(), truth, 0.06);
  }
}

TEST(ParamsLearner, AnUnknownBankLeaksIntoStiffnessButSpeedVarietyLimitsIt)
{
  // This started as "a banked road must visibly corrupt the estimate" and the measurement said otherwise, so
  // the test says what is true instead. Feeding no roll on a road banked 1.5° costs only 0.02–0.04 of
  // stiffness across corners spanning 12–25 m/s, and the reason is worth more than the number: the bank
  // enters the yaw prediction as g·sin(roll)/v while stiffness enters through the 1/(1+K·v²) term, so the two
  // have different speed signatures and a multi-speed data set separates them.
  //
  // Drive every corner at one speed and that separation disappears — which is also the practical advice:
  // a parameter learned on a single stretch of motorway is a parameter fitted to that stretch's camber.
  const double truth = 0.64;
  const auto multi = learn(truth, 15.7, 0.0, 1.5, Bank::kIntoTheTurn, /*feed_roll=*/false);
  const auto single = learn(truth, 15.7, 0.0, 1.5, Bank::kIntoTheTurn, /*feed_roll=*/false,
                            adas::ParamsLearner::Config{}, /*single_speed=*/true);

  const double err_multi = std::abs(multi.stiffnessFactor() - truth);
  const double err_single = std::abs(single.stiffnessFactor() - truth);
  EXPECT_GT(err_multi, 0.005) << "an unknown bank must not be free";
  EXPECT_GT(err_single, err_multi) << "one speed cannot separate the bank from the car";
}

TEST(ParamsLearner, StraightDrivingDoesNotLetTheUncertaintyRunAway)
{
  // A long straight carries no information about stiffness, so the random walk would inflate sigma until the
  // first corner threw the estimate. `paramsd` prevents that by "observing the state with its own value";
  // ported literally at CAN rate that freezes the filter instead (see `stiffness_std_max`), so it is a cap.
  adas::ParamsLearner est;
  for (int i = 0; i < 60000; ++i)  // ten minutes of straight road
    est.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_LE(est.stiffnessStd(), adas::ParamsLearner::Config{}.stiffness_std_max + 1e-9) << "sigma must be capped, not "
                                                                                           "free to grow";

  adas::ParamsLearner::Config no_cap;
  no_cap.stiffness_std_max = 1e9;  // effectively disabled
  adas::ParamsLearner without(no_cap);
  for (int i = 0; i < 60000; ++i)
    without.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_GT(without.stiffnessStd(), est.stiffnessStd()) << "without the cap the straight inflates sigma";
}

TEST(ParamsLearner, TheCapMustNotBeAPseudoMeasurement)
{
  // The bug this design avoids, stated as a test so nobody re-introduces it. Observing a state with its own
  // value has zero innovation, so it only shrinks the covariance — a hundred times a second that starves the
  // real measurement of gain and pins the estimate to wherever it started.
  adas::ParamsLearner::Config cfg;
  const auto est = learn(0.45, 15.7, 0.0, 0.0, Bank::kFlat, true, cfg);
  EXPECT_NEAR(est.stiffnessFactor(), 0.45, 0.06) << "if this drifts toward stiffness_init the cap has turned back into "
                                                    "a pseudo-measurement";
}

TEST(ParamsLearner, GatesRefuseWhatTheModelCannotExplain)
{
  adas::ParamsLearner est;
  EXPECT_FALSE(est.update(2.0, 5.0, -0.05, 0.0, 0.5, kDt)) << "crawling";
  EXPECT_FALSE(est.update(20.0, 60.0, -0.05, 0.0, 0.5, kDt)) << "past the linear tyre region";
  EXPECT_FALSE(est.update(20.0, 5.0, -2.0, 0.0, 0.5, kDt)) << "yaw rate beyond 1 rad/s";
  EXPECT_FALSE(est.update(20.0, 5.0, -0.05, 0.0, 0.5, 0.0)) << "no time step";
  EXPECT_EQ(est.sampleCount(), 0);
}

TEST(ParamsLearner, TransientsAreRefusedBecauseTheModelIsSteadyState)
{
  adas::ParamsLearner est;
  int used = 0;
  double swa = 0.0;
  for (int i = 0; i < 300; ++i) {
    swa += 0.2;  // a fast steering ramp
    const double yaw = canYawRate(20.0, swa, 0.64, 15.7, 0.0, 0.0);
    used += est.update(20.0, swa, yaw, 0.0, 0.5, kDt) ? 1 : 0;
  }
  EXPECT_LT(used, 60) << "a ramp is not a steady state and must be mostly dropped";
}

TEST(ParamsLearner, StaysInsideItsBounds)
{
  // Feed a yaw rate no set of sane parameters explains and check the filter refuses to leave the range
  // rather than chasing it. Upstream's own sanity range for stiffness is 0.2 to 5.
  adas::ParamsLearner est;
  for (int i = 0; i < 20000; ++i) {
    const double swa = (i / 100) % 2 ? 8.0 : -8.0;
    est.update(20.0, swa, -0.9 * (swa > 0 ? 1.0 : -1.0), 0.0, 0.5, kDt);
  }
  EXPECT_GE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_min);
  EXPECT_LE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_max);
  EXPECT_GE(est.steerRatio(), adas::ParamsLearner::Config{}.steer_ratio_min);
  EXPECT_LE(est.steerRatio(), adas::ParamsLearner::Config{}.steer_ratio_max);
}

TEST(ParamsLearner, NotValidUntilItHasSeenCorners)
{
  adas::ParamsLearner est;
  for (int i = 0; i < 5000; ++i)
    est.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_FALSE(est.valid()) << "a straight teaches nothing about stiffness, however many samples";
}

TEST(ParamsLearner, ThePredictionIsTheControllersOwnModel)
{
  // A learned parameter is only worth having if it means the same thing to the consumer. The prediction must
  // reduce to `curvatureFromSteer` on a flat road, which is what the lateral controller calls.
  adas::ParamsLearner est;
  const double v = 20.0, swa = 10.0;
  adas::VehicleModelParams p;
  p.tire_stiffness_factor = est.stiffnessFactor();
  const double delta = swa * M_PI / 180.0 / est.steerRatio();
  const double expected = v * adas::curvatureFromSteer(delta, v, p.wheelbase_m, adas::slipFactor(p));
  EXPECT_NEAR(est.predictYawRate(v, swa, 0.0), expected, 1e-12);
}

// ---------------------------------------------------------------------------------------------------
// The wiring. A learner nobody reads is a research project, and a learner read by accident is worse —
// these pin both directions of the two-flag arrangement.
// ---------------------------------------------------------------------------------------------------

TEST(LearnedParams, TheControllerKeepsItsConstantsUntilBothFlagsAgree)
{
  LaneKeep::Config cfg;
  cfg.tire_stiffness_factor = 0.64;
  cfg.steer_ratio = 15.7;
  cfg.use_learned_params = false;
  LaneKeep svc(cfg);

  // An estimate arrives, and it is a valid one. With the consumer flag off nothing may move.
  svc.setLearnedParams(true, 1.20, 16.4, 0.7);
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
  EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 15.7);
  EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.0);
  EXPECT_FALSE(svc.usingLearnedParams());
}

TEST(LearnedParams, WithTheFlagOnTheEstimateIsUsedAndLosingValidityWalksItBack)
{
  LaneKeep::Config cfg;
  cfg.tire_stiffness_factor = 0.64;
  cfg.steer_ratio = 15.7;
  cfg.use_learned_params = true;
  LaneKeep svc(cfg);

  // Before anything is learned the configured values are in force — not zeros, and not the learner's
  // initial guess arriving as if it were knowledge.
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
  EXPECT_FALSE(svc.usingLearnedParams());

  svc.setLearnedParams(true, 1.20, 16.4, 0.7);
  EXPECT_TRUE(svc.usingLearnedParams());
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 1.20);
  EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 16.4);
  EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.7);

  // Validity is lost — a stop, a reset, the sigma reopening. The configured values come back rather than
  // the last thing the estimator believed sticking around unexamined.
  svc.setLearnedParams(false, 1.20, 16.4, 0.7);
  EXPECT_FALSE(svc.usingLearnedParams());
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
  EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 15.7);
  EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.0);
}

TEST(LearnedParams, NonsenseIsRefusedEvenWhenItArrivesFlaggedValid)
{
  LaneKeep::Config cfg;
  cfg.use_learned_params = true;
  LaneKeep svc(cfg);
  svc.setLearnedParams(true, 0.0, 16.4, 0.0);  // stiffness zero: slipFactor would divide by it
  EXPECT_FALSE(svc.usingLearnedParams());
  svc.setLearnedParams(true, 0.8, 0.0, 0.0);  // ratio zero: the command would be identically zero
  EXPECT_FALSE(svc.usingLearnedParams());
}

TEST(ParamsLearner, TheWrongSteeringSignRunsItIntoTheBoundsAndStillClaimsValid)
{
  // Found by replaying the shipped filter over run 2026_08_06_00_36_42 (`bag_params_learner.py`). The port
  // negated the yaw rate on the way in — ISO to z-down, which is right — but did not apply this car's
  // `vehicle.steer_sign`, which the controller has always applied. On real data the CAN angle is positive
  // for a left turn (measured ISO yaw against the kinematic prediction: slope +0.824, correlation 0.987),
  // so the prediction opposed the measurement, and the filter went to whichever bounds shrink the predicted
  // magnitude: stiffness pinned to its 0.200 floor and steer ratio to its 20.0 ceiling, identical in every
  // quarter of the drive. The dangerous part is the last line of this test — a bounded, motionless,
  // completely wrong estimate reports itself as converged, because a saturated state has a small sigma.
  const double truth = 0.64;
  adas::ParamsLearner::Config bad;
  bad.steer_sign = -1.0;  // wrong for the data below, which is generated with +1
  // The loose ratio prior this filter shipped with when the defect was found. It has since been tightened
  // (see `steer_ratio_std_init`), which happens to hide part of this failure — so the test keeps the old
  // prior, because the point is the sign, not the prior.
  bad.steer_ratio_std_init = 0.5;
  bad.steer_ratio_std_max = 5.0;
  bad.steer_ratio_process_std = 0.005;
  const auto flipped = learn(truth, 15.7, 0.0, 0.0, Bank::kFlat, true, bad);

  EXPECT_LT(flipped.stiffnessFactor(), bad.stiffness_min + 1e-6) << "expected the floor, the failure mode";
  EXPECT_GT(flipped.steerRatio(), bad.steer_ratio_max - 1e-6) << "expected the ceiling";

  // The estimate is nonsense and `valid()` must say so. What catches it is the strict bound comparison, not
  // the sigma: a saturated state has stopped moving, so its sigma is small and reads as convergence. The
  // Python replay of this filter checked only count and sigma and duly reported `valid=True` on the bad
  // estimate — the same bug in the same place twice, which is why both bounds are checked here now.
  EXPECT_FALSE(flipped.valid());
  EXPECT_LT(flipped.stiffnessStd(), 0.15) << "the sigma alone would have let it through";

  // The same data with the sign the generator used recovers the truth, so the data is not the problem.
  adas::ParamsLearner::Config good;
  good.steer_sign = 1.0;
  const auto right = learn(truth, 15.7, 0.0, 0.0, Bank::kFlat, true, good);
  EXPECT_NEAR(right.stiffnessFactor(), truth, 0.06);
}

TEST(ShippedConfig, TheLearnerGetsTheSameSteeringSignAsTheController)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;
  EXPECT_DOUBLE_EQ(cfg.localization.params.steer_sign, cfg.lane_keep.steer_sign);
}

TEST(ShippedConfig, TheLearnerMayRunButTheControllerMayNotReadIt)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  // The observer ships **on** as of the 2026-08-06 drive: it changes no command, and running it is the only
  // way the estimate is ever seen on live data rather than in a bag replay. The consumer ships **off** and
  // this assertion is the one that matters — it is the difference between an instrument and a control input,
  // and the reason the two were ever separate flags. A header default is not a decision until the shipped
  // config agrees with it, which is why this suite exists at all.
  EXPECT_FALSE(cfg.lane_keep.use_learned_params) << "the learned parameters have never driven this car; that decision "
                                                    "needs a bag behind it";

  // Roll stays out of the learner: measured on the same run at 0.0114 rad/s of injected error per degree
  // against a 0.0065 rad/s residual. See `use_roll` in the header.
  EXPECT_FALSE(cfg.localization.params.use_roll);
}

TEST(LearnedParams, RunningTheObserverCannotTouchTheCommand)
{
  // The claim that lets the observer ship on. With the consumer flag off the controller must not subscribe
  // to the estimate at all, so no arriving value — valid, invalid or absurd — can reach a command.
  LaneKeep::Config cfg;
  cfg.tire_stiffness_factor = 0.64;
  cfg.steer_ratio = 15.7;
  cfg.use_learned_params = false;
  LaneKeep svc(cfg);

  for (const double tsf : {0.05, 0.2, 1.0, 4.9}) {
    svc.setLearnedParams(true, tsf, 12.5, 9.0);
    EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
    EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 15.7);
    EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.0);
  }
}

TEST(ShippedConfig, TheLearnerStartsFromTheParametersTheControllerUses)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  // One source for each constant. If the learner is given its own copy of the wheelbase or the stiffness,
  // a "learned" parameter comes to mean something slightly different from the parameter it replaces.
  const auto& pl = cfg.localization.params;
  EXPECT_DOUBLE_EQ(pl.vehicle.wheelbase_m, cfg.localization.wheelbase_m);
  EXPECT_DOUBLE_EQ(pl.vehicle.tire_stiffness_factor, cfg.lane_keep.tire_stiffness_factor);
  EXPECT_DOUBLE_EQ(pl.stiffness_init, cfg.lane_keep.tire_stiffness_factor);
  EXPECT_DOUBLE_EQ(pl.steer_ratio_init, cfg.lane_keep.steer_ratio);
}

// ---------------------------------------------------------------------------------------------------
// End-to-end through the service, because "the observer is enabled and learns nothing" is a failure that
// costs a drive rather than a test run. This exercises the real path: a ChassisSample on the real topic,
// through the real subscription, into the real publish.
// ---------------------------------------------------------------------------------------------------

namespace {

class ChassisPublisher : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "chassis_pub"; }
  void configure() override {}
  void send(const adas::ChassisSample& m) { publish(adas::topics::kVehicleChassis, m); }
};

}  // namespace

TEST(LearnedParams, TheServiceActuallyLearnsFromChassisMessages)
{
  adas::services::Localization::Config cfg;
  cfg.learn_vehicle_params = true;
  cfg.params.steer_sign = -1.0;   // this car: positive CAN angle is a left turn
  cfg.params.use_roll = false;    // as shipped
  const double truth_tsf = 0.45;  // deliberately far from the 0.64 the learner starts at

  auto pub = std::make_shared<ChassisPublisher>();
  auto loc = std::make_shared<adas::services::Localization>(cfg);
  auto mgr = std::make_shared<adas::middleware::Manager>(adas::middleware::Manager::Mode::Simulated,
                                                         std::vector<adas::middleware::ServicePtr>{pub, loc});
  mgr->setTime(0);

  // Steady corners from a car whose stiffness is 0.45, alternating direction, at several speeds.
  const double swas[] = {6.0, -6.0, 12.0, -12.0, 20.0, -20.0};
  const double speeds[] = {12.0, 12.0, 15.0, 15.0, 22.0, 22.0};
  int64_t t_us = 1'000'000;
  for (int rep = 0; rep < 20; ++rep) {
    for (int c = 0; c < 6; ++c) {
      for (int i = 0; i < 60; ++i) {
        adas::ChassisSample m;
        t_us += 10'000;  // 100 Hz, the CAN rate
        m.timestamp_us = t_us;
        m.speed_mps = speeds[c];
        m.steering_angle_deg = swas[c];
        // The generator is the learner's own model at the true stiffness, delivered in the ISO convention
        // the MQB decoder produces — the same round trip the real signal makes.
        // `canYawRate` already returns the ISO/left-positive value the MQB decoder produces, but it is
        // built for `steer_sign = +1`; this car is -1, so the corner turns the other way.
        m.yaw_rate = -canYawRate(speeds[c], swas[c], truth_tsf, 15.7, 0.0, 0.0);
        m.steer_rad = swas[c] * M_PI / 180.0 / 15.7;
        pub->send(m);
        mgr->step();
      }
    }
  }

  const auto& pose = loc->lastPose();
  EXPECT_GT(pose.learned_sample_count, 500) << "the learner saw no usable samples through the service";
  EXPECT_NE(pose.learned_stiffness_factor, cfg.params.stiffness_init) << "the estimate never moved off its starting "
                                                                         "value — check that steering_angle_deg is "
                                                                         "populated";
  EXPECT_GT(pose.learned_stiffness_std, 0.0);
  EXPECT_LT(pose.learned_stiffness_std, 0.5);
  EXPECT_NEAR(pose.learned_steer_ratio, 15.7, 0.5) << "the ratio must stay near its mechanical value";
}

TEST(LearnedParams, WithTheObserverOffNothingIsPublished)
{
  adas::services::Localization::Config cfg;
  cfg.learn_vehicle_params = false;

  auto pub = std::make_shared<ChassisPublisher>();
  auto loc = std::make_shared<adas::services::Localization>(cfg);
  auto mgr = std::make_shared<adas::middleware::Manager>(adas::middleware::Manager::Mode::Simulated,
                                                         std::vector<adas::middleware::ServicePtr>{pub, loc});
  mgr->setTime(0);

  for (int i = 0; i < 500; ++i) {
    adas::ChassisSample m;
    m.timestamp_us = 1'000'000 + i * 10'000;
    m.speed_mps = 15.0;
    m.steering_angle_deg = 12.0;
    m.yaw_rate = 0.1;
    pub->send(m);
    mgr->step();
  }

  EXPECT_EQ(loc->lastPose().learned_sample_count, 0);
  EXPECT_FALSE(loc->lastPose().learned_params_valid);
}
