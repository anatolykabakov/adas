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

class CarSim {
public:
  CarSim(double stiffness, double ratio, double offset_deg, const adas::ParamsLearner::Config& cfg)
    : cfg_(cfg), sf_(stiffness), sr_(ratio), off_(offset_deg * M_PI / 180.0)
  {
  }

  double step(double speed, double swa_deg, double roll_deg, double dt)
  {
    constexpr double kG = 9.81;
    constexpr double kCivicMass = 1326.0 + 136.0, kCivicWheelbase = 2.70;
    const double wb = cfg_.vehicle.wheelbase_m;
    const double aF = wb * cfg_.vehicle.center_to_front_frac, aR = wb - aF;
    const double cF = sf_ * 192150.0 * cfg_.vehicle.mass_kg / kCivicMass * (aR / wb) / (0.6);
    const double cR = sf_ * 202500.0 * cfg_.vehicle.mass_kg / kCivicMass * (aF / wb) / (0.4);
    const double m = cfg_.vehicle.mass_kg, j = cfg_.rotational_inertia;
    const double u = std::max(speed, 1.0);
    const double sa = cfg_.steer_sign * (swa_deg * M_PI / 180.0) - off_;

    const int sub = 20;
    const double h = dt / sub;
    for (int i = 0; i < sub; ++i) {
      const double dv = -(cF + cR) / (m * u) * v_ + (-(cF * aF - cR * aR) / (m * u) - u) * r_ + cF / (m * sr_) * sa -
                        kG * roll_deg * M_PI / 180.0;
      const double dr = -(cF * aF - cR * aR) / (j * u) * v_ + -(cF * aF * aF + cR * aR * aR) / (j * u) * r_ +
                        cF * aF / (j * sr_) * sa;
      v_ += h * dv;
      r_ += h * dr;
    }
    return -r_;  // внутри z вниз, на шине ISO
  }

private:
  adas::ParamsLearner::Config cfg_;
  double sf_, sr_, off_;
  double v_ = 0.0, r_ = 0.0;
};

adas::ParamsLearner learn(double truth_stiffness, double truth_ratio = 15.7, double truth_offset_deg = 0.0,
                          double bank_deg = 0.0, bool feed_roll = false, adas::ParamsLearner::Config cfg = {})
{
  cfg.use_roll = feed_roll;
  adas::ParamsLearner est(cfg);
  CarSim sim(truth_stiffness, truth_ratio, truth_offset_deg, cfg);

  const double swas[] = {6.0, -6.0, 12.0, -12.0, 20.0, -20.0, 3.0, -3.0};
  const double speeds[] = {12.0, 12.0, 15.0, 15.0, 22.0, 22.0, 25.0, 25.0};
  for (int rep = 0; rep < 120; ++rep) {
    for (int c = 0; c < 8; ++c) {
      const double swa = swas[c], v = speeds[c];
      const double roll = bank_deg;
      for (int i = 0; i < 100; ++i) {
        const double yaw = sim.step(v, swa, roll, kDt);
        est.update(v, swa, yaw, feed_roll ? roll : 0.0, feed_roll ? 0.5 : 99.0, kDt);
      }
    }
  }
  return est;
}

adas::ParamsLearner::Config movable()
{
  adas::ParamsLearner::Config cfg;
  cfg.stiffness_p0_std = 0.3;
  return cfg;
}

}  // namespace

TEST(ParamsLearner, StiffnessIsOnlyWeaklyObservable)
{
  for (const double truth : {0.6, 1.4}) {
    const auto est = learn(truth, 15.7, 0.0, 0.0, false, movable());
    const double start = movable().stiffness_init;
    EXPECT_LT(std::abs(est.stiffnessFactor() - truth), std::abs(start - truth)) << "truth " << truth;
    EXPECT_NEAR(est.stiffnessFactor(), truth, 0.35) << "truth " << truth;
  }
}

TEST(ParamsLearner, RecoversASteeringBias)
{
  const auto est = learn(1.0, 15.7, /*truth_offset_deg=*/0.8, 0.0, false, movable());
  EXPECT_NEAR(est.angleOffsetTotalDeg(), 0.8, 0.4);
}

TEST(ParamsLearner, RecoversTheSteerRatio)
{
  const auto est = learn(1.0, 17.0, 0.0, 0.0, false, movable());
  EXPECT_NEAR(est.steerRatio(), 17.0, 1.0);
}

TEST(ParamsLearner, KnowingTheRollRecoversTheTruthOnABankedRoad)
{
  const auto informed = learn(1.0, 15.7, 0.0, 1.5, /*feed_roll=*/true, movable());
  EXPECT_NEAR(informed.stiffnessFactor(), 1.0, 0.3);
}

TEST(ParamsLearner, ThePseudoObservationsBoundTheUncertainty)
{
  adas::ParamsLearner est(movable());
  for (int i = 0; i < 60000; ++i)
    est.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  const double bounded = est.stiffnessStd();

  adas::ParamsLearner::Config loose = movable();
  loose.stiffness_obs_std = 1e6;  // псевдонаблюдение фактически выключено
  adas::ParamsLearner without(loose);
  for (int i = 0; i < 60000; ++i)
    without.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_GT(without.stiffnessStd(), bounded) << "без псевдонаблюдения прямая раздувает сигму";
}

TEST(ParamsLearner, TheIntegratorIsStableAtTheUpstreamStep)
{
  adas::ParamsLearner est(movable());
  CarSim sim(1.0, 15.7, 0.0, movable());
  for (int i = 0; i < 4000; ++i) {
    const double swa = (i / 40) % 2 ? 10.0 : -10.0;
    const double yaw = sim.step(6.0, swa, 0.0, 0.05);
    est.update(6.0, swa, yaw, 0.0, 0.5, 0.05);
  }
  EXPECT_TRUE(std::isfinite(est.stiffnessFactor()));
  EXPECT_TRUE(std::isfinite(est.steerRatio()));
  EXPECT_GE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_min);
  EXPECT_LE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_max);
}

TEST(ParamsLearner, GatesRefuseWhatTheModelCannotExplain)
{
  adas::ParamsLearner est;
  EXPECT_FALSE(est.update(0.5, 5.0, -0.05, 0.0, 0.5, kDt)) << "стоим";
  EXPECT_FALSE(est.update(20.0, 60.0, -0.05, 0.0, 0.5, kDt)) << "за линейной областью шины";
  EXPECT_FALSE(est.update(20.0, 5.0, -2.0, 0.0, 0.5, kDt)) << "рысканье за 1 рад/с";
  EXPECT_FALSE(est.update(20.0, 5.0, -0.05, 0.0, 0.5, 0.0)) << "нет шага времени";
  EXPECT_EQ(est.sampleCount(), 0);
}

TEST(ParamsLearner, StaysInsideItsBounds)
{
  adas::ParamsLearner est(movable());
  for (int i = 0; i < 20000; ++i) {
    const double swa = (i / 100) % 2 ? 8.0 : -8.0;
    est.update(20.0, swa, -0.9 * (swa > 0 ? 1.0 : -1.0), 0.0, 0.5, kDt);
  }
  EXPECT_GE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_min);
  EXPECT_LE(est.stiffnessFactor(), adas::ParamsLearner::Config{}.stiffness_max);
  EXPECT_GE(est.steerRatio(), 0.5 * adas::ParamsLearner::Config{}.steer_ratio_init);
  EXPECT_LE(est.steerRatio(), 2.0 * adas::ParamsLearner::Config{}.steer_ratio_init);
}

TEST(ParamsLearner, NotValidUntilItHasSeenCorners)
{
  adas::ParamsLearner est(movable());
  for (int i = 0; i < 5000; ++i)
    est.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_FALSE(est.valid()) << "прямая ничему не учит, сколько бы отсчётов ни было";
}

TEST(ParamsLearner, ThePredictionIsTheControllersOwnModel)
{
  adas::ParamsLearner est;
  const double v = 20.0, swa = 10.0;
  adas::VehicleModelParams p;
  p.tire_stiffness_factor = est.stiffnessFactor();
  const double delta = swa * M_PI / 180.0 / est.steerRatio();
  const double expected = v * adas::curvatureFromSteer(delta, v, p.wheelbase_m, adas::slipFactor(p));
  EXPECT_NEAR(est.predictYawRate(v, swa, 0.0), expected, 1e-12);
}

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

TEST(ParamsLearner, TheWrongSteeringSignRunsItIntoTheBounds)
{
  adas::ParamsLearner::Config bad = movable();
  bad.steer_sign = -1.0;  // неверен для данных ниже, они сгенерированы с +1
  adas::ParamsLearner::Config truth_cfg = movable();
  truth_cfg.steer_sign = 1.0;

  adas::ParamsLearner flipped(bad);
  CarSim sim(1.0, 15.7, 0.0, truth_cfg);
  const double swas[] = {8.0, -8.0, 15.0, -15.0};
  for (int rep = 0; rep < 40; ++rep) {
    for (const double swa : swas) {
      for (int i = 0; i < 100; ++i) {
        const double yaw = sim.step(20.0, swa, 0.0, kDt);
        flipped.update(20.0, swa, yaw, 0.0, 99.0, kDt);
      }
    }
  }
  EXPECT_GT(std::abs(flipped.stiffnessFactor() - 1.0), 0.3) << "неверный знак обязан испортить оценку";

  const auto right = learn(1.0, 15.7, 0.0, 0.0, false, truth_cfg);
  EXPECT_NEAR(right.stiffnessFactor(), 1.0, 0.25) << "с верным знаком те же данные дают истину";
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
  if (cfg.lane_keep.dp_parity_pack) {
    EXPECT_TRUE(cfg.lane_keep.use_learned_params) << "пакет объявлен, а наблюдатель не подключён — тогда флаг пакета "
                                                     "лишний";
    EXPECT_TRUE(cfg.localization.learn_vehicle_params) << "контроллер читает оценку, которую никто не считает";
  } else {
    EXPECT_FALSE(cfg.lane_keep.use_learned_params) << "the learned parameters have never driven this car; that "
                                                      "decision needs a bag behind it";
  }

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
  cfg.params.steer_sign = -1.0;  // this car: positive CAN angle is a left turn
  cfg.params.use_roll = false;   // as shipped
  const double truth_tsf = 1.4;  // заметно в стороне от единицы, с которой стартует оценщик
  cfg.params.stiffness_p0_std = 0.1;  // при P = Q апстрима жёсткость не двигается вовсе

  auto pub = std::make_shared<ChassisPublisher>();
  auto loc = std::make_shared<adas::services::Localization>(cfg);
  auto mgr = std::make_shared<adas::middleware::Manager>(adas::middleware::Manager::Mode::Simulated,
                                                         std::vector<adas::middleware::ServicePtr>{pub, loc});
  mgr->setTime(0);

  CarSim sim(truth_tsf, 15.7, 0.0, cfg.params);
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
        m.yaw_rate = sim.step(speeds[c], swas[c], 0.0, 0.01);
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
  EXPECT_NEAR(pose.learned_steer_ratio, 15.7, 1.0) << "the ratio must stay near its mechanical value";
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
