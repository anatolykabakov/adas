#include <cmath>
#include "adas/utils/proto_convert.h"
#include <cstdlib>

#include <gtest/gtest.h>

#include "adas/adas_app.h"
#include "adas/middleware/manager.hpp"
#include "adas/services/planner.h"
#include "adas/services/localization.h"
#include "adas/utils/params_learner.h"

using adas::services::Planner;

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
    return -r_;
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
  const auto est = learn(1.0, 15.7, 0.8, 0.0, false, movable());
  EXPECT_NEAR(est.angleOffsetTotalDeg(), 0.8, 0.4);
}

TEST(ParamsLearner, RecoversTheSteerRatio)
{
  const auto est = learn(1.0, 17.0, 0.0, 0.0, false, movable());
  EXPECT_NEAR(est.steerRatio(), 17.0, 1.0);
}

TEST(ParamsLearner, KnowingTheRollRecoversTheTruthOnABankedRoad)
{
  const auto informed = learn(1.0, 15.7, 0.0, 1.5, true, movable());
  EXPECT_NEAR(informed.stiffnessFactor(), 1.0, 0.3);
}

TEST(ParamsLearner, ThePseudoObservationsBoundTheUncertainty)
{
  adas::ParamsLearner est(movable());
  for (int i = 0; i < 60000; ++i)
    est.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  const double bounded = est.stiffnessStd();

  adas::ParamsLearner::Config loose = movable();
  loose.stiffness_obs_std = 1e6;
  adas::ParamsLearner without(loose);
  for (int i = 0; i < 60000; ++i)
    without.update(25.0, 0.0, 0.0, 0.0, 0.5, kDt);
  EXPECT_GT(without.stiffnessStd(), bounded) << "without the pseudo-observation a straight road inflates sigma";
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
  EXPECT_FALSE(est.update(0.5, 5.0, -0.05, 0.0, 0.5, kDt)) << "standing still";
  EXPECT_FALSE(est.update(20.0, 60.0, -0.05, 0.0, 0.5, kDt)) << "beyond the tyre's linear region";
  EXPECT_FALSE(est.update(20.0, 5.0, -2.0, 0.0, 0.5, kDt)) << "yaw rate beyond 1 rad/s";
  EXPECT_FALSE(est.update(20.0, 5.0, -0.05, 0.0, 0.5, 0.0)) << "no time step";
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
  EXPECT_FALSE(est.valid()) << "a straight road teaches nothing, however many samples";
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
TEST(LearnedParams, WithTheFlagOnTheEstimateIsUsedAndLosingValidityWalksItBack)
{
  Planner::Config cfg;
  cfg.tire_stiffness_factor = 0.64;
  cfg.steer_ratio = 15.7;
  Planner svc(cfg);

  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
  EXPECT_FALSE(svc.usingLearnedParams());

  svc.setLearnedParams(true, 1.20, 16.4, 0.7);
  EXPECT_TRUE(svc.usingLearnedParams());
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 1.20);
  EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 16.4);
  EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.7);

  svc.setLearnedParams(false, 1.20, 16.4, 0.7);
  EXPECT_FALSE(svc.usingLearnedParams());
  EXPECT_DOUBLE_EQ(svc.effectiveStiffnessFactor(), 0.64);
  EXPECT_DOUBLE_EQ(svc.effectiveSteerRatio(), 15.7);
  EXPECT_DOUBLE_EQ(svc.effectiveAngleOffsetDeg(), 0.0);
}

TEST(LearnedParams, NonsenseIsRefusedEvenWhenItArrivesFlaggedValid)
{
  Planner::Config cfg;
  Planner svc(cfg);
  svc.setLearnedParams(true, 0.0, 16.4, 0.0);
  EXPECT_FALSE(svc.usingLearnedParams());
  svc.setLearnedParams(true, 0.8, 0.0, 0.0);
  EXPECT_FALSE(svc.usingLearnedParams());
}

TEST(ParamsLearner, TheWrongSteeringSignRunsItIntoTheBounds)
{
  adas::ParamsLearner::Config bad = movable();
  bad.steer_sign = -1.0;
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
  EXPECT_GT(std::abs(flipped.stiffnessFactor() - 1.0), 0.3) << "a wrong sign must spoil the estimate";

  const auto right = learn(1.0, 15.7, 0.0, 0.0, false, truth_cfg);
  EXPECT_NEAR(right.stiffnessFactor(), 1.0, 0.25) << "with the right sign the same data yields the truth";
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

  // The learner and the reading of its result are no longer switchable: one check remains — the road
  // bank inside the learner is still off, because no drive has verified it.
  EXPECT_FALSE(cfg.localization.params.use_roll);
}
TEST(ShippedConfig, TheLearnerStartsFromTheParametersTheControllerUses)
{
  const char* env = std::getenv("ADAS_CONFIG_UNDER_TEST");
  const char* path = env != nullptr ? env : ADAS_SHIPPED_CONFIG_JSON;
  bool ok = false;
  const AdasApp::Config cfg = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok) << "cannot parse " << path;

  const auto& pl = cfg.localization.params;
  EXPECT_DOUBLE_EQ(pl.vehicle.wheelbase_m, cfg.localization.wheelbase_m);
  EXPECT_DOUBLE_EQ(pl.vehicle.tire_stiffness_factor, cfg.lane_keep.tire_stiffness_factor);
  EXPECT_DOUBLE_EQ(pl.stiffness_init, cfg.lane_keep.tire_stiffness_factor);
  EXPECT_DOUBLE_EQ(pl.steer_ratio_init, cfg.lane_keep.steer_ratio);
}

namespace {
class ChassisPublisher : public adas::middleware::Service {
public:
  std::string_view getName() const override { return "chassis_pub"; }
  void configure() override {}
  void send(const adas::ChassisSample& m) { publish(adas::topics::kVehicleState, adas::carStateFromChassis(m)); }
};

}  // namespace

TEST(LearnedParams, TheServiceActuallyLearnsFromChassisMessages)
{
  adas::services::Localization::Config cfg;
  cfg.params.steer_sign = -1.0;
  cfg.params.use_roll = false;
  const double truth_tsf = 1.4;
  cfg.params.stiffness_p0_std = 0.1;

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
