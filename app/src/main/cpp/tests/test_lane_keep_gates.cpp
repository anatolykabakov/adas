#include <gtest/gtest.h>

#include "adas/lateral/slew_guard.h"
#include "adas/utils/lane_keep_gates.h"
#include "adas/utils/interval_filter.h"

using adas::AssistGate;
using adas::BlinkerGate;
using adas::HysteresisGate;
using adas::IntervalFilter;
using adas::StaleGate;
using adas::lateral::SlewGuard;

TEST(IntervalFilterTest, GapReturnsToNominalButChassisHoldsIts)
{
  const IntervalFilter::Config cfg{0.05, 0.02, 0.5, 0.5, 0.3};

  IntervalFilter vision(cfg);
  int64_t t = 1'000'000;
  for (int i = 0; i < 40; ++i)
    vision.update(t += 30'000);
  EXPECT_NEAR(0.03, vision.value(), 1e-6) << "a steady stream converges to its own interval";
  const double before_gap = vision.value();

  // A one-second gap is a break in the stream, not a slow frame.
  EXPECT_NEAR(0.05, vision.update(t += 1'000'000), 1e-12);
  EXPECT_NE(before_gap, vision.value());

  IntervalFilter chassis(cfg);
  int64_t c = 1'000'000;
  for (int i = 0; i < 40; ++i)
    chassis.update(c += 30'000);
  const double held = chassis.value();
  EXPECT_DOUBLE_EQ(held, chassis.updateHoldingOnGap(c += 1'000'000)) << "a dropout must not set the rate";
}

TEST(IntervalFilterTest, UntimestampedSourceDoesNotSetTheStep)
{
  IntervalFilter f({0.09, 0.02, 0.5, 0.5, 0.3});
  EXPECT_FALSE(f.primed());
  f.update(1'000'000);
  f.update(1'030'000);
  EXPECT_LT(f.value(), 0.09);
  EXPECT_NEAR(0.09, f.update(0), 1e-12) << "a message without a timestamp falls back to the nominal";
}

TEST(HysteresisGateTest, OpensHighAndClosesLow)
{
  HysteresisGate gate(1.5, 2.0);
  EXPECT_FALSE(gate.update(1.8)) << "between the thresholds a closed gate stays closed";
  EXPECT_TRUE(gate.update(2.1));
  EXPECT_TRUE(gate.update(1.8)) << "between the thresholds an open gate stays open";
  EXPECT_FALSE(gate.update(1.4));
}

TEST(StaleGateTest, ZeroAgeDisablesTheRule)
{
  StaleGate gate;
  double age = 0.0;
  EXPECT_TRUE(gate.update(10'000'000, 5'000'000, 0.0, age)) << "a zero threshold disables the rule";
  EXPECT_TRUE(gate.update(10'000'000, 0, 0.3, age)) << "lines without a timestamp cannot go stale";
}

TEST(StaleGateTest, ReportsOnlyTheTransitions)
{
  StaleGate gate;
  double age = 0.0;
  ASSERT_TRUE(gate.update(10'000'000, 10'000'000, 0.3, age));
  EXPECT_FALSE(gate.justChanged());

  EXPECT_FALSE(gate.update(10'000'000, 9'000'000, 0.3, age));
  EXPECT_TRUE(gate.justChanged()) << "the first staleness is an event";
  EXPECT_NEAR(1.0, age, 1e-9);

  EXPECT_FALSE(gate.update(10'000'000, 9'000'000, 0.3, age));
  EXPECT_FALSE(gate.justChanged()) << "the second one in a row is not";

  EXPECT_TRUE(gate.update(10'000'000, 10'000'000, 0.3, age));
  EXPECT_TRUE(gate.justChanged());
}

TEST(BlinkerGateTest, HoldsThroughTheResumeDelayAndClearsItself)
{
  BlinkerGate gate;
  int64_t t = 1'000'000;

  EXPECT_FALSE(gate.update(t, false, false, 0.2).suppressed);

  auto r = gate.update(t += 100'000, true, false, 0.2);
  EXPECT_TRUE(r.suppressed);
  EXPECT_TRUE(r.changed);
  EXPECT_TRUE(r.left);

  EXPECT_FALSE(gate.update(t += 100'000, true, false, 0.2).changed) << "a held signal is not an event";

  // Release: hold for as long as the car takes to cross the line.
  EXPECT_TRUE(gate.update(t += 100'000, false, false, 0.2).suppressed);
  r = gate.update(t += 250'000, false, false, 0.2);
  EXPECT_FALSE(r.suppressed);
  EXPECT_TRUE(r.changed);
  EXPECT_GT(r.since_off_s, 0.2);
}

TEST(AssistGateTest, NeverHeardMeansOpenButUnknown)
{
  AssistGate gate;
  const auto r = gate.update(10'000'000, 0.5);
  EXPECT_TRUE(r.allowed) << "no panda in the loop — an offline harness must still steer";
  EXPECT_FALSE(r.known);
}

TEST(AssistGateTest, HeardAndLostMeansClosed)
{
  AssistGate gate;
  gate.onReport(true, 10'000'000);
  auto r = gate.update(10'100'000, 0.5);
  EXPECT_TRUE(r.allowed);
  EXPECT_TRUE(r.known);

  r = gate.update(11'000'000, 0.5);
  EXPECT_FALSE(r.allowed) << "the device went silent, so it is not passing torque either";
  EXPECT_FALSE(r.known);
  EXPECT_TRUE(r.changed);
}

TEST(SlewGuardTest, ClipsTheJumpAndForgetsAfterAGap)
{
  SlewGuard guard({8.0, 5.0, 2.0, 2.67, 2});

  double steer = 0.0;
  EXPECT_FALSE(guard.apply(steer, 14.0, 0.05, true)) << "the first frame has nothing to compare against";

  steer = 1.0;
  EXPECT_TRUE(guard.apply(steer, 14.0, 0.05, true));
  EXPECT_LT(steer, 1.0);
  const double after_first = steer;

  double ignored = after_first;
  guard.apply(ignored, 14.0, 0.05, false);
  guard.apply(ignored, 14.0, 0.05, false);
  guard.apply(ignored, 14.0, 0.05, false);

  steer = 1.0;
  EXPECT_FALSE(guard.apply(steer, 14.0, 0.05, true)) << "after a gap there is no previous value to limit against";
  EXPECT_DOUBLE_EQ(1.0, steer);
}

TEST(SlewGuardTest, ZeroLimitDisablesIt)
{
  SlewGuard guard({0.0, 5.0, 2.0, 2.67, 10});
  double steer = 0.0;
  guard.apply(steer, 14.0, 0.05, true);
  steer = 1.0;
  EXPECT_FALSE(guard.apply(steer, 14.0, 0.05, true));
  EXPECT_DOUBLE_EQ(1.0, steer);
}
