#include <gtest/gtest.h>

#include "utils/safety_planner.hpp"

using adas::safety::computeSafetyPlan;
using adas::safety::PlannerInput;
using adas::safety::SafetyPlannerConfig;
using adas::safety::Warning;
using adas::safety::WarningLatch;

namespace {

bool has(const adas::safety::SafetyPlan& plan, Warning w)
{
  for (const auto x : plan.warnings)
    if (x == w)
      return true;
  return false;
}

bool anyForward(const adas::safety::SafetyPlan& plan) { return has(plan, Warning::FCW) || has(plan, Warning::AEB); }

PlannerInput cruising(double v_ms)
{
  PlannerInput in;
  in.ego_speed_ms = v_ms;
  in.lateral.valid = true;
  in.lateral.lane_anchored = true;
  return in;
}

}  // namespace

TEST(SafetyWarn, EmptyRoadAboveSpeedLimitIsNotAThreat)
{
  SafetyPlannerConfig cfg;
  const auto plan = computeSafetyPlan(cfg, cruising(38.0));
  EXPECT_LT(plan.acceleration_ms2, -3.0);
  EXPECT_FALSE(anyForward(plan));
}

TEST(SafetyWarn, EmptyCurveIsNotAThreat)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(22.0);
  in.lateral.kappa = 0.02;
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_LT(plan.acceleration_ms2, -3.0);
  EXPECT_FALSE(anyForward(plan));
}

TEST(SafetyWarn, SlowerLeadFarAheadIsNotAThreat)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.cipo = {true, 22.0, 60.0, 0.0};
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_TRUE(plan.threat.valid);
  EXPECT_GT(plan.threat.ttc_s, 15.0);
  EXPECT_FALSE(anyForward(plan));
}

TEST(SafetyWarn, FollowingAtSteadySpeedIsNotAThreat)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.cipo = {true, 25.0, 12.0, 0.0};
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_FALSE(plan.threat.valid);
  EXPECT_FALSE(anyForward(plan));
}

TEST(SafetyWarn, ClosingOnLeadRaisesFcwThenAeb)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);

  in.cipo = {true, 15.0, 26.0, 0.0};
  const auto fcw = computeSafetyPlan(cfg, in);
  EXPECT_TRUE(has(fcw, Warning::FCW));
  EXPECT_FALSE(has(fcw, Warning::AEB));

  in.cipo.gap_m = 14.0;
  const auto aeb = computeSafetyPlan(cfg, in);
  EXPECT_TRUE(has(aeb, Warning::AEB));
  EXPECT_FALSE(has(aeb, Warning::FCW));
}

TEST(SafetyWarn, LeadInTheNextLaneIsIgnored)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.cipo = {true, 15.0, 26.0, 3.5};
  EXPECT_FALSE(anyForward(computeSafetyPlan(cfg, in)));

  in.cipo.offset_m = 0.5;
  EXPECT_TRUE(anyForward(computeSafetyPlan(cfg, in)));
}

TEST(SafetyWarn, LeadOnACurveStaysInPath)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.kappa = 0.005;
  in.cipo = {true, 15.0, 26.0, 1.6};
  EXPECT_TRUE(anyForward(computeSafetyPlan(cfg, in)));
}

TEST(SafetyWarn, ManoeuvringSpeedDoesNotWarn)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(2.0);
  in.cipo = {true, 0.0, 2.5, 0.0};
  EXPECT_FALSE(anyForward(computeSafetyPlan(cfg, in)));
}

TEST(SafetyWarn, SteadyOffsetInsideACurveIsNotADeparture)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.cte_m = 0.6;
  in.lateral.cte_rate_ms = 0.0;
  in.lateral.kappa = 0.004;
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_FALSE(has(plan, Warning::RLDW));
  EXPECT_FALSE(has(plan, Warning::LLDW));
}

TEST(SafetyWarn, DriftingOutwardRaisesTheSideItDriftsTo)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);

  in.lateral.cte_m = 0.6;
  in.lateral.cte_rate_ms = 0.25;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::RLDW));

  in.lateral.cte_m = -0.6;
  in.lateral.cte_rate_ms = -0.25;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::LLDW));
}

TEST(SafetyWarn, ReturningToCentreDoesNotWarn)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.cte_m = 0.6;
  in.lateral.cte_rate_ms = -0.3;
  EXPECT_FALSE(has(computeSafetyPlan(cfg, in), Warning::RLDW));
}

TEST(SafetyWarn, FarPastTheLineWarnsEvenWithoutOutwardMotion)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.cte_m = 0.9;
  in.lateral.cte_rate_ms = 0.0;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::RLDW));
}

TEST(SafetyWarn, BelowLdwSpeedAndUnderDriverSteerStaysQuiet)
{
  SafetyPlannerConfig cfg;
  PlannerInput slow = cruising(8.0);
  slow.lateral.cte_m = 0.9;
  slow.lateral.cte_rate_ms = 0.3;
  EXPECT_FALSE(has(computeSafetyPlan(cfg, slow), Warning::RLDW));

  PlannerInput steering = cruising(25.0);
  steering.lateral.cte_m = 0.9;
  steering.lateral.cte_rate_ms = 0.3;
  steering.driver_steering = true;
  EXPECT_FALSE(has(computeSafetyPlan(cfg, steering), Warning::RLDW));
}

TEST(SafetyWarn, NoLaneLinesNoLaneWarning)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.lane_anchored = false;
  in.lateral.cte_m = 1.2;
  in.lateral.cte_rate_ms = 0.4;
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_FALSE(has(plan, Warning::RLDW));
  EXPECT_FALSE(has(plan, Warning::LLDW));
}

TEST(SafetyWarn, NoPathNoLaneWarning)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.valid = false;
  in.lateral.cte_m = 1.5;
  in.lateral.cte_rate_ms = 0.5;
  const auto plan = computeSafetyPlan(cfg, in);
  EXPECT_FALSE(has(plan, Warning::RLDW));
  EXPECT_FALSE(has(plan, Warning::LLDW));
}

TEST(SafetyWarn, LatchIgnoresSingleFrameSpikesAndHoldsAfterwards)
{
  WarningLatch latch(3, 10);

  EXPECT_FALSE(latch.update(true));
  EXPECT_FALSE(latch.update(false));
  EXPECT_FALSE(latch.update(true));
  EXPECT_FALSE(latch.update(true));
  EXPECT_TRUE(latch.update(true));

  for (int i = 0; i < 9; ++i)
    EXPECT_TRUE(latch.update(false)) << "dropped at quiet frame " << i;
  EXPECT_FALSE(latch.update(false));
}

TEST(SafetyWarn, SignalledLaneChangeSuppressesOnlyThatSide)
{
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.lateral.cte_m = 0.9;
  in.lateral.cte_rate_ms = 0.3;

  in.right_blinker = true;
  EXPECT_FALSE(has(computeSafetyPlan(cfg, in), Warning::RLDW));

  in.right_blinker = false;
  in.left_blinker = true;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::RLDW));
}

// Both cases below are false positives measured on run 2026_08_04_21_00_18 (23.5 min, night,
// no collisions and no unintended departures), so the correct behaviour is silence.

TEST(SafetyWarn, StopAndGoCreepDoesNotWarn)
{
  // Worst real case from that run: 4.3 m/s, lead 9.5 m nearly stationary. Arithmetically TTC ≈ 1.9 s,
  // which used to raise FCW; at that speed the gap is trivially recoverable.
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(4.3);
  in.cipo = {true, 0.3, 9.5, 0.0};
  EXPECT_FALSE(anyForward(computeSafetyPlan(cfg, in)));

  // Fastest FCW frame of that run — still below the gate.
  PlannerInput in2 = cruising(8.2);
  in2.cipo = {true, 4.4, 12.7, 0.0};
  EXPECT_FALSE(anyForward(computeSafetyPlan(cfg, in2)));
}

TEST(SafetyWarn, RealThreatAboveTheSpeedGateStillWarns)
{
  // The gate must not silence the case it exists for: highway closing on a much slower lead.
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(25.0);
  in.cipo = {true, 5.0, 20.0, 0.0};
  EXPECT_TRUE(anyForward(computeSafetyPlan(cfg, in)));
}

TEST(SafetyWarn, OurOwnSteeringSuppressesLaneDeparture)
{
  // The assistant held the car 0.54 m off centre while drifting at 0.15 m/s — that is its tracking
  // error, not a departure, and LDW warned about it in 82 % of its frames on that run.
  SafetyPlannerConfig cfg;
  PlannerInput in = cruising(18.0);
  in.lateral.cte_m = -0.54;
  in.lateral.cte_rate_ms = -0.15;

  in.lat_active = false;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::LLDW)) << "without our steering it is a real departure";

  in.lat_active = true;
  EXPECT_FALSE(has(computeSafetyPlan(cfg, in), Warning::LLDW));
  EXPECT_FALSE(has(computeSafetyPlan(cfg, in), Warning::RLDW));

  // The suppression is a policy, not a hard rule: it can be switched off.
  cfg.ldw_suppress_on_lat_active = false;
  EXPECT_TRUE(has(computeSafetyPlan(cfg, in), Warning::LLDW));
}
