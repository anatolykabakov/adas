#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "adas/longitudinal/long_control.h"
#include "adas/longitudinal/long_mpc.h"
#include "adas/longitudinal/long_planner.h"
#include "adas/utils/curvature_preview.h"

using adas::curvatureSpeedLimit;
using adas::maxCurvatureAhead;
using adas::Vec2;
namespace lg = adas::longitudinal;

namespace {
std::vector<Vec2> arc(double radius, double length_m, double step = 1.0)
{
  std::vector<Vec2> pts;
  for (double s = 0.0; s <= length_m; s += step) {
    const double th = s / radius;
    pts.push_back({radius * std::sin(th), radius * (1.0 - std::cos(th))});
  }
  return pts;
}

std::vector<Vec2> straight(double length_m, double step = 1.0)
{
  std::vector<Vec2> pts;
  for (double s = 0.0; s <= length_m; s += step)
    pts.push_back({s, 0.0});
  return pts;
}

/// Drive the planner for `seconds` against a lead moving at constant speed; return the last output.
lg::PlannerOutput runFollow(lg::Planner& planner, double v_ego0, double gap0, double v_lead, double v_cruise,
                            double seconds, double* min_gap = nullptr, double* max_decel = nullptr,
                            double* v_ego_out = nullptr, double* gap_out = nullptr)
{
  const double dt = planner.config().dt_s;
  double v_ego = v_ego0;
  double gap = gap0;
  lg::PlannerOutput out;
  bool first = true;
  for (double t = 0.0; t < seconds; t += dt) {
    lg::PlannerInput in;
    in.v_ego = v_ego;
    in.a_ego = out.a_target;
    in.v_cruise_mps = v_cruise;
    in.reset_state = first;
    first = false;
    in.lead0.valid = true;
    in.lead0.d_rel = gap;
    in.lead0.v_lead = v_lead;
    in.lead0.prob = 0.95;
    out = planner.update(in);
    // The car follows the plan exactly: the planner's own model of itself is the plant here.
    v_ego = std::max(0.0, v_ego + out.a_target * dt);
    gap += (v_lead - v_ego) * dt;
    if (min_gap)
      *min_gap = std::min(*min_gap, gap);
    if (max_decel)
      *max_decel = std::min(*max_decel, out.a_target);
  }
  if (v_ego_out)
    *v_ego_out = v_ego;
  if (gap_out)
    *gap_out = gap;
  return out;
}

}  // namespace

TEST(CurvaturePreview, StraightAsksForNoLimit)
{
  const double k = maxCurvatureAhead(straight(80.0), 10.0, 70.0);
  EXPECT_NEAR(0.0, k, 1e-4);
  EXPECT_DOUBLE_EQ(27.0, curvatureSpeedLimit(k, 1.8, 27.0));
}

TEST(CurvaturePreview, RecoversArcCurvature)
{
  for (const double radius : {120.0, 200.0, 400.0}) {
    const double k = maxCurvatureAhead(arc(radius, 90.0), 10.0, 80.0);
    EXPECT_NEAR(1.0 / radius, k, 0.15 / radius) << "radius " << radius;
  }
}

TEST(CurvaturePreview, SpeedLimitMatchesLateralAcceleration)
{
  EXPECT_NEAR(19.0, curvatureSpeedLimit(1.0 / 200.0, 1.8, 40.0), 0.2);
  EXPECT_NEAR(8.6, curvatureSpeedLimit(1.0 / 41.0, 1.8, 40.0), 0.2);
}

// --- the MPC ---------------------------------------------------------------------------------------

TEST(LongMpc, GridAndDistanceFormulasMatchUpstream)
{
  const auto& T = lg::mpcTimes();
  EXPECT_DOUBLE_EQ(0.0, T[0]);
  EXPECT_NEAR(2.5, T[6], 1e-9);
  EXPECT_NEAR(10.0, T[12], 1e-9);
  EXPECT_NEAR(2.5, lg::modelTimes()[16], 1e-9);

  lg::LongMpcConfig cfg;
  // v²/5 + 1.45 v + 6 at 20 m/s
  EXPECT_NEAR(80.0 + 29.0 + 6.0, lg::safeObstacleDistance(20.0, cfg, cfg.t_follow), 1e-9);
  EXPECT_NEAR(80.0, lg::stoppedEquivalenceFactor(20.0, cfg), 1e-9);
  // Same speed: the desired gap is the time gap plus the standstill gap.
  EXPECT_NEAR(1.45 * 20.0 + 6.0, lg::desiredFollowDistance(20.0, 20.0, cfg), 1e-9);
}

TEST(LongMpc, LeadExtrapolationDecaysAccelerationAndNeverReverses)
{
  const auto traj = lg::extrapolateLead(30.0, 5.0, -3.0, 1.5);
  EXPECT_DOUBLE_EQ(30.0, traj.x[0]);
  for (int i = 0; i < lg::kMpcNodes; ++i) {
    EXPECT_GE(traj.v[i], 0.0);
    if (i > 0)
      EXPECT_GE(traj.x[i], traj.x[i - 1]);
  }
  // The braking decays as exp(−τt²/2): the lead loses about a·√(π/2τ) ≈ 3.1 m/s, not all of it.
  EXPECT_LT(traj.v[lg::kMpcNodes - 1], 3.0);
  EXPECT_GT(traj.v[lg::kMpcNodes - 1], 1.5);
}

TEST(LongMpc, CruiseAloneAcceleratesTowardTheSetSpeedInsideTheLimits)
{
  lg::LongMpc mpc;
  mpc.setAccelLimits(-1.2, 1.2);
  // One second of ticks, the car following its own plan: the first solve is soft because the jerk
  // cost starts it from rest, the warm-started ones build the acceleration up to the ceiling.
  double v = 10.0, a = 0.0;
  for (int tick = 0; tick < 20; ++tick) {
    mpc.setCurState(v, a);
    mpc.setWeights(tick > 0);
    ASSERT_TRUE(mpc.update(std::nullopt, std::nullopt, 25.0, false));
    a = lg::interp(0.05, lg::mpcTimes().data(), mpc.aSolution().data(), lg::kMpcNodes);
    v += 0.05 * a;
  }
  EXPECT_EQ("cruise", mpc.source());
  EXPECT_GT(a, 0.8);
  for (int i = 0; i < lg::kMpcN; ++i)  // the terminal node carries no path constraint, as upstream
    EXPECT_LE(mpc.aSolution()[i], 1.2 + 0.05) << "node " << i;
  // Speed rises monotonically toward the set speed and never overshoots it by more than a hair.
  for (int i = 1; i < lg::kMpcNodes; ++i)
    EXPECT_GE(mpc.vSolution()[i], mpc.vSolution()[i - 1] - 1e-6);
  EXPECT_LE(mpc.vSolution()[lg::kMpcNodes - 1], 25.5);
}

TEST(LongMpc, ALeadBelowTheSetSpeedBoundsThePlan)
{
  lg::LongMpc mpc;
  mpc.setAccelLimits(-1.2, 1.2);
  mpc.setCurState(20.0, 0.0);
  // A fresh engage: no previous acceleration to stay near (with one, A_CHANGE_COST = 200 eases the
  // first second on purpose — upstream's plan never jumps away from what it asked a tick ago).
  mpc.setWeights(false);
  const auto lead = lg::extrapolateLead(35.0, 15.0, 0.0, 1.5);
  ASSERT_TRUE(mpc.update(lead, std::nullopt, 30.0, true));
  EXPECT_EQ("lead0", mpc.source());
  // Closing on a slower lead from 35 m: the plan brakes — within the first second, jerk-limited.
  double a_first_second = 0.0;
  for (int i = 0; i < lg::kMpcNodes && lg::mpcTimes()[i] <= 1.2; ++i)
    a_first_second = std::min(a_first_second, mpc.aSolution()[i]);
  EXPECT_LT(a_first_second, -0.3);
  // ...and never plans into the lead.
  for (int i = 0; i < lg::kMpcNodes; ++i)
    EXPECT_LT(mpc.xSolution()[i], lead.x[i]) << "node " << i;
  // By the end of the horizon the plan has settled near the lead's speed.
  EXPECT_NEAR(15.0, mpc.vSolution()[lg::kMpcNodes - 1], 2.0);
}

TEST(LongMpc, HardBrakingLeadIsAnsweredBeyondTheComfortLimit)
{
  lg::LongMpc mpc;
  mpc.setAccelLimits(-1.2, 1.2);
  mpc.setCurState(25.0, 0.0);
  mpc.setWeights(true);
  // Lead 25 m ahead braking at −4 m/s² from 25 m/s.
  const auto lead = lg::extrapolateLead(25.0, 25.0, -4.0, 0.2);
  ASSERT_TRUE(mpc.update(lead, std::nullopt, 30.0, true));
  double a_min = 0.0;
  for (int i = 0; i < lg::kMpcNodes; ++i)
    a_min = std::min(a_min, mpc.aSolution()[i]);
  EXPECT_LT(a_min, -1.2) << "the comfort limit is for cruising; a braking lead gets the hard one";
  EXPECT_GE(a_min, -3.5 - 0.3);
}

TEST(LongMpc, WarmStartConvergesFasterThanColdStart)
{
  lg::LongMpc mpc;
  mpc.setAccelLimits(-1.2, 1.2);
  mpc.setCurState(15.0, 0.0);
  mpc.setWeights(true);
  ASSERT_TRUE(mpc.update(std::nullopt, std::nullopt, 25.0, false));
  const int cold = mpc.iterations();
  mpc.setCurState(15.05, mpc.aSolution()[1]);
  ASSERT_TRUE(mpc.update(std::nullopt, std::nullopt, 25.0, false));
  EXPECT_LE(mpc.iterations(), cold);
}

// --- the planner around it ---------------------------------------------------------------------------

TEST(LongPlanner, NoSetSpeedMeansNoPlan)
{
  lg::Planner planner;
  lg::PlannerInput in;
  in.v_ego = 15.0;
  in.v_cruise_mps = 0.0;
  const auto out = planner.update(in);
  EXPECT_FALSE(out.valid);
  EXPECT_EQ("no_cruise", out.status);
}

TEST(LongPlanner, AccelCeilingFallsWithSpeed)
{
  lg::Planner planner;
  EXPECT_NEAR(1.6, planner.maxAccel(0.0), 1e-9);
  EXPECT_NEAR(1.2, planner.maxAccel(10.0), 1e-9);
  EXPECT_NEAR(0.6, planner.maxAccel(50.0), 1e-9);
  // In a turn the longitudinal budget shrinks: 20 m/s at 60° of wheel is nearly all lateral.
  const auto lim = planner.limitAccelInTurns(20.0, 60.0, {-1.2, 1.2});
  EXPECT_LT(lim[1], 0.5);
  EXPECT_DOUBLE_EQ(-1.2, lim[0]);
}

TEST(LongPlanner, ReachesTheSetSpeedOnAnEmptyRoad)
{
  lg::Planner planner;
  double v_ego = 0.0;
  const double dt = planner.config().dt_s;
  lg::PlannerOutput out;
  double a_max_seen = 0.0;
  for (double t = 0.0; t < 40.0; t += dt) {
    lg::PlannerInput in;
    in.v_ego = v_ego;
    in.a_ego = out.a_target;
    in.v_cruise_mps = 20.0;
    in.reset_state = t == 0.0;
    out = planner.update(in);
    ASSERT_TRUE(out.valid);
    a_max_seen = std::max(a_max_seen, out.a_target);
    v_ego = std::max(0.0, v_ego + out.a_target * dt);
  }
  EXPECT_NEAR(20.0, v_ego, 0.5);
  EXPECT_LE(a_max_seen, 1.6 + 0.1);
  EXPECT_TRUE(out.source == "cruise" || out.source == "lead0") << out.source;
}

TEST(LongPlanner, SettlesAtTheDesiredGapBehindASlowerLead)
{
  lg::Planner planner;
  double min_gap = 1e9, max_decel = 0.0, v_ego = 0.0, gap = 0.0;
  // Closing at 10 m/s from 60 m — a cut-in-like start, deep inside the desired distance.
  const auto out = runFollow(planner, 25.0, 60.0, 15.0, 30.0, 60.0, &min_gap, &max_decel, &v_ego, &gap);
  ASSERT_TRUE(out.valid);
  EXPECT_TRUE(out.has_lead);
  EXPECT_NEAR(15.0, v_ego, 0.5);
  // Never close: the gap bottoms out well clear of the lead, with the hard limit never needed.
  EXPECT_GT(min_gap, 15.0);
  EXPECT_GE(max_decel, -3.5);
  EXPECT_LT(max_decel, -1.0) << "closing at 10 m/s from 60 m has to brake harder than the comfort limit";
  // And settles at upstream's desired following distance for 15 m/s: 1.45 s + 6 m.
  const double want = lg::desiredFollowDistance(15.0, 15.0, planner.config().mpc);
  EXPECT_NEAR(want, gap, 0.15 * want);
}

TEST(LongPlanner, StopsBehindAStoppedLeadAtTheStandstillGap)
{
  lg::Planner planner;
  double min_gap = 1e9, max_decel = 0.0, v_ego = 0.0;
  runFollow(planner, 15.0, 80.0, 0.0, 20.0, 40.0, &min_gap, &max_decel, &v_ego);
  EXPECT_NEAR(0.0, v_ego, 0.3);
  EXPECT_GT(min_gap, 3.0);
  EXPECT_LT(min_gap, 9.0);
  EXPECT_GE(max_decel, -3.5);
}

TEST(LongPlanner, ALeadInAnotherLaneIsIgnored)
{
  lg::Planner planner;
  const auto path = straight(120.0);
  lg::PlannerInput in;
  in.v_ego = 20.0;
  in.v_cruise_mps = 25.0;
  in.reset_state = true;
  in.path = &path;
  in.lead0.valid = true;
  in.lead0.d_rel = 20.0;
  in.lead0.v_lead = 5.0;
  in.lead0.prob = 0.95;
  in.lead0.y_rel = 3.5;  // one lane over
  const auto out = planner.update(in);
  EXPECT_FALSE(out.has_lead);
  EXPECT_FALSE(out.lead_in_lane);
  EXPECT_GE(out.a_target, -0.3);
}

TEST(LongPlanner, LeadSourceNoneDrivesOnTheSetSpeedAlone)
{
  lg::PlannerConfig cfg;
  cfg.lead_source = "none";
  lg::Planner planner(cfg);
  lg::PlannerInput in;
  in.v_ego = 20.0;
  in.v_cruise_mps = 25.0;
  in.reset_state = true;
  in.lead0.valid = true;
  in.lead0.d_rel = 20.0;
  in.lead0.v_lead = 5.0;
  in.lead0.prob = 0.95;
  const auto out = planner.update(in);
  ASSERT_TRUE(out.valid);
  EXPECT_FALSE(out.has_lead);
  EXPECT_EQ("none", out.lead_source);
  EXPECT_GE(out.a_target, -0.3) << "with leads switched off a car 20 m ahead is not seen";
}

TEST(LongPlanner, TheCurveAheadCapsTheSetSpeed)
{
  lg::Planner planner;
  const auto path = arc(120.0, 150.0);
  lg::PlannerInput in;
  in.v_ego = 22.0;
  in.v_cruise_mps = 30.0;
  in.reset_state = true;
  in.path = &path;
  // The curvature estimate is low-passed over 2 s so one jittery frame cannot brake the car; the same
  // bend seen for three seconds does.
  lg::PlannerOutput out;
  for (int tick = 0; tick < 60; ++tick) {
    out = planner.update(in);
    in.reset_state = false;
  }
  ASSERT_TRUE(out.valid);
  EXPECT_GT(out.v_curv, 0.0);
  EXPECT_LT(out.v_cruise, 30.0);
  EXPECT_NEAR(std::sqrt(1.8 * 120.0), out.v_cruise, 2.0);
  EXPECT_NE(std::string::npos, out.source.find("curv"));
}

TEST(CruiseSetpoint, StalkEdgesDriveTheSetSpeed)
{
  lg::PlannerConfig cfg;
  lg::CruiseSetpoint sp;
  lg::CruiseSetpoint::Buttons b;
  double t = 0.0;
  auto step = [&](double dt = 0.01) {
    t += dt;
    return sp.update(b, 20.0, true, cfg, t);
  };
  EXPECT_DOUBLE_EQ(0.0, step());
  b.set = true;
  EXPECT_NEAR(20.0, step(), 0.15);  // latched, rounded to km/h
  EXPECT_NEAR(20.0, step(), 0.15);  // held: no second press
  b.set = false;
  b.accel = true;
  step();                            // pressed — nothing yet, a tap acts on release
  b.accel = false;
  EXPECT_NEAR(20.0 + 1.0 / 3.6, step(), 0.15);
  b.decel = true;
  step();
  b.decel = false;
  EXPECT_NEAR(20.0, step(), 0.15);
  EXPECT_DOUBLE_EQ(0.0, sp.update(b, 25.0, false, cfg, t + 0.01));  // main switch off clears
  b.resume = true;
  EXPECT_NEAR(20.0, sp.update(b, 10.0, true, cfg, t + 0.02), 0.15);  // resume brings it back
}

TEST(CruiseSetpoint, HoldingTheTipStepsInTens)
{
  lg::PlannerConfig cfg;
  lg::CruiseSetpoint sp;
  lg::CruiseSetpoint::Buttons b;
  double t = 0.0;
  auto step = [&](double dt = 0.01) {
    t += dt;
    return sp.update(b, 53.0 / 3.6, true, cfg, t);
  };
  b.set = true;
  step();
  b.set = false;
  EXPECT_NEAR(53.0, step() * 3.6, 0.1);
  // Hold + for 1.2 s: long steps at 0.5 s (→ 60) and 1.0 s (→ 70), each on the next multiple of ten.
  b.accel = true;
  double kph = 0.0;
  for (int i = 0; i < 120; ++i)
    kph = step() * 3.6;
  EXPECT_NEAR(70.0, kph, 0.1);
  // Releasing after a long press adds no short step.
  b.accel = false;
  EXPECT_NEAR(70.0, step() * 3.6, 0.1);
  // A short tap of − still moves one km/h.
  b.decel = true;
  step();
  b.decel = false;
  EXPECT_NEAR(69.0, step() * 3.6, 0.1);
  // Holding − lands on the multiple below: 69 → 60.
  b.decel = true;
  for (int i = 0; i < 55; ++i)
    kph = step() * 3.6;
  EXPECT_NEAR(60.0, kph, 0.1);
}

// --- the control law -------------------------------------------------------------------------------

namespace {
lg::LongControlInput planAt(double v_now, double a, double t_since = 0.0)
{
  lg::LongControlInput in;
  in.active = true;
  in.v_ego = v_now;
  in.t_since_plan_s = t_since;
  const auto& T = lg::modelTimes();
  for (int i = 0; i < lg::kControlN; ++i) {
    in.speeds[i] = std::max(0.0, v_now + a * T[i]);
    in.accels[i] = a;
  }
  return in;
}
}  // namespace

TEST(LongControl, FeedforwardCarriesTheAccelerationAndPCorrectsTheSpeed)
{
  lg::LongControl lc;
  auto in = planAt(20.0, 0.5);
  const auto out = lc.update(in);
  EXPECT_EQ(lg::LongCtrlState::Pid, out.state);
  // Read 0.15 s ahead: the speed target is v + a·delay, the acceleration comes back as the plan's.
  EXPECT_NEAR(20.0 + 0.5 * 0.15, out.v_target, 1e-6);
  EXPECT_NEAR(0.5, out.a_target, 1e-6);
  EXPECT_NEAR(0.5 + 0.1 * (0.5 * 0.15), out.accel, 1e-6);

  // Behind the plan by 2 m/s: the P term adds 0.2 m/s².
  in.v_ego = 18.0;
  const auto lag = lc.update(in);
  EXPECT_NEAR(0.5 + 0.1 * (20.075 - 18.0), lag.accel, 1e-6);
}

TEST(LongControl, StoppingThenStartingWalkTheStateMachine)
{
  lg::LongControl lc;
  // A plan that is at rest and stays at rest, car nearly stopped: stopping.
  auto stop = planAt(0.0, 0.0);
  stop.v_ego = 0.4;
  auto out = lc.update(stop);
  EXPECT_EQ(lg::LongCtrlState::Stopping, out.state);
  // The request ramps down at stoppingDecelRate toward stopAccel, never below it.
  for (int i = 0; i < 400; ++i)
    out = lc.update(stop);
  EXPECT_NEAR(-2.0, out.accel, 0.05);

  // The plan says go: starting, at startAccel, then pid once the car moves.
  auto go = planAt(0.0, 1.0);
  go.v_ego = 0.0;
  go.standstill = false;
  out = lc.update(go);
  EXPECT_EQ(lg::LongCtrlState::Starting, out.state);
  EXPECT_NEAR(1.0, out.accel, 1e-9);
  go.v_ego = 1.5;
  out = lc.update(go);
  EXPECT_EQ(lg::LongCtrlState::Pid, out.state);
}

TEST(LongControl, InactiveMeansOffAndZero)
{
  lg::LongControl lc;
  auto in = planAt(20.0, 1.0);
  in.active = false;
  const auto out = lc.update(in);
  EXPECT_EQ(lg::LongCtrlState::Off, out.state);
  EXPECT_DOUBLE_EQ(0.0, out.accel);
}

TEST(LongControl, TheEnvelopeIsNeverExceeded)
{
  lg::LongControl lc;
  const auto hi = lc.update(planAt(5.0, 5.0));
  EXPECT_LE(hi.accel, 2.0);
  const auto lo = lc.update(planAt(30.0, -8.0));
  EXPECT_GE(lo.accel, -3.5);
}
