#pragma once

#include <array>
#include <optional>
#include <string>

namespace adas {
namespace longitudinal {
/// Horizon: 12 intervals, 13 nodes on upstream's quadratic time grid T_IDXS_MPC = 10·(i/12)² s.
inline constexpr int kMpcN = 12;
inline constexpr int kMpcNodes = kMpcN + 1;
/// The model's own 33-point grid, 10·(i/32)² s; the plan the control law tracks is its first 17 points.
inline constexpr int kModelT = 33;
inline constexpr int kControlN = 17;

/// \return The MPC shooting-node times [s].
const std::array<double, kMpcNodes>& mpcTimes();
/// \return The model/control grid [s].
const std::array<double, kModelT>& modelTimes();

/** The cost the openpilot longitudinal MPC minimises, with its numbers. Every weight below is
 *  upstream's `long_mpc.py` constant; the tuning is theirs, the solver is ours. */
struct LongMpcConfig {
  double t_follow = 1.45;         ///< Desired time gap to the lead [s].
  double comfort_brake = 2.5;     ///< Deceleration the safe-distance formula assumes [m/s²].
  double stop_distance = 6.0;     ///< Gap kept at standstill [m].
  double x_ego_obstacle_cost = 3.0;
  double x_ego_cost = 0.0;
  double v_ego_cost = 0.0;
  double a_ego_cost = 0.0;
  double j_ego_cost = 5.0;
  double a_change_cost = 200.0;   ///< Penalty on moving away from last tick's acceleration; fades out after 1 s.
  double danger_zone_cost = 100.0;
  double limit_cost = 1e6;        ///< Slack weight on speed and acceleration bounds.
  /// Hard acceleration bounds of the plan [m/s²] (MIN_ACCEL / MAX_ACCEL). The per-tick limits from the
  /// planner shape the cruise obstacle; these bound what the solver may ever ask for.
  double min_accel = -3.5;
  double max_accel = 2.0;
  double lead_danger_factor = 0.75;
  double crash_distance = 0.25;   ///< Predicted gap below which a node counts as a crash [m].
  int max_iterations = 40;        ///< Gauss–Newton iterations per solve; warm starts need a handful.
};

/// A lead's predicted position and speed on the MPC grid.
struct LeadTrajectory {
  std::array<double, kMpcNodes> x{};
  std::array<double, kMpcNodes> v{};
};

/// Safe following distance at ego speed: what ego needs to stop plus the time gap plus the standstill gap.
double safeObstacleDistance(double v_ego, const LongMpcConfig& cfg, double t_follow);
/// How much further a moving lead's stopping point is than its current position.
double stoppedEquivalenceFactor(double v_lead, const LongMpcConfig& cfg);
/// The gap the plan settles at behind a lead going at `v_lead`.
double desiredFollowDistance(double v_ego, double v_lead, const LongMpcConfig& cfg);
/// Predict a lead over the horizon: its acceleration decays as exp(−τt²/2), speed never goes negative.
LeadTrajectory extrapolateLead(double x_lead, double v_lead, double a_lead, double a_lead_tau);

/**
 * \brief Longitudinal MPC: a triple integrator (position, speed, acceleration) driven by jerk, minimising
 *        upstream's cost over 12 intervals.
 *
 * The problem is small — twelve unknowns, a linear plant, a cost that is nonlinear only through the
 * safe-distance term — so it is solved here by dense Gauss–Newton with Levenberg damping and the soft
 * constraints as squared penalties, warm-started from the previous tick. That is a different solver
 * from upstream's acados SQP-RTI on the same problem; the formulation, weights and grid are theirs.
 */
class LongMpc {
public:
  explicit LongMpc(LongMpcConfig cfg = {});

  /// Forget the previous solution: after a crash reset, a long standstill, or an engage.
  void reset();
  /// This tick's comfort limits [m/s²]: the lower one shapes the cruise obstacle, the upper one is the ceiling.
  void setAccelLimits(double a_min, double a_max);
  /// Ego state at node 0: speed and acceleration. Position is always 0.
  void setCurState(double v_ego, double a_ego);
  /// Whether last tick's acceleration is a reference to stay near (false right after a reset or at standstill).
  void setWeights(bool prev_accel_constraint);

  /**
   * \brief Solve for this tick.
   * \param lead0 Nearest lead (or nothing); \param lead1 the one behind it.
   * \param v_cruise Set speed [m/s]; the "cruise obstacle" is where a car at that speed would be.
   * \param lead0_believed True when the nearest lead is believed strongly enough to count crashes against.
   * \return True when the solve converged to finite numbers; false means the state was reset.
   */
  bool update(const std::optional<LeadTrajectory>& lead0, const std::optional<LeadTrajectory>& lead1, double v_cruise,
              bool lead0_believed);

  const std::array<double, kMpcNodes>& xSolution() const { return x_sol_; }
  const std::array<double, kMpcNodes>& vSolution() const { return v_sol_; }
  const std::array<double, kMpcNodes>& aSolution() const { return a_sol_; }
  const std::array<double, kMpcN>& jSolution() const { return j_sol_; }
  /// Which obstacle bound the plan at node 0: "lead0", "lead1" or "cruise".
  const std::string& source() const { return source_; }
  /// Consecutive ticks in which the plan still predicted a crash inside 5 s.
  int crashCount() const { return crash_cnt_; }
  int iterations() const { return iters_; }
  double cost() const { return cost_; }
  const LongMpcConfig& config() const { return cfg_; }

private:
  struct Params {
    double a_min = -1.2;
    double a_max = 1.2;
    std::array<double, kMpcNodes> x_obstacle{};
    std::array<double, kMpcNodes> prev_a{};
    double lead_danger_factor = 0.75;
  };
  void rollout(const std::array<double, kMpcN>& u, std::array<double, kMpcNodes>& x, std::array<double, kMpcNodes>& v,
               std::array<double, kMpcNodes>& a) const;
  double residuals(const std::array<double, kMpcN>& u, const Params& p, double* r, double* J) const;
  bool solve(const Params& p);

  LongMpcConfig cfg_;
  double x0_v_ = 0.0;
  double x0_a_ = 0.0;
  double a_min_ = -1.2;
  double a_max_ = 1.2;
  bool prev_accel_constraint_ = true;
  std::array<double, kMpcN> u_{};
  std::array<double, kMpcNodes> x_sol_{};
  std::array<double, kMpcNodes> v_sol_{};
  std::array<double, kMpcNodes> a_sol_{};
  std::array<double, kMpcN> j_sol_{};
  std::array<double, kMpcNodes> prev_a_{};
  std::string source_ = "cruise";
  int crash_cnt_ = 0;
  int iters_ = 0;
  double cost_ = 0.0;
};

/// Linear interpolation with clamped ends, the `interp` every planner file leans on.
double interp(double x, const double* xp, const double* fp, int n);

}  // namespace longitudinal
}  // namespace adas
