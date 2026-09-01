#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "adas/longitudinal/long_mpc.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace longitudinal {
/** The longitudinal planner's settings: upstream `longitudinal_planner.py` numbers plus the two things
 *  this stack adds — a curvature preview on the fused path and a camera-to-bumper offset for leads. */
struct PlannerConfig {
  LongMpcConfig mpc{};

  double a_cruise_min = -1.2;  ///< Deceleration the plan may ask for when following or cruising [m/s²].
  /// Acceleration ceiling by speed: 1.6 m/s² from rest, 0.6 at 40 m/s.
  std::vector<double> a_cruise_max_bp{0.0, 10.0, 25.0, 40.0};
  std::vector<double> a_cruise_max_v{1.6, 1.2, 0.8, 0.6};
  /// Total (lateral + longitudinal) acceleration budget by speed, for the in-turn limit.
  std::vector<double> a_total_max_bp{20.0, 40.0};
  std::vector<double> a_total_max_v{1.7, 3.2};

  /// Where leads come from: "vision" (the model's lead heads — the only source this stack has),
  /// "none" (plan on the set speed alone), "radar" (reserved: no platform decodes radar objects yet, it
  /// falls back to vision and says so). Named in the config so the limitation lives in code, not lore.
  std::string lead_source = "vision";
  double lead_prob_thresh = 0.5;     ///< Model confidence needed before a lead is believed at all.
  double lead_fcw_prob = 0.9;        ///< Confidence needed before a predicted crash counts as one.
  double lead_max_offset_m = 2.0;    ///< Lateral offset from our path beyond which a lead is in another lane.
  double lead_accel_tau = 1.5;       ///< How fast a lead's acceleration is assumed to decay (s⁻²).
  double lead_origin_offset_m = 1.4; ///< Camera → front bumper [m]: model distances are from the lens.

  double v_cruise_min_mps = 8.0 / 3.6;    ///< Lowest set speed the stalk accepts [m/s].
  double v_cruise_max_mps = 145.0 / 3.6;  ///< Highest [m/s].
  double cruise_short_step_kph = 1.0;     ///< A tap of +/− moves the set speed by this [km/h].
  double cruise_long_press_s = 0.5;       ///< Held longer than this, +/− repeats in long steps.
  double cruise_long_step_kph = 10.0;     ///< Each long step lands on the next multiple of this [km/h].

  double v_filter_rc_s = 2.0;   ///< Time constant of the desired-speed filter [s].
  double dt_s = 0.05;           ///< Planner tick [s]; the state is integrated forward by this.
  double actuator_delay_s = 0.15;  ///< Longitudinal actuator delay the control law compensates [s].

  /// Curvature preview on the fused path: cap the set speed so the curve ahead stays under `curv_a_lat_max`.
  bool curv_limit_enabled = true;
  double curv_a_lat_max = 1.8;      ///< Lateral acceleration allowed in the curve ahead [m/s²].
  double curv_preview_s = 4.0;      ///< How far ahead the curve is looked for [s of travel].
  double curv_min_speed_ms = 8.0;   ///< Below this speed the preview does not limit [m/s].
  double curv_v_floor_ms = 8.0;     ///< The preview never asks for less than this [m/s].
  /// The curvature estimate is a fit through a noisy path; it is low-passed over this many seconds and
  /// ignored below the deadband, so one jittery frame cannot brake the car.
  double curv_filter_tau_s = 2.0;
  /// Radius above which the preview does not act: a fit through 0.15 m of path noise reads as R ≈ 300 m on
  /// a straight, so the cap only starts where the lateral acceleration would actually matter.
  double curv_deadband = 1.0 / 250.0;

  double steer_ratio = 15.6;   ///< For the in-turn limit: wheel angle → road-wheel angle.
  double wheelbase_m = 2.636;  ///< For the in-turn limit.
};

/// A lead as the model reports it, already in metres from the front bumper.
struct Lead {
  bool valid = false;
  double d_rel = 0.0;
  double y_rel = 0.0;
  double v_lead = 0.0;
  double a_lead = 0.0;
  double prob = 0.0;
};

struct PlannerInput {
  double v_ego = 0.0;
  double a_ego = 0.0;
  double steering_angle_deg = 0.0;
  bool standstill = false;
  bool reset_state = false;     ///< Engage or mode change: start the filtered state from the car's.
  double v_cruise_mps = 0.0;    ///< Set speed [m/s]; ≤ 0 means no set speed, the plan holds.
  Lead lead0{};
  Lead lead1{};
  const std::vector<Vec2>* path = nullptr;  ///< Fused reference path, for the curvature preview.
};

struct PlannerOutput {
  bool valid = false;                      ///< False when there is no set speed to plan for.
  std::array<double, kControlN> speeds{};  ///< On modelTimes()[:17].
  std::array<double, kControlN> accels{};
  std::array<double, kControlN> jerks{};
  double v_target = 0.0;   ///< speeds[0] — the desired speed now.
  double a_target = 0.0;   ///< Acceleration desired now, after the tick's integration.
  double v_cruise = 0.0;   ///< The set speed the plan was made for, after the curvature preview.
  double v_curv = 0.0;     ///< Curvature-preview speed cap (0 when not limiting).
  double kappa_ahead = 0.0;
  bool has_lead = false;
  bool lead_in_lane = true;
  bool fcw = false;
  std::string source = "cruise";  ///< lead0 | lead1 | cruise, "+curv" when the preview capped the set speed.
  std::string lead_source = "vision";  ///< Which lead source the plan was made with.
  std::string status = "ok";
  bool solver_ok = false;
  int solver_iters = 0;
  double solver_cost = 0.0;
};

/**
 * \brief upstream's `LongitudinalPlanner` in its `acc` mode: filter the desired speed, bound the
 *        acceleration by speed and by the turn, predict the leads, ask the MPC, integrate one tick.
 */
class Planner {
public:
  explicit Planner(PlannerConfig cfg = {});

  PlannerOutput update(const PlannerInput& in);
  void reset(double v_ego, double a_ego);

  const PlannerConfig& config() const { return cfg_; }
  PlannerConfig& config() { return cfg_; }
  const LongMpc& mpc() const { return mpc_; }

  /// Acceleration ceiling for this speed (A_CRUISE_MAX interpolation).
  double maxAccel(double v_ego) const;
  /// The in-turn limit: what is left for longitudinal acceleration after the lateral one.
  std::array<double, 2> limitAccelInTurns(double v_ego, double steering_angle_deg,
                                          std::array<double, 2> limits) const;
  /// Model lead → predicted trajectory, or the fast fake lead the MPC uses when there is none.
  std::optional<LeadTrajectory> processLead(const Lead& lead, double v_ego) const;

private:
  PlannerConfig cfg_;
  LongMpc mpc_;
  double v_desired_ = 0.0;
  double a_desired_ = 0.0;
  double kappa_filt_ = 0.0;
  bool initialized_ = false;
};

/** The set speed as the stalk drives it: Set latches the current speed, Resume brings the last one back.
 *  A tap of +/− (released within `cruise_long_press_s`) moves it one short step; holding the button
 *  repeats long steps every `cruise_long_press_s`, each landing on the next multiple of the long step,
 *  and the release after a long press adds nothing — upstream's VCruiseHelper, in km/h. */
class CruiseSetpoint {
public:
  struct Buttons {
    bool set = false;
    bool resume = false;
    bool accel = false;
    bool decel = false;
    bool cancel = false;
  };
  /// \param available The ACC main switch is on; off clears the set speed.
  /// \param now_s Time of this button sample [s]; presses are timed against it.
  /// \return The set speed [m/s], 0 when there is none.
  double update(const Buttons& now, double v_ego, bool available, const PlannerConfig& cfg, double now_s);
  /// Set it from outside (a parameter, the simulator); 0 clears.
  void set(double v_mps) { v_cruise_ = v_mps > 0 ? v_mps : 0.0; }
  double value() const { return v_cruise_; }

private:
  /// One +/− button's press timing.
  struct Press {
    double since_s = 0.0;
    double last_long_s = 0.0;
    bool long_fired = false;
  };
  void tip(bool held, bool was_held, Press& p, int direction, const PlannerConfig& cfg, double now_s);

  Buttons prev_{};
  Press accel_{};
  Press decel_{};
  double v_cruise_ = 0.0;
  double v_last_ = 0.0;
};

}  // namespace longitudinal
}  // namespace adas
