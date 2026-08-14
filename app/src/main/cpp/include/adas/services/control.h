#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/lateral/angle_control.hpp"
#include "adas/lateral/limits.hpp"
#include "adas/utils/lane_keep_gates.h"

namespace adas {
namespace services {
/**
 * \brief The control law and nothing else.
 *
 * \details Three services divide the lateral loop: the planner produces a plan in curvature, this
 * controller turns it into a command, and the platform carries the command onto the bus. Nothing here
 * knows about CAN — not an address, not a signal, not a frame counter. The controller sees the chassis,
 * the plan and the panda's report, and emits an intent: torque, whether steering is engaged, the HUD
 * pictograms, and the wish to press a cruise button. How that reaches the bus is the platform's problem.
 *
 * With no hardware in the picture, the controller is testable against a recording: feed it a chassis
 * frame and a plan, and compare its command with a reference log.
 */
class Control : public adas::middleware::Service {
public:
  struct Config {
    /// Drive the longitudinal axis by pressing the stock cruise buttons. Off means lateral only.
    bool cruise_buttons_enabled = false;
    /// Speed error below which no button is pressed [m/s]; keeps the setpoint from hunting.
    double cruise_deadband_ms = 0.70;
    /// Setpoint change per button press [m/s] — one press of the stock stalk is 1 km/h.
    double cruise_tip_step_ms = 1.0 / 3.6;

    /** \brief Control-law gains, shared with the planner.
     *
     *  `AdasApp` fills both from one config section on purpose: a gain that can be tuned in one place
     *  and forgotten in the other is a gain that will differ between them. */
    lateral::AngleControl::Config ctl{};
    lateral::SlewGuard::Config slew{};  ///< Rate limit applied to the command before the PID sees it.
    double wheelbase_m = 2.636;         ///< Wheelbase [m], for the slip model.
    /** \brief Lever arm for converting curvature into a steering angle [m].
     *
     *  The solver calls this `mpc_Lf`; the wheelbase goes into the slip model instead. The two numbers
     *  differ, and substituting one for the other means driving at the wrong angle. */
    double lf_m = 2.67;
    /// Torque at full command [cNm]. Must not exceed what the panda's safety model allows, or the command
    /// is clipped somewhere that does not record having clipped it.
    double max_torque_cnm = 300.0;
    /// A plan older than this withdraws the command [s]. At 100 km/h 0.3 s is 8 m of road.
    double lane_max_age_s = 0.30;
    /// After the blinker goes off, wait this long before steering again [s], so the lane change finishes.
    double lka_blinker_resume_delay_s = 1.0;
    double assist_max_age_s =
        0.5;  ///< A panda report older than this counts as unknown [s]: silence is not permission.
  };

  explicit Control(Config config);

  void configure() override;
  /**
   * \brief Override the PID gains at runtime.
   *
   * \param[in] kp Proportional gain on the steering-angle error.
   * \param[in] ki Integral gain; a run with `ki = 0` is the only way to compare the instant response,
   * which is why the offline tools need this entry point.
   * \param[in] kf Feedforward gain, applied to `swa * (v^2 + v0^2)`.
   */
  void setPidGains(double kp, double ki, double kf) { ctl_.setPidGains(kp, ki, kf); }
  void reset() override {}
  std::string_view getName() const override { return "control"; }
  const Config& config() const { return config_; }

private:
  /** \brief One control step on a fixed 100 Hz tick: plan in curvature, gates, PID, command.
   *
   *  `controlsd` is built the same way around `Ratekeeper(100)`, where `LaC.update(latActive, CS, VM, lp)`
   *  yields the angle and the torque. Everything past that point is the bus, and the bus is not here. */
  void latTick();
  void registerParameters();

  /// The gates' verdict: whether to steer, and why not when it declines.
  struct LatGates {
    std::string status = "no_plan";
    bool assist_allowed = false;
    bool assist_known = false;
  };
  LatGates gates(const ChassisSample& ch, int64_t now_us);
  lateral::VehicleParams vehicleParams() const;
  void onHealth(const adas::proto::PandaHealth& h);
  /// Cruise-button intent: faster, slower, or nothing. Holding the button down is the platform's job.
  int cruiseIntent();

  Config config_;

  lateral::AngleControl ctl_;
  StaleGate stale_gate_;
  BlinkerGate blinker_gate_;
  AssistGate assist_gate_;

  /// The planner's last plan. The lateral loop's only input is curvature, never an angle.
  adas::proto::LatPlan plan_{};
  bool have_plan_ = false;
  /// Chassis state: decoded CAN in the car, and whatever the harness publishes in simulation.
  ChassisSample chassis_{};
  double frame_dt_s_ = 0.01;
  double road_roll_rad_ = 0.0;
  bool road_roll_valid_ = false;

  adas::proto::CarState car_state_{};

  bool have_long_plan_ = false;
  double long_v_target_ = 0.0;
  int64_t long_plan_ts_ms_ = 0;
  bool cruise_was_engaged_ = false;
  int64_t cruise_next_decision_ms_ = 0;
  double cruise_v_set_ = 0.0;
  double cruise_v_set_ceiling_ = 0.0;
};

}  // namespace services
}  // namespace adas
