#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/lateral/angle_control.h"
#include "adas/lateral/limits.h"
#include "adas/longitudinal/long_control.h"
#include "adas/utils/lane_keep_gates.h"

namespace adas {
namespace services {
/** The control laws and nothing else: the lateral one (curvature → torque) and the longitudinal one
 *  (speed plan → acceleration), on one 100 Hz tick, into one command message. */
class Control : public adas::middleware::Service {
public:
  struct Config {
    /** \brief Control-law gains, shared with the planner.
     *
     *  `AdasApp` fills both from one config section on purpose: a gain that can be tuned in one place
     *  and forgotten in the other is a gain that will differ between them. */
    lateral::AngleControl::Config ctl{};
    lateral::SlewGuard::Config slew{};  ///< Rate limit applied to the command before the PID sees it.
    double wheelbase_m = 2.636;         ///< Wheelbase [m], for the slip model.
    bool lat_use_vehicle_model = true;  ///< False drops the slip term, leaving the kinematic model.
    /** \brief Lever arm for converting curvature into a steering angle [m].
     *
     *  The solver calls this `mpc_Lf`; the wheelbase goes into the slip model instead. The two numbers
     *  differ, and substituting one for the other means driving at the wrong angle. */
    double lf_m = 2.67;  ///< Lever arm for curvature-to-angle [m]; mpc_Lf, not the wheelbase.
    /// Torque at full command [cNm]. Must not exceed what the panda's safety model allows, or the command
    /// is clipped somewhere that does not record having clipped it.
    double max_torque_cnm = 300.0;
    /// A plan older than this withdraws the command [s]. At 100 km/h 0.3 s is 8 m of road.
    double lane_max_age_s = 0.30;
    /// After the blinker goes off, wait this long before steering again [s], so the lane change finishes.
    double lka_blinker_resume_delay_s = 1.0;
    double assist_max_age_s =
        0.5;  ///< A panda report older than this counts as unknown [s]: silence is not permission.

    /// Run the longitudinal law at all. Off on a car whose platform does not take the axis.
    bool long_control_enabled = false;
    /// A longitudinal plan older than this withdraws the acceleration [s]; the plan comes at 20 Hz.
    double long_plan_max_age_s = 0.5;
    double long_actuator_delay_s = 0.15;  ///< See longitudinal::LongControlConfig.
    longitudinal::LongControlConfig long_ctl{};
  };

  /// \param[in] config Control law settings; see Config fields.
  explicit Control(Config config);

  void configure() override;
  void reset() override {}
  std::string_view getName() const override { return "control"; }
  /// \return The config in force.
  const Config& config() const { return config_; }

private:
  /** \brief One control step on a fixed 100 Hz tick: plan in curvature, gates, PID, command.
   *
   *  `controlsd` is built the same way around `Ratekeeper(100)`, where `LaC.update(latActive, CS, VM, lp)`
   *  yields the angle and the torque, and `LoC.update(...)` the acceleration. Everything past that point
   *  is the bus, and the bus is not here. */
  void latTick();
  void registerParameters();

  /// The gates' verdict: whether to steer, and why not when it declines.
  struct LatGates {
    std::string status = "no_plan";
    bool assist_allowed = false;
    bool assist_known = false;
  };
  LatGates gates(const ChassisSample& ch, int64_t now_us);
  /// The longitudinal gates: a fresh plan with a set speed, the panda's controls_allowed, no pedal.
  std::string longGate(int64_t now_us);
  lateral::VehicleParams vehicleParams() const;
  void onHealth(const adas::proto::PandaHealth& h);

  Config config_;

  lateral::AngleControl ctl_;
  StaleGate stale_gate_;
  BlinkerGate blinker_gate_;
  AssistGate assist_gate_;
  AssistGate controls_gate_;  ///< The panda's controls_allowed, aged the same way as the lateral permission.

  /// The planner's last plan. The lateral loop's only input is curvature, never an angle.
  adas::proto::LatPlan plan_{};
  bool have_plan_ = false;
  /// Chassis state: decoded CAN in the car, and whatever the harness publishes in simulation.
  ChassisSample chassis_{};
  double frame_dt_s_ = 0.01;
  double road_roll_rad_ = 0.0;
  bool road_roll_valid_ = false;

  adas::proto::CarState car_state_{};

  longitudinal::LongControl long_ctl_;
  adas::proto::LongPlanState long_plan_{};
  bool have_long_plan_ = false;
  int64_t long_plan_rx_us_ = 0;
};

}  // namespace services
}  // namespace adas
