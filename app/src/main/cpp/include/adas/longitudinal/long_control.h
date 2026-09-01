#pragma once

#include <array>

#include "adas/longitudinal/long_mpc.h"

namespace adas {
namespace longitudinal {
/** upstream's `LongControl` tuning for this car (VW MQB): a proportional law on speed with the plan's
 *  acceleration as feedforward, and the four-state machine around a stop. */
struct LongControlConfig {
  double kp = 0.1;    ///< Acceleration per m/s of speed error.
  double ki = 0.0;    ///< Integral gain; VW ships zero.
  double kf = 1.0;    ///< Feedforward gain on the plan's acceleration.
  double v_ego_stopping = 1.0;       ///< Below this a planned stop enters the stopping state [m/s].
  double v_ego_starting = 1.0;       ///< Above this the car has started [m/s].
  double start_accel = 1.0;          ///< Acceleration requested while starting [m/s²].
  double stop_accel = -2.0;          ///< Acceleration held once stopped [m/s²].
  double stopping_decel_rate = 0.8;  ///< How fast the request ramps down while stopping [m/s³].
  bool stopping_control = true;
  bool starting_state = true;
  double actuator_delay_s = 0.15;    ///< The plan is read this far ahead to cover the actuator's delay.
  double accel_min = -3.5;           ///< The platform's envelope [m/s²].
  double accel_max = 2.0;
  double dt_s = 0.01;                ///< Control tick [s].
};

enum class LongCtrlState : int { Off = 0, Pid = 1, Stopping = 2, Starting = 3 };

struct LongControlInput {
  bool active = false;          ///< The longitudinal loop is allowed to act.
  double v_ego = 0.0;
  bool brake_pressed = false;
  bool standstill = false;
  double t_since_plan_s = 0.0;  ///< Age of the plan being tracked [s].
  std::array<double, kControlN> speeds{};
  std::array<double, kControlN> accels{};
};

struct LongControlOutput {
  LongCtrlState state = LongCtrlState::Off;
  double accel = 0.0;          ///< The command [m/s²], inside the envelope.
  double v_target = 0.0;       ///< Plan speed at the actuator delay.
  double v_target_1s = 0.0;    ///< Plan speed one second later — decides stopping/starting.
  double a_target = 0.0;       ///< Feedforward acceleration.
  double p = 0.0;
  double i = 0.0;
  double f = 0.0;
};

class LongControl {
public:
  explicit LongControl(LongControlConfig cfg = {});
  LongControlOutput update(const LongControlInput& in);
  void reset(double v_pid);
  LongCtrlState state() const { return state_; }
  const LongControlConfig& config() const { return cfg_; }

private:
  LongCtrlState transition(const LongControlInput& in, double v_target, double v_target_1s) const;

  LongControlConfig cfg_;
  LongCtrlState state_ = LongCtrlState::Off;
  double v_pid_ = 0.0;
  double integral_ = 0.0;
  double last_output_accel_ = 0.0;
};

}  // namespace longitudinal
}  // namespace adas
