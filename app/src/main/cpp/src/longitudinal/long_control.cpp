#include "adas/longitudinal/long_control.h"

#include <algorithm>
#include <cmath>

namespace adas {
namespace longitudinal {
LongControl::LongControl(LongControlConfig cfg) : cfg_(cfg) {}

void LongControl::reset(double v_pid)
{
  v_pid_ = v_pid;
  integral_ = 0.0;
}

LongCtrlState LongControl::transition(const LongControlInput& in, double v_target, double v_target_1s) const
{
  const bool accelerating = v_target_1s > v_target;
  const bool planned_stop = v_target < cfg_.v_ego_stopping && v_target_1s < cfg_.v_ego_stopping && !accelerating;
  const bool stay_stopped = in.v_ego < cfg_.v_ego_stopping && (in.brake_pressed || in.standstill);
  const bool stopping_condition = planned_stop || stay_stopped;
  const bool starting_condition = v_target_1s > cfg_.v_ego_starting && accelerating && !in.brake_pressed;
  const bool started_condition = in.v_ego > cfg_.v_ego_starting;

  if (!in.active)
    return LongCtrlState::Off;
  switch (state_) {
    case LongCtrlState::Off:
      return stopping_condition ? LongCtrlState::Stopping : LongCtrlState::Pid;
    case LongCtrlState::Pid:
      return stopping_condition ? LongCtrlState::Stopping : LongCtrlState::Pid;
    case LongCtrlState::Stopping:
      if (starting_condition)
        return cfg_.starting_state ? LongCtrlState::Starting : LongCtrlState::Pid;
      return LongCtrlState::Stopping;
    case LongCtrlState::Starting:
      if (stopping_condition)
        return LongCtrlState::Stopping;
      if (started_condition)
        return LongCtrlState::Pid;
      return LongCtrlState::Starting;
  }
  return LongCtrlState::Off;
}

LongControlOutput LongControl::update(const LongControlInput& in)
{
  LongControlOutput out;
  const auto& T = modelTimes();
  const double t = std::max(0.0, in.t_since_plan_s);
  const double delay = cfg_.actuator_delay_s;

  // Read the plan at the actuator delay, and turn the speed slope over that delay into the
  // acceleration to request — the plan's own acceleration alone would lag by exactly that delay.
  const double v_now = interp(t, T.data(), in.speeds.data(), kControlN);
  const double a_now = interp(t, T.data(), in.accels.data(), kControlN);
  const double v_target = interp(delay + t, T.data(), in.speeds.data(), kControlN);
  const double a_target = delay > 1e-6 ? 2.0 * (v_target - v_now) / delay - a_now : a_now;
  const double v_target_1s = interp(delay + t + 1.0, T.data(), in.speeds.data(), kControlN);

  state_ = transition(in, v_target, v_target_1s);
  out.state = state_;
  out.v_target = v_target;
  out.v_target_1s = v_target_1s;
  out.a_target = a_target;

  double output_accel = last_output_accel_;
  switch (state_) {
    case LongCtrlState::Off:
      reset(in.v_ego);
      output_accel = 0.0;
      break;
    case LongCtrlState::Stopping:
      if (output_accel > cfg_.stop_accel) {
        output_accel = std::min(output_accel, 0.0);
        output_accel -= cfg_.stopping_decel_rate * cfg_.dt_s;
      }
      reset(in.v_ego);
      break;
    case LongCtrlState::Starting:
      output_accel = cfg_.start_accel;
      reset(in.v_ego);
      break;
    case LongCtrlState::Pid: {
      v_pid_ = v_target;
      const bool prevent_overshoot = !cfg_.stopping_control && in.v_ego < 1.5 && v_target_1s < 0.7 &&
                                     v_target_1s < v_pid_;
      const double error = v_pid_ - in.v_ego;
      out.p = cfg_.kp * error;
      out.f = cfg_.kf * a_target;
      const double control_unclamped = out.p + integral_ + out.f;
      const double i_new = integral_ + error * cfg_.ki * cfg_.dt_s;
      const bool room = (error >= 0.0 && (control_unclamped <= cfg_.accel_max || i_new < 0.0)) ||
                        (error <= 0.0 && (control_unclamped >= cfg_.accel_min || i_new > 0.0));
      if (room && !prevent_overshoot)
        integral_ = i_new;
      out.i = integral_;
      output_accel = out.p + out.i + out.f;
      break;
    }
  }
  output_accel = std::clamp(output_accel, cfg_.accel_min, cfg_.accel_max);
  last_output_accel_ = output_accel;
  out.accel = output_accel;
  return out;
}

}  // namespace longitudinal
}  // namespace adas
