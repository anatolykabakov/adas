#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace adas {
/// Plain PID with an anti-windup clamp and a feedforward term, at a fixed rate.
class PidController {
public:
  /**
   * \param[in] k_p Proportional gain.
   * \param[in] k_i Integral gain, applied per second — hence the rate below.
   * \param[in] k_f Feedforward gain.
   * \param[in] rate_hz Rate the integrator assumes [Hz]. A constant, not the measured interval: measuring
   * it made the command depend on the very first tick.
   * \param[in] pos_limit Output clamp, positive side.
   */
  PidController(double k_p = 0.6, double k_i = 0.2, double k_f = 0.00015, double rate_hz = 50.0, double pos_limit = 1.0,
                double neg_limit = -1.0)
    : k_p_(k_p)
    , k_i_(k_i)
    , k_f_(k_f)
    , pos_limit_(pos_limit)
    , neg_limit_(neg_limit)
    , i_rate_(1.0 / std::max(rate_hz, 1.0))
    , i_unwind_rate_(0.3 / std::max(rate_hz, 1.0))
  {
  }

  /// Zeroes the integrator and the last output. Call whenever the loop stops commanding.
  void reset()
  {
    p_ = i_ = d_ = f_ = 0.0;
    control_ = 0.0;
  }

  /// Replace the gains without disturbing the integrator.
  void setGains(double k_p, double k_i, double k_f)
  {
    k_p_ = k_p;
    k_i_ = k_i;
    k_f_ = k_f;
  }

  /// \param[in] rate_hz New integrator rate [Hz]; the accumulated integral is kept.
  void setRate(double rate_hz)
  {
    const double r = std::max(rate_hz, 1.0);
    i_rate_ = 1.0 / r;
    i_unwind_rate_ = 0.3 / r;
  }

  /**
   * \brief One PID step.
   *
   * \param[in] error Setpoint minus measurement, in the units the gains were tuned for.
   * \param[in] speed_mps Ego speed [m/s]; the feedforward scales with it.
   * \param[in] override Driver is holding the wheel: the integrator is frozen rather than wound up
   * against a hand.
   * \param[in] feedforward Feedforward term before the gain.
   * \return The clamped control output.
   */
  double update(double error, double speed_mps = 0.0, bool override = false, double feedforward = 0.0,
                bool freeze_integrator = false)
  {
    (void)speed_mps;
    p_ = error * k_p_;
    f_ = feedforward * k_f_;
    d_ = 0.0;

    if (override) {
      i_ -= i_unwind_rate_ * (i_ >= 0.0 ? 1.0 : -1.0);
    } else {
      const double i_new = i_ + error * k_i_ * i_rate_;
      const double control_try = p_ + i_new + d_ + f_;

      if (((error >= 0.0 && (control_try <= pos_limit_ || i_new < 0.0)) ||
           (error <= 0.0 && (control_try >= neg_limit_ || i_new > 0.0))) &&
          !freeze_integrator) {
        i_ = i_new;
      }
    }

    control_ = std::clamp(p_ + i_ + d_ + f_, neg_limit_, pos_limit_);
    return control_;
  }

  /// The proportional, integral and feedforward parts of the last output, for the record.
  double p() const { return p_; }
  double i() const { return i_; }
  double f() const { return f_; }
  /// The last output, after clamping.
  double control() const { return control_; }

private:
  double k_p_ = 0.6;
  double k_i_ = 0.2;
  double k_f_ = 0.00015;
  double pos_limit_ = 1.0;
  double neg_limit_ = -1.0;
  double i_rate_ = 0.02;
  double i_unwind_rate_ = 0.006;
  double p_ = 0, i_ = 0, d_ = 0, f_ = 0;
  double control_ = 0;
};

inline constexpr double kFeedforwardFloorMps = 9.8;

/**
 * \brief The steering-angle PID: desired angle in, normalised torque out.
 *
 * \details Closes the loop on the measured steering-wheel angle, not on a model of it, and hands control
 * back to the driver when the wheel is touched. The feedforward term carries most of the command on a
 * steady curve; the integrator only cleans up what the feedforward misses.
 */
class LatControlPid {
public:
  /// Gains as in \ref PidController, plus the speed floor used by the feedforward.
  LatControlPid(double k_p = 0.6, double k_i = 0.2, double k_f = 0.00015, double rate_hz = 50.0,
                double v_ff_floor_mps = kFeedforwardFloorMps)
    : pid_(k_p, k_i, k_f, rate_hz), v_ff_floor_mps_(v_ff_floor_mps)
  {
  }

  void reset() { pid_.reset(); }

  void setGains(double k_p, double k_i, double k_f) { pid_.setGains(k_p, k_i, k_f); }

  /**
   * \param[in] v_mps Speed floor for the feedforward [m/s].
   *
   * The term is `swa * (v^2 + v0^2)`, so the floor keeps it from vanishing at low speed, where the wheel is
   * heaviest and the loop would otherwise build the whole command out of the integrator.
   */
  void setFeedforwardFloor(double v_mps) { v_ff_floor_mps_ = std::max(0.0, v_mps); }

  void setRate(double rate_hz) { pid_.setRate(rate_hz); }

  struct Result {
    double steer_norm = 0.0;
    double angle_des_deg = 0.0;
    double angle_act_deg = 0.0;
    double angle_error_deg = 0.0;
    double p = 0, i = 0, f = 0;
    bool active = false;
  };

  /**
   * \brief One step of the steering-angle loop.
   *
   * \param[in] active False releases the loop: output zero and the integrator reset.
   * \param[in] desired_swa_deg Requested steering-wheel angle [deg], including the learned zero.
   * \param[in] actual_swa_deg Measured steering-wheel angle [deg], from CAN.
   * \param[in] v_ego_mps Ego speed [m/s].
   * \param[in] steering_pressed Driver on the wheel; freezes the integrator.
   * \param[in] ff_swa_deg Angle the feedforward uses, without the learned zero. NaN reuses the request.
   * \return Normalised command and every intermediate value behind it.
   */
  Result update(bool active, double desired_swa_deg, double actual_swa_deg, double v_ego_mps, bool steering_pressed,
                double ff_swa_deg = std::numeric_limits<double>::quiet_NaN())
  {
    Result r;
    r.angle_des_deg = desired_swa_deg;
    r.angle_act_deg = actual_swa_deg;
    r.angle_error_deg = desired_swa_deg - actual_swa_deg;
    if (!active) {
      pid_.reset();
      return r;
    }

    const double ff_angle = std::isfinite(ff_swa_deg) ? ff_swa_deg : desired_swa_deg;
    const double ff = ff_angle * (v_ego_mps * v_ego_mps + v_ff_floor_mps_ * v_ff_floor_mps_);
    r.steer_norm = pid_.update(r.angle_error_deg, v_ego_mps, steering_pressed, ff);
    r.p = pid_.p();
    r.i = pid_.i();
    r.f = pid_.f();
    r.active = true;
    return r;
  }

private:
  PidController pid_;
  double v_ff_floor_mps_ = kFeedforwardFloorMps;
};

}  // namespace adas
