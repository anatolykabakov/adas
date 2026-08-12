#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace adas {
class PidController {
public:
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

  void reset()
  {
    p_ = i_ = d_ = f_ = 0.0;
    control_ = 0.0;
  }

  void setGains(double k_p, double k_i, double k_f)
  {
    k_p_ = k_p;
    k_i_ = k_i;
    k_f_ = k_f;
  }

  void setRate(double rate_hz)
  {
    const double r = std::max(rate_hz, 1.0);
    i_rate_ = 1.0 / r;
    i_unwind_rate_ = 0.3 / r;
  }

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

  double p() const { return p_; }
  double i() const { return i_; }
  double f() const { return f_; }
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

class LatControlPid {
public:
  LatControlPid(double k_p = 0.6, double k_i = 0.2, double k_f = 0.00015, double rate_hz = 50.0,
                double v_ff_floor_mps = kFeedforwardFloorMps)
    : pid_(k_p, k_i, k_f, rate_hz), v_ff_floor_mps_(v_ff_floor_mps)
  {
  }

  void reset() { pid_.reset(); }

  void setGains(double k_p, double k_i, double k_f) { pid_.setGains(k_p, k_i, k_f); }

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
