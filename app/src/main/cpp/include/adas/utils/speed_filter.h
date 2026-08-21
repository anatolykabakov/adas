#pragma once

#include <cmath>

namespace adas {
/** Two-state (speed, acceleration) Kalman filter over the CAN wheel speed. */
class SpeedFilter {
public:
  struct Config {
    double wheel_speed_factor = 1.0;  ///< Correction on wheel speed; the measured 1.2 % scale error goes here.

    double accel_process_noise = 1.5;  ///< Process noise on acceleration [m/s^3].

    double speed_measurement_noise = 0.05;  ///< Assumed wheel-speed noise [m/s].

    double reseed_jump_ms = 2.0;  ///< A jump larger than this reseeds the filter instead of being smoothed [m/s].
  };

  SpeedFilter() = default;
  /// \param[in] cfg Smoothing constants.
  explicit SpeedFilter(Config cfg) : cfg_(cfg) {}

  /// Replace the config.
  void setConfig(const Config& cfg) { cfg_ = cfg; }
  /// \return The config in force.
  const Config& config() const { return cfg_; }

  /// Forget the state; the next sample re-primes it.
  void reset()
  {
    inited_ = false;
    v_ = 0.0;
    a_ = 0.0;
    p_vv_ = 1.0;
    p_va_ = 0.0;
    p_aa_ = 1.0;
  }

  /** Feed one raw wheel-speed average with the interval since the previous one. */
  void update(double v_raw_ms, double dt_s)
  {
    const double z = v_raw_ms * cfg_.wheel_speed_factor;
    if (!std::isfinite(z)) {
      return;
    }
    if (!inited_ || !(dt_s > 0.0) || dt_s > 0.5 || std::abs(z - v_) > cfg_.reseed_jump_ms) {
      v_ = z;
      a_ = 0.0;
      p_vv_ = cfg_.speed_measurement_noise * cfg_.speed_measurement_noise;
      p_va_ = 0.0;
      p_aa_ = 1.0;
      inited_ = true;
      return;
    }

    v_ += a_ * dt_s;
    const double q = cfg_.accel_process_noise * cfg_.accel_process_noise * dt_s;
    const double p_vv = p_vv_ + 2.0 * dt_s * p_va_ + dt_s * dt_s * p_aa_ + q * dt_s * dt_s / 3.0;
    const double p_va = p_va_ + dt_s * p_aa_ + q * dt_s / 2.0;
    const double p_aa = p_aa_ + q;
    p_vv_ = p_vv;
    p_va_ = p_va;
    p_aa_ = p_aa;

    const double r = cfg_.speed_measurement_noise * cfg_.speed_measurement_noise;
    const double s = p_vv_ + r;
    if (!(s > 0.0))
      return;
    const double k_v = p_vv_ / s;
    const double k_a = p_va_ / s;
    const double innov = z - v_;
    v_ += k_v * innov;
    a_ += k_a * innov;

    const double new_p_vv = (1.0 - k_v) * p_vv_;
    const double new_p_va = (1.0 - k_v) * p_va_;
    const double new_p_aa = p_aa_ - k_a * p_va_;
    p_vv_ = new_p_vv;
    p_va_ = new_p_va;
    p_aa_ = new_p_aa;
  }

  /// \return True after the first sample.
  bool ready() const { return inited_; }
  /// \return Filtered speed [m/s].
  double speed() const { return v_; }
  /// \return Filtered acceleration [m/s^2].
  double accel() const { return a_; }

private:
  Config cfg_{};
  bool inited_ = false;
  double v_ = 0.0;
  double a_ = 0.0;
  double p_vv_ = 1.0;
  double p_va_ = 0.0;
  double p_aa_ = 1.0;
};

}  // namespace adas
