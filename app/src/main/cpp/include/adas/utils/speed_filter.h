#pragma once

#include <cmath>

namespace adas {
/** Two-state (speed, acceleration) Kalman filter over the CAN wheel speed.
 *
 *  Why this exists. The decoder used to publish `v_ego` as the raw four-wheel average and `a_ego` as a
 *  finite difference over the message interval. Measured on run 2026_08_06_00_36_42 above 5 m/s, that
 *  `a_ego` came out with p5/p95 of ±3.8 m/s², extremes of −61.5 and +74.5, and an RMS step of
 *  4.16 m/s² between consecutive samples — a car's real acceleration lives inside ±3 m/s² and cannot
 *  change by 4 m/s² in 10 ms. The field was quantisation noise wearing an acceleration label. Nothing
 *  read it, which is the only reason it did no harm; anything longitudinal that started using it would
 *  have been poisoned.
 *
 *  Upstream solves this the same way — `CarInterfaceBase.update_speed_kf` in
 *  `flowpilot/selfdrive/car/interfaces.py` runs a fixed-gain `KF1D` with `A = [[1, dt], [0, 1]]`,
 *  `C = [1, 0]` and `K = [0.174, 1.659]` at 100 Hz, and takes both `vEgo` and `aEgo` from it. Their
 *  gains are baked for one rate; ours are computed from the actual `dt` between frames, because
 *  `ESP_02` does not arrive on a metronome and a gain tuned for 10 ms is wrong at 25.
 *
 *  Also here: `wheel_speed_factor`. Wheel speed is a wheel circumference times a rate, so a worn or
 *  under-inflated tyre makes the whole signal read high by a constant. Measured against GNSS Doppler
 *  on two runs (`bag_speed_sources.py`): **+1.17 % and +1.20 %**, flat across speed bands (1.005 to
 *  1.022, no trend) — the signature of a radius constant rather than slip. It matters because
 *  everything lateral is scaled by speed: the understeer term `κ = δ / (L·(1 + K·v²))` squares it.
 */
class SpeedFilter {
public:
  struct Config {
    double wheel_speed_factor = 1.0;

    double accel_process_noise = 1.5;

    double speed_measurement_noise = 0.05;

    double reseed_jump_ms = 2.0;
  };

  SpeedFilter() = default;
  explicit SpeedFilter(Config cfg) : cfg_(cfg) {}

  void setConfig(const Config& cfg) { cfg_ = cfg; }
  const Config& config() const { return cfg_; }

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

  bool ready() const { return inited_; }
  double speed() const { return v_; }
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
