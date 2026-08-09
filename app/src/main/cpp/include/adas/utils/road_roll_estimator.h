#pragma once

#include <algorithm>
#include <cmath>

namespace adas {

/** Road bank (roll) from the lateral accelerometer, the yaw rate and the speed.
 *
 *  Why this exists: `paramsd` cannot learn tyre stiffness honestly without it. A banked road and an
 *  understeering car produce the same lateral-acceleration signature, so a parameter estimator with no
 *  roll input silently folds the road into the vehicle. Upstream's `paramsd` takes roll from its localizer
 *  and falls back to "zero with a 10° standard deviation" when it cannot — that fallback is what we have
 *  today, and this class is the alternative.
 *
 *  ## The measurement
 *
 *  In the vehicle frame (x forward, y right, z down), with the road banked by φ, gravity has a lateral
 *  component, so the accelerometer reads
 *
 *      f_y = a_y − g·sin(φ),      a_y = −v · yaw_rate_can
 *
 *  hence `sin(φ) = (a_y − f_y) / g`. The minus in `a_y` is not optional: `chassis.yaw_rate` is decoded the
 *  way flowpilot decodes `ESP_02`, i.e. in openpilot's ISO convention with z up and positive for a **left**
 *  turn, while this frame has z down. Getting it backwards produces an apparent body-roll gradient of
 *  116 °/g — seventeen times what a car can do — which is the only reason the sign error was caught.
 *
 *  ## The body-roll confound, and why it is subtracted rather than ignored
 *
 *  The phone is bolted to the body, and the body rolls on its suspension away from the turn in proportion
 *  to lateral acceleration. To first order that tilt is indistinguishable from road bank, so the estimate
 *  reads `φ_road + K·a_y`. Measured on two runs (`bag_road_roll.py`), restricted to real cornering where
 *  the regression has a lever arm at all (|a_y| between 1 and 2 m/s²): **+3.2 and +2.8 °/g**, which is the
 *  low end of the physical range for a hatchback. On near-straight samples the same regression returns
 *  −7 °/g with a correlation of −0.07 — fitting noise, and a good reminder to gate a slope estimate on
 *  having some signal to slope against.
 *
 *  ## What accuracy to expect
 *
 *  Per-sample scatter is 2.1–2.3° (the phone is on a windscreen mount at 15 Hz; road vibration dominates).
 *  Averaging brings that to 0.9° over one second and **0.65° over ten**, and then it stops falling — the
 *  floor is the road's own camber changing along the route, which is the thing we are trying to see. So
 *  the filter is deliberately slow, and it reports its own uncertainty so a consumer can gate on it the
 *  way `paramsd` does.
 */
class RoadRollEstimator {
public:
  struct Config {
    /** Suspension roll gradient, degrees per g, subtracted from the raw estimate. Measured 2.8–3.2 on
     *  this car; 0 disables the correction and leaves the confound in, which is the honest thing to do
     *  on a vehicle where it has not been measured. */
    double body_roll_deg_per_g = 3.0;

    /** Time constant of the output filter, seconds. Ten seconds is where the per-sample 2.1° scatter
     *  stops improving; shorter buys noise, longer starts smearing one corner into the next. */
    double tau_s = 10.0;

    /** Below this speed the relation is dominated by its own noise: `a_y = −v·ψ̇` vanishes and the
     *  accelerometer sees mostly the phone shaking. */
    double min_speed_ms = 8.0;

    /** Reject transients. The relation above is steady-state; during a steering input the suspension is
     *  still moving and the body-roll correction does not hold. Units: m/s³. */
    double max_lateral_jerk = 0.8;

    /** A sample further than this from the current estimate is a bump or a clipped accelerometer, not a
     *  road. Degrees. */
    double max_innovation_deg = 8.0;

    /** Standard deviation reported before enough samples have arrived, degrees. Deliberately the same
     *  10° that `paramsd` uses for "no usable roll", so a consumer gating on it behaves identically
     *  whether we publish nothing or publish an unconverged estimate. */
    double unconverged_std_deg = 10.0;

    /** Per-sample scatter, degrees — the measured 2.1–2.3. Used to shrink the reported uncertainty as
     *  samples accumulate. */
    double sample_std_deg = 2.2;
  };

  RoadRollEstimator() = default;
  explicit RoadRollEstimator(Config cfg) : cfg_(cfg) {}

  void setConfig(const Config& cfg) { cfg_ = cfg; }
  const Config& config() const { return cfg_; }

  void reset()
  {
    roll_deg_ = 0.0;
    n_ = 0;
    have_prev_ay_ = false;
    prev_ay_ = 0.0;
  }

  /** One sample. `yaw_rate_can` is `chassis.yaw_rate` as decoded — ISO, left-positive. `f_y` is the
   *  lateral specific force in the vehicle frame (x forward, y right, z down), i.e. the calibrated
   *  accelerometer's y component. Returns true if the sample was used. */
  bool update(double speed_ms, double yaw_rate_can, double f_y, double dt_s)
  {
    if (!(dt_s > 0.0) || dt_s > 1.0)
      return finish(false);
    if (!std::isfinite(speed_ms) || !std::isfinite(yaw_rate_can) || !std::isfinite(f_y))
      return finish(false);
    if (speed_ms < cfg_.min_speed_ms)
      return finish(false);

    const double a_y = -speed_ms * yaw_rate_can;

    // Steady state only. Without this the suspension's own transient is read as road.
    if (have_prev_ay_) {
      const double jerk = std::abs(a_y - prev_ay_) / dt_s;
      if (jerk > cfg_.max_lateral_jerk) {
        prev_ay_ = a_y;
        return finish(false);
      }
    }
    prev_ay_ = a_y;
    have_prev_ay_ = true;

    constexpr double kG = 9.81;
    const double sin_phi = std::clamp((a_y - f_y) / kG, -1.0, 1.0);
    double sample_deg = std::asin(sin_phi) * 180.0 / M_PI;
    // Take out the part that scales with lateral acceleration: that is the body on its springs.
    sample_deg -= cfg_.body_roll_deg_per_g * (a_y / kG);

    if (n_ > 0 && std::abs(sample_deg - roll_deg_) > cfg_.max_innovation_deg)
      return finish(false);

    const double alpha = n_ == 0 ? 1.0 : std::clamp(dt_s / std::max(cfg_.tau_s, 1e-3), 0.0, 1.0);
    roll_deg_ = roll_deg_ + alpha * (sample_deg - roll_deg_);
    if (n_ < 1'000'000)
      ++n_;
    return finish(true);
  }

  double rollDeg() const { return roll_deg_; }
  int sampleCount() const { return n_; }

  /** Reported uncertainty, degrees. Shrinks as `sample_std / sqrt(n)` but never below the 0.65° floor the
   *  data shows, because past ten seconds the residual is the road changing rather than noise — claiming
   *  better than that would invite a consumer to trust a number the sensor cannot support. */
  double rollStdDeg() const
  {
    if (n_ < 30)
      return cfg_.unconverged_std_deg;
    const double shrunk = cfg_.sample_std_deg / std::sqrt(static_cast<double>(n_));
    return std::max(0.65, shrunk);
  }

  bool valid() const { return n_ >= 30; }

private:
  bool finish(bool used) { return used; }

  Config cfg_{};
  double roll_deg_ = 0.0;
  int n_ = 0;
  bool have_prev_ay_ = false;
  double prev_ay_ = 0.0;
};

}  // namespace adas
