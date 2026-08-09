#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "utils/vehicle_model.h"

namespace adas {

/** Learns the vehicle parameters the lateral controller uses, instead of having them hand-tuned.
 *
 *  This is the portable part of upstream's `paramsd` (`docs/PARAMSD.md`): the two numbers that can be
 *  learned from steering angle, speed and yaw rate alone — tyre stiffness and steer ratio — plus the
 *  steering bias, for which we had no mechanism at all. Their nine-state filter also carries internal
 *  velocity and yaw-rate states and a road-roll state; we do not need those, because the roll comes from
 *  `RoadRollEstimator` as an input and the rest are measured directly on CAN.
 *
 *  ## Why it exists
 *
 *  `tire_stiffness_factor` is one constant, and the thing it represents is not constant. Measured on this
 *  car, `κ_fact/κ_kin` is 0.97 at 6–9 m/s, 0.80 at 12–15 and 0.54 at 21–26. A value chosen for the city
 *  under-commands the highway and vice versa: 0.50 measured right on urban arcs made a *replayed* urban arc
 *  worse. And two independent estimates disagreed about the direction — our 0.54 against comma's learned
 *  1.319 on the same car — which took a third sensor to resolve.
 *
 *  ## The measurement
 *
 *  Deliberately the controller's own function, `curvatureFromSteer`, so a learned `tire_stiffness_factor`
 *  means exactly what the consumer of it means. A parameter learned against a slightly different model than
 *  the one that uses it is worse than a hand-tuned constant, because it looks principled.
 *
 *      delta   = (SWA - offset) / steer_ratio
 *      kappa   = curvatureFromSteer(delta, v, L, slipFactor(tsf))
 *      yaw_pred(z down) = v * kappa + g * sin(roll) / v
 *
 *  The roll term is the reason this could not be built before. A banked road supplies part of the lateral
 *  acceleration for free, so at a given steering angle the car turns more than the flat-road model says. At
 *  20 m/s one degree of bank is 0.0086 rad/s of yaw rate — a few percent of a normal cornering rate, and a
 *  systematic error that lands entirely in the learned stiffness if it is not accounted for. There is a test
 *  for exactly that.
 *
 *  ## Signs
 *
 *  Everything inside is the z-down frame: positive yaw rate is a right turn, positive curvature is a right
 *  turn, positive roll tips the right side down. `chassis.yaw_rate` is ISO — z up, positive for a **left**
 *  turn — so it is negated on the way in. This project has already paid for that convention once, in the
 *  road-roll estimator, where a flipped sign reported a body-roll gradient of 116 deg/g instead of an error.
 *
 *  ## Jacobians
 *
 *  Numeric, by central difference. Three states at CAN rate cost nothing, and the analytic derivative
 *  through `slipFactor` — which itself divides by products of the stiffnesses — is exactly the kind of
 *  expression that is wrong for a month before anyone notices. The measurement function is the single source
 *  of truth; differentiating it numerically cannot disagree with it.
 */
class ParamsLearner {
public:
  enum State { kStiffness = 0, kSteerRatio = 1, kAngleOffsetDeg = 2, kNumStates = 3 };

  struct Config {
    VehicleModelParams vehicle{};

    /** Starting point and how far the estimate may wander. Bounds are upstream's own sanity range for
     *  stiffness; the steer-ratio range brackets the port value (15.7) and comma's learned 16.27. */
    double stiffness_init = 0.64;
    double stiffness_min = 0.2;
    double stiffness_max = 5.0;
    double steer_ratio_init = 15.7;

    /** Sign relating the steering-wheel angle on CAN to the road-wheel angle in this frame, i.e. the
     *  `vehicle.steer_sign` the controller already applies. Not cosmetic, and not something a filter can
     *  absorb: with the wrong sign the prediction opposes the measurement, the innovation is roughly twice
     *  the yaw rate on every corner, and the filter runs to whichever bounds reduce the predicted magnitude
     *  and sits there. Measured on run 2026_08_06_00_36_42 with the sign missing: stiffness pinned at its
     *  0.200 floor and steer ratio at its 20.0 ceiling, identical in all four quarters of the drive, while
     *  `valid()` cheerfully returned true. On this car the CAN angle is positive for a **left** turn — the
     *  regression of the measured ISO yaw rate against the kinematic prediction gives +0.824 at a
     *  correlation of 0.987 — and this frame is positive for a right turn, so the sign is -1. */
    double steer_sign = 1.0;
    double steer_ratio_min = 12.0;
    double steer_ratio_max = 20.0;
    double angle_offset_max_deg = 10.0;

    /** Initial uncertainty. Generous on stiffness because that is the number in dispute; *tight* on steer
     *  ratio, and the tightness is the whole design.
     *
     *  Stiffness and steer ratio are close to degenerate against a yaw-rate measurement. Both scale the
     *  predicted curvature, and while their speed signatures differ — the ratio linearly, the stiffness
     *  through 1/(1 - slip*v^2) — the difference over a normal drive's speed range is small enough that the
     *  filter slides along the ridge instead of picking a point on it. Measured by replaying run
     *  2026_08_06_00_36_42 with a 0.5 prior on the ratio: it wandered from 15.7 to 14.06 and dragged
     *  stiffness to 0.374, and the pair predicted the yaw rate 3 % *worse* than the shipped constants while
     *  reporting itself converged. With this prior instead the ratio stays near its mechanical value, the
     *  stiffness lands at 0.603 +- 0.030 against a configured 0.64, and the residual improves by 1 %.
     *
     *  The lesson generalises past this filter: a state that the data cannot separate from another state is
     *  not made observable by declaring it a state. Either constrain it by what is known independently — a
     *  steering rack's ratio is a mechanical fact, known to a few percent — or do not estimate it. */
    double stiffness_std_init = 0.5;
    double steer_ratio_std_init = 0.1;
    double angle_offset_std_init = 1.0;

    /** Random walk, per second. Slow: these are properties of a car, not of a corner. */
    double stiffness_process_std = 0.005;
    /** Two orders below the stiffness: a steering rack's ratio does not drift over a drive, and letting it
     *  random-walk is how it slides down the degeneracy described under `steer_ratio_std_init`. */
    double steer_ratio_process_std = 0.00005;
    double angle_offset_process_std = 0.02;

    /** Yaw-rate measurement noise, rad/s. The ESP sensor was validated against the phone gyro at a ratio of
     *  1.017 with no speed dependence, so it is trusted — this covers the model, not the sensor. */
    double yaw_rate_std = 0.02;

    /** Ceiling on each slow parameter's uncertainty — `paramsd`'s anti-growth device, ported as what it
     *  actually is rather than as what it looks like.
     *
     *  Upstream writes it as "observe the state with its own current value at high noise". That form is
     *  tempting to copy literally, and copying it literally is a bug: the innovation `z − x` is identically
     *  zero, so the update cannot move the estimate and only shrinks the covariance. Applied once per
     *  localizer message that is a harmless bound; applied on every CAN sample at 100 Hz it shrinks the
     *  covariance a hundred times a second without a single bit of information arriving, the yaw-rate
     *  measurement loses all gain, and the filter freezes near wherever it started. Measured on the
     *  synthetic recovery test: a true stiffness of 0.45 came out as 0.515, pinned by the initial 0.64. It
     *  also broke `valid()`, which read the shrunk sigma as convergence after ten minutes of straight road.
     *
     *  So it is a cap: sigma may never exceed this, and only real measurements may reduce it. */
    double stiffness_std_max = 0.5;
    double steer_ratio_std_max = 0.3;

    /** Gates, all from `paramsd` unless noted. */
    double min_speed_ms = 5.0;      //!< upstream uses 1; below 5 the yaw signal is mostly noise here
    double max_steer_deg = 45.0;    //!< linear region of the tyre model
    double max_yaw_rate = 1.0;      //!< rad/s
    double max_lateral_jerk = 1.0;  //!< m/s^3; the model is steady-state
    double max_roll_std_deg = 3.0;  //!< reject the roll input when it has not converged

    /** Use the road-bank input at all — and on this hardware the answer measured out as *no*.
     *
     *  The term is `g·sin(roll)/v`, and at 15 m/s one degree of bank is 0.0114 rad/s. The whole yaw-rate
     *  residual of the flat model on run 2026_08_06_00_36_42 is 0.0065 rad/s. So a one-degree error in the
     *  bank injects nearly twice the error the model already has, and the bank estimate from a windscreen-
     *  mounted phone is good to 0.65° at best — that is its floor after ten seconds of averaging, where it
     *  stops improving because the residual is the road's camber genuinely changing. Feeding it in raised the
     *  residual from 0.0065 to 0.0104 rad/s, a 60 % degradation.
     *
     *  This is a negative result about the estimator built immediately before this one, and it is the honest
     *  reading: `RoadRollEstimator` measures what it claims to measure, and what it measures is an order of
     *  magnitude too coarse to help here. The correct conclusion is not to improve the roll estimate — a
     *  0.1° bank estimate needs an IMU bolted to the chassis, not a phone on glass — but to notice that the
     *  flat-road model is already better than the banked one, and that the bank therefore averages out over
     *  a drive rather than biasing it. The synthetic tests still exercise both paths, because on a road with
     *  a *known* bank the term is correct and the tests are what prove the sign is right.
     *
     *  Above `max_roll_std_deg` the road is taken as flat regardless, mirroring `paramsd`, which observes
     *  zero roll at a 10° sigma when its localizer cannot be trusted rather than skipping the sample. */
    bool use_roll = false;
  };

  ParamsLearner() : ParamsLearner(Config{}) {}
  explicit ParamsLearner(Config cfg) : cfg_(cfg) { reset(); }

  void setConfig(const Config& cfg)
  {
    cfg_ = cfg;
    reset();
  }
  const Config& config() const { return cfg_; }

  void reset()
  {
    x_ = {cfg_.stiffness_init, cfg_.steer_ratio_init, 0.0};
    p_ = {cfg_.stiffness_std_init * cfg_.stiffness_std_init, cfg_.steer_ratio_std_init * cfg_.steer_ratio_std_init,
          cfg_.angle_offset_std_init * cfg_.angle_offset_std_init};
    n_ = 0;
    have_prev_ay_ = false;
    prev_ay_ = 0.0;
  }

  /** One CAN sample. `yaw_rate_can` is `chassis.yaw_rate` as decoded (ISO, left-positive); `road_roll_deg`
   *  and its sigma come from `RoadRollEstimator`. Returns true if the sample was used. */
  bool update(double speed_ms, double swa_deg, double yaw_rate_can, double road_roll_deg, double road_roll_std_deg,
              double dt_s)
  {
    if (!(dt_s > 0.0) || dt_s > 1.0)
      return false;
    if (!std::isfinite(speed_ms) || !std::isfinite(swa_deg) || !std::isfinite(yaw_rate_can))
      return false;
    if (speed_ms < cfg_.min_speed_ms || std::abs(swa_deg) > cfg_.max_steer_deg)
      return false;
    if (std::abs(yaw_rate_can) > cfg_.max_yaw_rate)
      return false;

    const double yaw_meas = -yaw_rate_can;  // ISO in, z-down inside; see the class comment
    const double a_y = speed_ms * yaw_meas;
    if (have_prev_ay_ && std::abs(a_y - prev_ay_) / dt_s > cfg_.max_lateral_jerk) {
      prev_ay_ = a_y;
      return false;
    }
    prev_ay_ = a_y;
    have_prev_ay_ = true;

    // Roll only when it is worth having. Otherwise take the road as flat, which is what `paramsd` does by
    // observing zero at a 10° sigma rather than dropping the sample.
    const double roll = (cfg_.use_roll && std::isfinite(road_roll_deg) && road_roll_std_deg <= cfg_.max_roll_std_deg) ?
                            road_roll_deg :
                            0.0;

    // Random walk.
    p_[kStiffness] += cfg_.stiffness_process_std * cfg_.stiffness_process_std * dt_s;
    p_[kSteerRatio] += cfg_.steer_ratio_process_std * cfg_.steer_ratio_process_std * dt_s;
    p_[kAngleOffsetDeg] += cfg_.angle_offset_process_std * cfg_.angle_offset_process_std * dt_s;

    observeYawRate(speed_ms, swa_deg, roll, yaw_meas);
    // Cap the slow parameters' uncertainty. See `stiffness_std_max` for why this is a ceiling and not a
    // pseudo-measurement.
    p_[kStiffness] = std::min(p_[kStiffness], cfg_.stiffness_std_max * cfg_.stiffness_std_max);
    p_[kSteerRatio] = std::min(p_[kSteerRatio], cfg_.steer_ratio_std_max * cfg_.steer_ratio_std_max);

    clampStates();
    if (n_ < 1'000'000)
      ++n_;
    return true;
  }

  double stiffnessFactor() const { return x_[kStiffness]; }
  double steerRatio() const { return x_[kSteerRatio]; }
  double angleOffsetDeg() const { return x_[kAngleOffsetDeg]; }
  double stiffnessStd() const { return std::sqrt(std::max(p_[kStiffness], 0.0)); }
  double steerRatioStd() const { return std::sqrt(std::max(p_[kSteerRatio], 0.0)); }
  double angleOffsetStdDeg() const { return std::sqrt(std::max(p_[kAngleOffsetDeg], 0.0)); }
  int sampleCount() const { return n_; }

  /** Ready to be consumed.
   *
   *  Four kinds of check, and each one is here because the other three miss something. The count catches a
   *  filter that has barely run; the sigma catches one that ran on straights, where the measurement carries
   *  no information about stiffness and the covariance never shrinks. The bound checks are strict — `>` and
   *  `<`, not `>=` — because clamping lands a saturated state exactly on its bound, and saturation is the
   *  failure mode that most resembles success: the state stops moving and its sigma is small, which is what
   *  convergence looks like from the outside. The steer-ratio bounds were missing from this list until a bag
   *  replay with a flipped steering sign drove the ratio onto its ceiling and nothing here objected. */
  bool valid() const
  {
    return n_ >= 500 && stiffnessStd() < 0.15 && x_[kStiffness] > cfg_.stiffness_min &&
           x_[kStiffness] < cfg_.stiffness_max && x_[kSteerRatio] > cfg_.steer_ratio_min &&
           x_[kSteerRatio] < cfg_.steer_ratio_max && std::abs(x_[kAngleOffsetDeg]) < cfg_.angle_offset_max_deg;
  }

  /** The predicted yaw rate in the z-down frame for the current states. Public because the tests and the
   *  offline replay need exactly the function the filter uses, not a re-derivation of it. */
  double predictYawRate(double speed_ms, double swa_deg, double roll_deg) const
  {
    return predictWith(x_, speed_ms, swa_deg, roll_deg);
  }

private:
  using Vec = std::array<double, kNumStates>;

  double predictWith(const Vec& x, double speed_ms, double swa_deg, double roll_deg) const
  {
    constexpr double kG = 9.81;
    const double v = std::max(speed_ms, 1e-3);
    VehicleModelParams p = cfg_.vehicle;
    p.tire_stiffness_factor = std::max(x[kStiffness], 1e-3);
    const double slip = slipFactor(p);
    const double sign = cfg_.steer_sign < 0.0 ? -1.0 : 1.0;
    const double delta = sign * (swa_deg - x[kAngleOffsetDeg]) * M_PI / 180.0 / std::max(x[kSteerRatio], 1e-3);
    const double kappa = curvatureFromSteer(delta, v, p.wheelbase_m, slip);
    return v * kappa + kG * std::sin(roll_deg * M_PI / 180.0) / v;
  }

  void observeYawRate(double speed_ms, double swa_deg, double roll_deg, double yaw_meas)
  {
    const double h = predictWith(x_, speed_ms, swa_deg, roll_deg);
    const double innov = yaw_meas - h;

    // Central-difference Jacobian; steps are a small fraction of each state's own scale.
    const Vec step = {1e-3, 1e-2, 1e-2};
    Vec jac{};
    for (int i = 0; i < kNumStates; ++i) {
      Vec hi = x_;
      Vec lo = x_;
      hi[i] += step[i];
      lo[i] -= step[i];
      jac[i] = (predictWith(hi, speed_ms, swa_deg, roll_deg) - predictWith(lo, speed_ms, swa_deg, roll_deg)) /
               (2.0 * step[i]);
    }

    // Diagonal covariance keeps this a few lines and costs little: the three parameters are not strongly
    // correlated through a single scalar measurement, and pretending otherwise would need a real matrix.
    double s = cfg_.yaw_rate_std * cfg_.yaw_rate_std;
    for (int i = 0; i < kNumStates; ++i)
      s += jac[i] * p_[i] * jac[i];
    if (!(s > 0.0))
      return;
    for (int i = 0; i < kNumStates; ++i) {
      const double k = p_[i] * jac[i] / s;
      x_[i] += k * innov;
      p_[i] = std::max((1.0 - k * jac[i]) * p_[i], 1e-12);
    }
  }

  void clampStates()
  {
    x_[kStiffness] = std::clamp(x_[kStiffness], cfg_.stiffness_min, cfg_.stiffness_max);
    x_[kSteerRatio] = std::clamp(x_[kSteerRatio], cfg_.steer_ratio_min, cfg_.steer_ratio_max);
    x_[kAngleOffsetDeg] = std::clamp(x_[kAngleOffsetDeg], -cfg_.angle_offset_max_deg, cfg_.angle_offset_max_deg);
  }

  Config cfg_{};
  Vec x_{};
  Vec p_{};
  int n_ = 0;
  bool have_prev_ay_ = false;
  double prev_ay_ = 0.0;
};

}  // namespace adas
