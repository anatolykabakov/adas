#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "adas/utils/vehicle_model.h"

namespace adas {
/**
 * \brief Learns steering ratio, tyre stiffness and the angle zero while driving.
 *
 * \details The port of upstream's `paramsd`: a small filter over steering angle, speed and yaw rate. The
 * estimate only reaches the controller when `lane_keep.use_learned_params` says so, so enabling the
 * learner changes no command by itself — the honest sequence is to drive, compare against the configured
 * constants in the bag, and only then hand it the wheel.
 */
class ParamsLearner {
public:
  enum State {
    kStiffness = 0,
    kSteerRatio = 1,
    kAngleOffset = 2,
    kAngleOffsetFast = 3,
    kU = 4,
    kV = 5,
    kYawRate = 6,
    kSteerAngle = 7,
    kRoadRoll = 8,
    kNumStates = 9
  };

  struct Config {
    VehicleModelParams vehicle{};  ///< The reference car the estimate is relative to.

    double rotational_inertia = 2468.4;  ///< Yaw inertia [kg*m^2]; scaled from the reference car by mass and wheelbase.

    double stiffness_init = 1.0;         ///< Prior on the stiffness factor.
    double steer_ratio_init = 15.7;      ///< Prior on the steering ratio.
    double angle_offset_init_deg = 0.0;  ///< Prior on the steering-wheel zero [deg].

    double stiffness_min = 0.2;    ///< Lower clamp on stiffness. Hitting it usually means the steering sign is wrong.
    double stiffness_max = 5.0;    ///< Upper clamp on the stiffness estimate.
    double steer_ratio_min = 0.0;  ///< Clamp on the ratio estimate; 0 derives it from the prior.
    double steer_ratio_max = 0.0;  ///< Upper clamp on the ratio estimate; 0 derives it from the prior.
    /// Clamp on the learned zero [deg]; beyond this it is a sensor fault, not a misalignment.
    double angle_offset_max_deg = 10.0;

    /// Sign convention of the measured wheel angle. Wrong here and the estimate cannot converge.
    double steer_sign = 1.0;

    double stiffness_process_std = 0.05 / 100.0;  ///< How fast stiffness is allowed to drift per step.
    double steer_ratio_process_std = 0.01;        ///< How fast the ratio is allowed to drift per step.
    /// Drift allowed for the slow zero [deg/step]: it is mechanical and nearly constant.
    double angle_offset_process_std_deg = 0.02;
    /// Drift allowed for the fast zero [deg], which absorbs road camber.
    double angle_offset_fast_process_std_deg = 0.25;
    double u_process_std = 0.1;                ///< Process noise on longitudinal speed.
    double v_process_std = 0.01;               ///< Process noise on lateral speed.
    double yaw_rate_process_std_deg = 0.1;     ///< Process noise on yaw rate [deg/s].
    double steer_angle_process_std_deg = 0.1;  ///< Process noise on the steering angle [deg/s].
    double roll_process_std_deg = 1.0;         ///< Drift allowed for the road-bank state [deg/step].

    double p_initial_scale = 1.0;     ///< Scale on the whole initial covariance; larger means less trust in the priors.
    double stiffness_p0_std = 0.0;    ///< Initial spread on stiffness; 0 derives it from `p_initial_scale`.
    double steer_ratio_p0_std = 0.0;  ///< Initial spread on the ratio; 0 derives it.

    double yaw_rate_std = 0.02;         ///< Assumed yaw-rate measurement noise [rad/s].
    double steer_angle_std_deg = 0.05;  ///< Assumed steering-angle measurement noise [deg].
    /// Observation noise on the fast zero [deg]; deliberately loose, it absorbs camber.
    double angle_offset_fast_obs_std_deg = 10.0;
    double steer_ratio_obs_std = 5.0;  ///< Observation noise on the ratio.
    double stiffness_obs_std = 0.5;    ///< Observation noise on stiffness.
    double speed_obs_std = 0.1;        ///< Observation noise on speed [m/s].
    double roll_obs_std_deg = 1.0;     ///< Observation noise on the road bank [deg].

    double min_speed_ms = 1.0;  ///< Below this speed nothing is learned [m/s].
    /// Samples with a larger wheel angle are dropped [deg]: the linear model stops holding.
    double max_steer_deg = 45.0;
    double max_yaw_rate = 1.0;      ///< Samples with a larger yaw rate are dropped [rad/s].
    double max_lateral_jerk = 0.0;  ///< Samples during faster transients are dropped [m/s^3]; 0 disables the check.
    double max_roll_std_deg = 1.5;  ///< Samples with a worse bank estimate are dropped [deg].

    double excited_swa_deg = 2.0;   ///< Wheel angle above which the drive counts as exciting the parameters [deg].
    int min_excited_samples = 500;  ///< Exciting samples needed before the estimate is reported as usable.

    bool use_roll = false;  ///< Include the road bank as a state; needs a bank estimate worth using.

    double stiffness_std_init = 0.0;     ///< Reported initial spreads; 0 derives them from the covariance.
    double steer_ratio_std_init = 0.0;   ///< Initial spread reported for the ratio.
    double angle_offset_std_init = 0.0;  ///< Initial spread reported for the zero.
    double stiffness_std_max = 0.0;      ///< Spread above which stiffness is not handed to the controller.
    double steer_ratio_std_max = 0.0;    ///< Spread above which the ratio is not handed over.
  };

  ParamsLearner() : ParamsLearner(Config{}) {}
  explicit ParamsLearner(Config cfg) : cfg_(cfg) { reset(); }

  /// Replace the configuration and restart the estimate, since the priors changed under it.
  void setConfig(const Config& cfg)
  {
    cfg_ = cfg;
    reset();
  }
  const Config& config() const { return cfg_; }

  /// Back to the priors: state to the configured initial values, covariance to its initial spread.
  void reset()
  {
    x_.setZero();
    x_[kStiffness] = cfg_.stiffness_init;
    x_[kSteerRatio] = cfg_.steer_ratio_init;
    x_[kAngleOffset] = cfg_.angle_offset_init_deg * M_PI / 180.0;
    x_[kU] = 10.0;

    q_.setZero();
    const double d2r = M_PI / 180.0;
    q_[kStiffness] = sq(cfg_.stiffness_process_std);
    q_[kSteerRatio] = sq(cfg_.steer_ratio_process_std);
    q_[kAngleOffset] = sq(cfg_.angle_offset_process_std_deg * d2r);
    q_[kAngleOffsetFast] = sq(cfg_.angle_offset_fast_process_std_deg * d2r);
    q_[kU] = sq(cfg_.u_process_std);
    q_[kV] = sq(cfg_.v_process_std);
    q_[kYawRate] = sq(cfg_.yaw_rate_process_std_deg * d2r);
    q_[kSteerAngle] = sq(cfg_.steer_angle_process_std_deg * d2r);
    q_[kRoadRoll] = sq(cfg_.roll_process_std_deg * d2r);

    p_ = (q_ * cfg_.p_initial_scale).asDiagonal();
    if (cfg_.stiffness_p0_std > 0.0)
      p_(kStiffness, kStiffness) = sq(cfg_.stiffness_p0_std);
    if (cfg_.steer_ratio_p0_std > 0.0)
      p_(kSteerRatio, kSteerRatio) = sq(cfg_.steer_ratio_p0_std);
    n_ = 0;
    excited_ = 0;
    have_prev_ay_ = false;
    prev_ay_ = 0.0;
  }

  /**
   * \brief One learner step.
   *
   * \param[in] speed_ms Ego speed [m/s]. Below the configured minimum nothing is learned: at rest the
   * steering angle says nothing about the ratio.
   * \param[in] swa_deg Measured steering-wheel angle [deg].
   * \param[in] yaw_rate_can Yaw rate from CAN [rad/s]. Deliberately not the EKF's — that one is partly
   * produced by the bicycle model whose parameters are being estimated here.
   * \param[in] road_roll_deg Road bank [deg], which otherwise looks like understeer.
   * \param[in] road_roll_std_deg Its standard deviation [deg]; a bank known badly is weighted down.
   * \param[in] dt Step [s].
   * \return True when the sample was used.
   */
  bool update(double speed_ms, double swa_deg, double yaw_rate_can, double road_roll_deg, double road_roll_std_deg,
              double dt_s, bool localizer_tick = true)
  {
    if (!(dt_s > 0.0) || dt_s > 1.0)
      return false;
    if (!std::isfinite(speed_ms) || !std::isfinite(swa_deg) || !std::isfinite(yaw_rate_can))
      return false;
    if (speed_ms < cfg_.min_speed_ms || std::abs(swa_deg) > cfg_.max_steer_deg)
      return false;
    if (std::abs(yaw_rate_can) > cfg_.max_yaw_rate)
      return false;

    const double yaw_meas = -yaw_rate_can;
    if (cfg_.max_lateral_jerk > 0.0) {
      const double a_y = speed_ms * yaw_meas;
      if (have_prev_ay_ && std::abs(a_y - prev_ay_) / dt_s > cfg_.max_lateral_jerk) {
        prev_ay_ = a_y;
        return false;
      }
      prev_ay_ = a_y;
      have_prev_ay_ = true;
    }

    predict(dt_s);

    const double d2r = M_PI / 180.0;
    observe(kSteerAngle, cfg_.steer_sign * swa_deg * d2r, sq(cfg_.steer_angle_std_deg * d2r));
    observe(kU, speed_ms, sq(cfg_.speed_obs_std));

    if (localizer_tick) {
      observe(kYawRate, yaw_meas, sq(cfg_.yaw_rate_std));
      const bool roll_ok = cfg_.use_roll && std::isfinite(road_roll_deg) && road_roll_std_deg <= cfg_.max_roll_std_deg;
      observe(kRoadRoll, roll_ok ? road_roll_deg * d2r : 0.0,
              sq((roll_ok ? std::max(road_roll_std_deg, 0.1) : 10.0) * d2r));
      observe(kAngleOffsetFast, 0.0, sq(cfg_.angle_offset_fast_obs_std_deg * d2r));
      observe(kStiffness, x_[kStiffness], sq(cfg_.stiffness_obs_std));
      observe(kSteerRatio, x_[kSteerRatio], sq(cfg_.steer_ratio_obs_std));
    }

    clampStates();
    if (n_ < 1'000'000)
      ++n_;
    if (std::abs(swa_deg) > cfg_.excited_swa_deg && excited_ < 1'000'000)
      ++excited_;
    return true;
  }

  /// Learned scale on the reference tyre stiffness; 1.0 is the reference car.
  double stiffnessFactor() const { return x_[kStiffness]; }
  /// Learned steering-wheel to road-wheel ratio.
  double steerRatio() const { return x_[kSteerRatio]; }
  /// Slow part of the steering-wheel zero [deg] — the mechanical misalignment.
  double angleOffsetDeg() const { return x_[kAngleOffset] * 180.0 / M_PI; }
  double angleOffsetTotalDeg() const { return (x_[kAngleOffset] + x_[kAngleOffsetFast]) * 180.0 / M_PI; }
  /// Road bank as the filter currently believes it [deg].
  double roadRollDeg() const { return x_[kRoadRoll] * 180.0 / M_PI; }
  /// Yaw rate the learner's own model predicts [rad/s], for comparison against CAN.
  double yawRate() const { return x_[kYawRate]; }
  /// Standard deviations of the estimates. The gate for handing them to the controller: a number with a
  /// wide spread is a guess.
  double stiffnessStd() const { return std::sqrt(std::max(p_(kStiffness, kStiffness), 0.0)); }
  double steerRatioStd() const { return std::sqrt(std::max(p_(kSteerRatio, kSteerRatio), 0.0)); }
  double angleOffsetStdDeg() const { return std::sqrt(std::max(p_(kAngleOffset, kAngleOffset), 0.0)) * 180.0 / M_PI; }
  int sampleCount() const { return n_; }
  double cov(int i, int j) const { return p_(i, j); }

  int excitedCount() const { return excited_; }

  bool valid() const
  {
    return n_ >= 500 && excited_ >= cfg_.min_excited_samples && stiffnessStd() < 0.15 &&
           x_[kStiffness] > cfg_.stiffness_min && x_[kStiffness] < cfg_.stiffness_max &&
           x_[kSteerRatio] > steerRatioMin() && x_[kSteerRatio] < steerRatioMax() &&
           std::abs(angleOffsetDeg()) < cfg_.angle_offset_max_deg;
  }

  double predictYawRate(double speed_ms, double swa_deg, double roll_deg) const
  {
    constexpr double kG = 9.81;
    const double v = std::max(speed_ms, 1e-3);
    VehicleModelParams p = cfg_.vehicle;
    p.tire_stiffness_factor = std::max(x_[kStiffness], 1e-3);
    const double sign = cfg_.steer_sign < 0.0 ? -1.0 : 1.0;
    const double delta = sign * (swa_deg - angleOffsetDeg()) * M_PI / 180.0 / std::max(x_[kSteerRatio], 1e-3);
    return v * curvatureFromSteer(delta, v, p.wheelbase_m, slipFactor(p)) + kG * std::sin(roll_deg * M_PI / 180.0) / v;
  }

private:
  using Vec = Eigen::Matrix<double, kNumStates, 1>;
  using Mat = Eigen::Matrix<double, kNumStates, kNumStates>;

  static constexpr double kMaxSubStepS = 0.002;

  static double sq(double a) { return a * a; }
  double steerRatioMin() const
  {
    return cfg_.steer_ratio_min > 0.0 ? cfg_.steer_ratio_min : 0.5 * cfg_.steer_ratio_init;
  }
  double steerRatioMax() const
  {
    return cfg_.steer_ratio_max > 0.0 ? cfg_.steer_ratio_max : 2.0 * cfg_.steer_ratio_init;
  }

  void baseStiffness(double& cF, double& cR) const
  {
    constexpr double kCivicMass = 1326.0 + 136.0;
    constexpr double kCivicWheelbase = 2.70;
    constexpr double kCivicC2F = kCivicWheelbase * 0.4;
    constexpr double kCivicC2R = kCivicWheelbase - kCivicC2F;
    const double wb = cfg_.vehicle.wheelbase_m;
    const double aF = wb * cfg_.vehicle.center_to_front_frac;
    const double aR = wb - aF;
    cF = 192150.0 * cfg_.vehicle.mass_kg / kCivicMass * (aR / wb) / (kCivicC2R / kCivicWheelbase);
    cR = 202500.0 * cfg_.vehicle.mass_kg / kCivicMass * (aF / wb) / (kCivicC2F / kCivicWheelbase);
  }

  Vec derivative(const Vec& x) const
  {
    constexpr double kG = 9.81;
    Vec d = Vec::Zero();
    const double u = std::max(x[kU], 1.0);
    double cF0 = 0.0, cR0 = 0.0;
    baseStiffness(cF0, cR0);
    const double sf = std::max(x[kStiffness], 1e-3);
    const double cF = sf * cF0, cR = sf * cR0;
    const double m = cfg_.vehicle.mass_kg;
    const double j = std::max(cfg_.rotational_inertia, 1e-3);
    const double wb = cfg_.vehicle.wheelbase_m;
    const double aF = wb * cfg_.vehicle.center_to_front_frac;
    const double aR = wb - aF;
    const double sR = std::max(x[kSteerRatio], 1e-3);

    const double a00 = -(cF + cR) / (m * u);
    const double a01 = -(cF * aF - cR * aR) / (m * u) - u;
    const double a10 = -(cF * aF - cR * aR) / (j * u);
    const double a11 = -(cF * aF * aF + cR * aR * aR) / (j * u);
    const double b0 = cF / (m * sR);
    const double b1 = cF * aF / (j * sR);
    const double sa = x[kSteerAngle] - x[kAngleOffset] - x[kAngleOffsetFast];

    d[kV] = a00 * x[kV] + a01 * x[kYawRate] + b0 * sa - kG * x[kRoadRoll];
    d[kYawRate] = a10 * x[kV] + a11 * x[kYawRate] + b1 * sa;
    return d;
  }

  Vec integrate(const Vec& x, double dt) const
  {
    const int sub = std::clamp(static_cast<int>(std::ceil(dt / kMaxSubStepS)), 1, 64);
    const double h = dt / sub;
    Vec y = x;
    for (int i = 0; i < sub; ++i)
      y += h * derivative(y);
    return y;
  }

  void predict(double dt)
  {
    const Vec f = integrate(x_, dt);

    static const Vec kStep = (Vec() << 1e-4, 1e-3, 1e-5, 1e-5, 1e-3, 1e-4, 1e-5, 1e-5, 1e-5).finished();
    Mat F = Mat::Identity();
    for (int i = 0; i < kNumStates; ++i) {
      Vec hi = x_, lo = x_;
      hi[i] += kStep[i];
      lo[i] -= kStep[i];
      F.col(i) = (integrate(hi, dt) - integrate(lo, dt)) / (2.0 * kStep[i]);
    }

    x_ = f;
    p_ = F * p_ * F.transpose();
    p_.diagonal() += q_ * dt;
  }

  void observe(int state, double z, double r)
  {
    const double s = p_(state, state) + r;
    if (!(s > 0.0))
      return;
    const Vec k = p_.col(state) / s;
    x_ += k * (z - x_[state]);
    p_ -= k * p_.row(state);
    p_ = 0.5 * (p_ + p_.transpose()).eval();
  }

  void clampStates()
  {
    x_[kStiffness] = std::clamp(x_[kStiffness], cfg_.stiffness_min, cfg_.stiffness_max);
    x_[kSteerRatio] = std::clamp(x_[kSteerRatio], steerRatioMin(), steerRatioMax());
    const double max_off = cfg_.angle_offset_max_deg * M_PI / 180.0;
    x_[kAngleOffset] = std::clamp(x_[kAngleOffset], -max_off, max_off);
    x_[kAngleOffsetFast] = std::clamp(x_[kAngleOffsetFast], -max_off, max_off);
  }

  Config cfg_{};
  Vec x_ = Vec::Zero();
  Vec q_ = Vec::Zero();
  Mat p_ = Mat::Zero();
  int n_ = 0;
  int excited_ = 0;
  bool have_prev_ay_ = false;
  double prev_ay_ = 0.0;
};

}  // namespace adas
