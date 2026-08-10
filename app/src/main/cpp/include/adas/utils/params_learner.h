#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "adas/utils/vehicle_model.h"

namespace adas {

class ParamsLearner {
public:
  enum State {
    kStiffness = 0,
    kSteerRatio = 1,
    kAngleOffset = 2,      //!< рад
    kAngleOffsetFast = 3,  //!< рад
    kU = 4,                //!< продольная скорость, м/с
    kV = 5,                //!< боковая скорость, м/с
    kYawRate = 6,          //!< рад/с, z вниз
    kSteerAngle = 7,       //!< угол руля, рад
    kRoadRoll = 8,         //!< рад
    kNumStates = 9
  };

  struct Config {
    VehicleModelParams vehicle{};

    double rotational_inertia = 2468.4;

    double stiffness_init = 1.0;
    double steer_ratio_init = 15.7;
    double angle_offset_init_deg = 0.0;

    double stiffness_min = 0.2;
    double stiffness_max = 5.0;
    double steer_ratio_min = 0.0;  //!< 0 — взять 0.5·steer_ratio_init
    double steer_ratio_max = 0.0;  //!< 0 — взять 2.0·steer_ratio_init
    double angle_offset_max_deg = 10.0;

    double steer_sign = 1.0;

    double stiffness_process_std = 0.05 / 100.0;
    double steer_ratio_process_std = 0.01;
    double angle_offset_process_std_deg = 0.02;
    double angle_offset_fast_process_std_deg = 0.25;
    double u_process_std = 0.1;
    double v_process_std = 0.01;
    double yaw_rate_process_std_deg = 0.1;
    double steer_angle_process_std_deg = 0.1;
    double roll_process_std_deg = 1.0;

    double p_initial_scale = 1.0;
    double stiffness_p0_std = 0.0;  //!< 0 — брать из Q
    double steer_ratio_p0_std = 0.0;

    double yaw_rate_std = 0.02;
    double steer_angle_std_deg = 0.05;
    double angle_offset_fast_obs_std_deg = 10.0;
    double steer_ratio_obs_std = 5.0;
    double stiffness_obs_std = 0.5;
    double speed_obs_std = 0.1;
    double roll_obs_std_deg = 1.0;

    double min_speed_ms = 1.0;
    double max_steer_deg = 45.0;
    double max_yaw_rate = 1.0;
    double max_lateral_jerk = 0.0;  //!< 0 — не отбрасывать переходные процессы: модель динамическая
    double max_roll_std_deg = 1.5;

    double excited_swa_deg = 2.0;
    int min_excited_samples = 500;

    bool use_roll = false;

    double stiffness_std_init = 0.0;
    double steer_ratio_std_init = 0.0;
    double angle_offset_std_init = 0.0;
    double stiffness_std_max = 0.0;
    double steer_ratio_std_max = 0.0;
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

  double stiffnessFactor() const { return x_[kStiffness]; }
  double steerRatio() const { return x_[kSteerRatio]; }
  double angleOffsetDeg() const { return x_[kAngleOffset] * 180.0 / M_PI; }
  double angleOffsetTotalDeg() const { return (x_[kAngleOffset] + x_[kAngleOffsetFast]) * 180.0 / M_PI; }
  double roadRollDeg() const { return x_[kRoadRoll] * 180.0 / M_PI; }
  double yawRate() const { return x_[kYawRate]; }
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
    Mat F = Mat::Identity();  // столбцы заполняются ниже численно
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
