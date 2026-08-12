#include "adas/utils/vehicle_ekf.h"

#include <cmath>

namespace adas {
VehicleEKF::VehicleEKF(double wheelbase_, double gps_noise_pos, double imu_noise_yaw_rate) : wheelbase(wheelbase_)
{
  Q_.setZero();
  Q_(0, 0) = 0.1 * 0.1;
  Q_(1, 1) = 0.1 * 0.1;
  Q_(2, 2) = 0.01 * 0.01;
  Q_(3, 3) = 0.5 * 0.5;
  Q_(4, 4) = 0.05 * 0.05;
  R_gps_ = Mat2::Identity() * (gps_noise_pos * gps_noise_pos);
  R_imu_ = imu_noise_yaw_rate * imu_noise_yaw_rate;
  reset();
}

void VehicleEKF::reset(double x, double y, double yaw, double v, double yaw_rate, double pos_unc, double yaw_unc,
                       double v_unc, double yaw_rate_unc)
{
  state_ << x, y, yaw, v, yaw_rate;
  P_.setZero();
  P_(0, 0) = pos_unc * pos_unc;
  P_(1, 1) = pos_unc * pos_unc;
  P_(2, 2) = yaw_unc * yaw_unc;
  P_(3, 3) = v_unc * v_unc;
  P_(4, 4) = yaw_rate_unc * yaw_rate_unc;
  prediction_count = gps_update_count = gps_rejected_count = gps_reseed_count = 0;
  gps_yaw_update_count = gps_vel_update_count = imu_update_count = cam_odo_update_count = 0;
  model_update_count = wheel_speed_update_count = 0;
  consecutive_gps_rejects_ = 0;
}

void VehicleEKF::predict(double v_measured, double steering_angle, double dt)
{
  const double x = state_(0);
  const double y = state_(1);
  const double yaw = state_(2);
  const double v = state_(3);

  double yaw_rate_model = 0.0;
  const bool have_model = std::abs(steering_angle) > 0.001 && std::abs(v_measured) > 0.01;
  if (have_model)
    yaw_rate_model = v_measured * std::tan(steering_angle) / wheelbase;

  const double yaw_rate_used = yaw_rate_is_state_ ? state_(4) : yaw_rate_model;

  state_(0) = x + v * std::cos(yaw) * dt;
  state_(1) = y + v * std::sin(yaw) * dt;
  state_(2) = normalizeAngle(yaw + yaw_rate_used * dt);
  if (!speed_is_state_)
    state_(3) = v_measured;
  if (!yaw_rate_is_state_)
    state_(4) = yaw_rate_model;

  Mat5 F = Mat5::Identity();
  F(0, 2) = -v * std::sin(yaw) * dt;
  F(0, 3) = std::cos(yaw) * dt;
  F(1, 2) = v * std::cos(yaw) * dt;
  F(1, 3) = std::sin(yaw) * dt;
  F(2, 4) = dt;

  P_ = F * P_ * F.transpose() + Q_;
  ++prediction_count;

  if (yaw_rate_is_state_ && have_model)
    applyYawRateUpdate(yaw_rate_model, R_model_, YawRateSource::BicycleModel);
  if (speed_is_state_)
    applySpeedUpdate(v_measured, R_wheel_);
}

void VehicleEKF::applySpeedUpdate(double v_meas, double R)
{
  if (!std::isfinite(v_meas) || !(R > 0.0))
    return;
  const double S = P_(3, 3) + R;
  if (std::abs(S) < 1e-12)
    return;

  Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
  H(0, 3) = 1.0;
  const Vec5 K = P_ * H.transpose() / S;
  state_ += K * (v_meas - state_(3));
  state_(2) = normalizeAngle(state_(2));

  const Mat5 IKH = Mat5::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + (R * K) * K.transpose();
  ++wheel_speed_update_count;
}

VehicleEKF::GpsPosResult VehicleEKF::updateGps(double gps_x, double gps_y, double max_innovation,
                                               double reseed_innovation, bool allow_reseed, double R_pos)
{
  const Vec2 innov(gps_x - state_(0), gps_y - state_(1));
  const double mag = innov.norm();

  if (allow_reseed && (mag > reseed_innovation || (mag > max_innovation && consecutive_gps_rejects_ >= 4))) {
    state_(0) = gps_x;
    state_(1) = gps_y;
    P_(0, 0) = std::max(P_(0, 0), 25.0);
    P_(1, 1) = std::max(P_(1, 1), 25.0);
    P_(2, 2) = std::max(P_(2, 2), 0.25);
    consecutive_gps_rejects_ = 0;
    ++gps_reseed_count;
    ++gps_update_count;
    return GpsPosResult::Reseeded;
  }

  if (mag > max_innovation) {
    ++gps_rejected_count;
    ++consecutive_gps_rejects_;
    return GpsPosResult::Rejected;
  }

  Mat25 H = Mat25::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  Mat2 R = R_gps_;
  if (R_pos > 0.0) {
    R = Mat2::Identity() * R_pos;
  }
  if (mag > 0.5 * max_innovation) {
    const double scale = (mag / max_innovation) * (mag / max_innovation);
    R *= std::max(1.0, 4.0 * scale);
  }

  const Mat2 S = H * P_ * H.transpose() + R;
  Eigen::FullPivLU<Mat2> lu(S);
  if (!lu.isInvertible()) {
    ++gps_rejected_count;
    ++consecutive_gps_rejects_;
    return GpsPosResult::Rejected;
  }

  const Mat52 K = P_ * H.transpose() * lu.inverse();
  state_ += K * innov;
  state_(2) = normalizeAngle(state_(2));

  const Mat5 IKH = Mat5::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  consecutive_gps_rejects_ = 0;
  ++gps_update_count;
  return GpsPosResult::Accepted;
}

bool VehicleEKF::updateGpsYaw(double yaw_enu, double R_yaw, bool force)
{
  if (!(R_yaw > 0.0) || !std::isfinite(yaw_enu))
    return false;
  const double innov = normalizeAngle(yaw_enu - state_(2));
  if (!force && std::abs(innov) > 1.2)
    return false;

  const double S = P_(2, 2) + R_yaw;
  if (std::abs(S) < 1e-12)
    return false;
  const double K = P_(2, 2) / S;
  state_(2) = normalizeAngle(state_(2) + K * innov);
  for (int i = 0; i < 5; ++i) {
    if (i == 2)
      continue;
    P_(2, i) *= (1.0 - K);
    P_(i, 2) = P_(2, i);
  }
  P_(2, 2) = (1.0 - K) * P_(2, 2) * (1.0 - K) + K * R_yaw * K;
  ++gps_yaw_update_count;
  return true;
}

bool VehicleEKF::updateGpsVel(double vx_east, double vy_north, double R_vel)
{
  if (!(R_vel > 0.0) || !std::isfinite(vx_east) || !std::isfinite(vy_north))
    return false;
  const double yaw = state_(2);
  const double c = std::cos(yaw), s = std::sin(yaw);
  // z = [vx, vy] = v * [cos ψ, sin ψ]  (ENU)
  const Vec2 z_pred(state_(3) * c, state_(3) * s);
  const Vec2 innov(vx_east - z_pred(0), vy_north - z_pred(1));
  if (innov.norm() > 15.0)
    return false;

  Mat25 H = Mat25::Zero();
  H(0, 2) = -state_(3) * s;
  H(0, 3) = c;
  H(1, 2) = state_(3) * c;
  H(1, 3) = s;
  const Mat2 R = Mat2::Identity() * R_vel;
  const Mat2 S = H * P_ * H.transpose() + R;
  Eigen::FullPivLU<Mat2> lu(S);
  if (!lu.isInvertible())
    return false;
  const Mat52 K = P_ * H.transpose() * lu.inverse();
  state_ += K * innov;
  state_(2) = normalizeAngle(state_(2));
  const Mat5 IKH = Mat5::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  ++gps_vel_update_count;
  return true;
}

void VehicleEKF::applyYawRateUpdate(double yaw_rate_meas, double R, YawRateSource source)
{
  if (!std::isfinite(yaw_rate_meas) || !(R > 0.0))
    return;
  const double innov = yaw_rate_meas - state_(4);
  const double S = P_(4, 4) + R;
  if (std::abs(S) < 1e-12)
    return;

  Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
  H(0, 4) = 1.0;
  const Vec5 K = P_ * H.transpose() / S;
  state_ += K * innov;
  state_(2) = normalizeAngle(state_(2));

  const Mat5 IKH = Mat5::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + (R * K) * K.transpose();
  switch (source) {
    case YawRateSource::Imu:
      ++imu_update_count;
      break;
    case YawRateSource::CameraOdometry:
      ++cam_odo_update_count;
      break;
    case YawRateSource::BicycleModel:
      ++model_update_count;
      break;
  }
}

}  // namespace adas
