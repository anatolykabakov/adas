#include "utils/imu_calibrator.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace adas {

ImuCalibrator::ImuCalibrator(double speed_threshold_mps, int min_samples, int max_buffer, bool invert_yaw_rate)
  : speed_threshold_mps_(speed_threshold_mps)
  , min_samples_(std::max(8, min_samples))
  , max_buffer_(std::max(min_samples_, max_buffer))
  , invert_yaw_rate_(invert_yaw_rate)
{
}

void ImuCalibrator::reset()
{
  has_prior_ = false;
  orientation_locked_ = false;
  bias_ = Vec3::Zero();
  R_ = Mat3::Identity();
  accel_buf_.clear();
  gyro_buf_.clear();
  speed_mps_ = 0.0;
}

void ImuCalibrator::setSpeed(double speed_mps) { speed_mps_ = std::max(0.0, speed_mps); }

void ImuCalibrator::setMountPrior(double roll_deg, double pitch_deg, double yaw_deg)
{
  R_ = rotationFromMountRpy(roll_deg, pitch_deg, yaw_deg);
  has_prior_ = true;
}

Mat3 ImuCalibrator::rotationFromMountRpy(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double r = roll_deg * M_PI / 180.0;
  const double p = pitch_deg * M_PI / 180.0;
  const double y = yaw_deg * M_PI / 180.0;
  return (Eigen::AngleAxisd(y, Vec3::UnitZ()) * Eigen::AngleAxisd(p, Vec3::UnitY()) *
          Eigen::AngleAxisd(r, Vec3::UnitX()))
      .toRotationMatrix();
}

Mat3 ImuCalibrator::rotationFromGravityKeepingHeading(const Vec3& accel, const Mat3& heading_ref)
{
  const Mat3 from_gravity = rotationFromGravity(accel);
  // Both rotations agree about where down is; they differ by a rotation about it. Take that difference out
  // of `from_gravity` and put the reference's heading back in.
  const Vec3 down(0.0, 0.0, 1.0);
  const Vec3 fwd_ref = heading_ref.transpose() * Vec3(1.0, 0.0, 0.0);
  // Project the reference forward axis into the plane gravity defines, expressed in the gravity frame.
  Vec3 fwd_in_gravity = from_gravity * fwd_ref;
  fwd_in_gravity -= fwd_in_gravity.dot(down) * down;
  if (fwd_in_gravity.norm() < 1e-6)
    return from_gravity;  // reference forward is straight down: nothing to preserve
  fwd_in_gravity.normalize();
  const double heading = std::atan2(fwd_in_gravity.y(), fwd_in_gravity.x());
  Mat3 unspin = Mat3::Identity();
  const double c = std::cos(-heading), s = std::sin(-heading);
  unspin(0, 0) = c;
  unspin(0, 1) = -s;
  unspin(1, 0) = s;
  unspin(1, 1) = c;
  return unspin * from_gravity;
}

Mat3 ImuCalibrator::rotationFromGravity(const Vec3& accel)
{
  const double n = accel.norm();
  if (n < 1e-6)
    return Mat3::Identity();

  // Rotate measured gravity direction onto vehicle +down (-Z).
  return Eigen::Quaterniond::FromTwoVectors(accel / n, Vec3(0.0, 0.0, -1.0)).toRotationMatrix();
}

bool ImuCalibrator::isQuiet(const RawImuSample& raw) const
{
  const double g = Vec3(raw.ax, raw.ay, raw.az).norm();
  const double g_err = std::abs(g - 9.81);
  const double w = Vec3(raw.gx, raw.gy, raw.gz).norm();
  return g_err <= accel_g_err_max_ && w <= gyro_quiet_max_;
}

void ImuCalibrator::tryLockOrientation()
{
  if (orientation_locked_)
    return;
  if (static_cast<int>(accel_buf_.size()) < min_samples_ || static_cast<int>(gyro_buf_.size()) < min_samples_) {
    return;
  }

  Vec3 sum_a = Vec3::Zero();
  for (const auto& a : accel_buf_)
    sum_a += a;
  const Vec3 mean_a = sum_a / static_cast<double>(accel_buf_.size());
  // Keep the heading: gravity does not measure it, and overwriting it here used to discard the prior's.
  R_ = has_prior_ ? rotationFromGravityKeepingHeading(mean_a, R_) : rotationFromGravity(mean_a);

  Vec3 sum_g = Vec3::Zero();
  for (const auto& g : gyro_buf_)
    sum_g += g;
  bias_ = sum_g / static_cast<double>(gyro_buf_.size());

  orientation_locked_ = true;
  accel_buf_.clear();
  gyro_buf_.clear();
}

void ImuCalibrator::updateBiasEma(const Vec3& gyro)
{
  const double a = bias_ema_alpha_;
  bias_ = (1.0 - a) * bias_ + a * gyro;
}

double ImuCalibrator::apply(const Vec3& gyro) const
{
  double yaw = (R_ * (gyro - bias_)).z();
  if (invert_yaw_rate_)
    yaw = -yaw;
  return yaw;
}

std::optional<double> ImuCalibrator::push(const RawImuSample& raw)
{
  if (!raw.valid)
    return std::nullopt;

  const bool stationary = speed_mps_ < speed_threshold_mps_;
  const bool quiet = isQuiet(raw);
  const Vec3 accel(raw.ax, raw.ay, raw.az);
  const Vec3 gyro(raw.gx, raw.gy, raw.gz);

  if (stationary && quiet) {
    if (!orientation_locked_) {
      accel_buf_.push_back(accel);
      gyro_buf_.push_back(gyro);
      while (static_cast<int>(accel_buf_.size()) > max_buffer_)
        accel_buf_.erase(accel_buf_.begin());
      while (static_cast<int>(gyro_buf_.size()) > max_buffer_)
        gyro_buf_.erase(gyro_buf_.begin());
      tryLockOrientation();
    } else {
      updateBiasEma(gyro);
    }
  }

  if (!has_prior_ && !orientation_locked_)
    return std::nullopt;
  // Lateral specific force, for the road-roll estimator. Recorded on every accepted sample so it stays in
  // step with the yaw rate returned below; meaningless without a heading, which `hasHeading` reports.
  last_lat_accel_ = (R_ * accel).y();
  return apply(gyro);
}

}  // namespace adas
