#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "adas/utils/adas_topics.h"
#include "adas/utils/math_utils.h"

namespace adas {

class ImuCalibrator {
public:
  explicit ImuCalibrator(double speed_threshold_mps = 0.5 / 3.6, int min_samples = 50, int max_buffer = 400,
                         bool invert_yaw_rate = true);

  void reset();
  void setSpeed(double speed_mps);

  void setMountPrior(double roll_deg, double pitch_deg, double yaw_deg);

  std::optional<double> push(const RawImuSample& raw);

  bool hasPrior() const { return has_prior_; }
  bool orientationLocked() const { return orientation_locked_; }
  bool ready() const { return has_prior_ || orientation_locked_; }
  const Vec3& bias() const { return bias_; }
  const Mat3& rotation() const { return R_; }

  /** Lateral specific force in the vehicle frame (x forward, y right, z down) for the last sample, i.e.
   *  the accelerometer's y component after calibration. This is what `RoadRollEstimator` needs; it is only
   *  meaningful once a heading exists, which is what `hasHeading` reports. */
  double lastLatAccel() const { return last_lat_accel_; }
  bool hasHeading() const { return has_prior_; }
  int orient_samples() const { return static_cast<int>(accel_buf_.size()); }
  int bias_samples() const { return static_cast<int>(gyro_buf_.size()); }

  static Mat3 rotationFromGravity(const Vec3& accel);

  /** Roll and pitch from gravity, heading kept from `heading_ref`.
   *
   *  Gravity fixes two of the three angles and says nothing about the third: any rotation about the
   *  gravity axis leaves the measured vector unchanged. `rotationFromGravity` resolves that freedom by
   *  taking the minimal rotation, which is fine for yaw rate — only the z component is read, and gravity
   *  fixes z — and wrong for anything lateral, because the resulting y axis points nowhere in particular.
   *
   *  So the standstill lock keeps the heading it already had (from the mount prior) and replaces only what
   *  gravity actually measured. Without this the lock silently destroys the heading, and the lateral
   *  specific force published alongside the yaw rate would be a mixture of longitudinal and lateral. */
  static Mat3 rotationFromGravityKeepingHeading(const Vec3& accel, const Mat3& heading_ref);
  static Mat3 rotationFromGravity(double ax, double ay, double az) { return rotationFromGravity(Vec3(ax, ay, az)); }
  static Mat3 rotationFromMountRpy(double roll_deg, double pitch_deg, double yaw_deg);

private:
  bool isQuiet(const RawImuSample& raw) const;
  void tryLockOrientation();
  void updateBiasEma(const Vec3& gyro);
  double apply(const Vec3& gyro) const;

  double speed_threshold_mps_;
  int min_samples_;
  int max_buffer_;
  bool invert_yaw_rate_;

  double speed_mps_ = 0.0;
  bool has_prior_ = false;
  bool orientation_locked_ = false;
  Vec3 bias_ = Vec3::Zero();
  Mat3 R_ = Mat3::Identity();

  double last_lat_accel_ = 0.0;
  std::vector<Vec3> accel_buf_;
  std::vector<Vec3> gyro_buf_;

  double gyro_quiet_max_ = 0.08;
  double accel_g_err_max_ = 1.5;
  double bias_ema_alpha_ = 0.02;
};

}  // namespace adas
