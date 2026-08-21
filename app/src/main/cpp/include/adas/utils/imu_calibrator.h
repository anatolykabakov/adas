#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "adas/utils/adas_topics.h"
#include "adas/utils/math_utils.h"

namespace adas {
/** Finds which way the phone is mounted from gravity and rotation. */
class ImuCalibrator {
public:
  /// \param[in] speed_threshold_mps Below this the car counts as stationary for bias collection.
  explicit ImuCalibrator(double speed_threshold_mps = 0.5 / 3.6, int min_samples = 50, int max_buffer = 400,
                         bool invert_yaw_rate = true);

  /// Forget bias, orientation and buffers.
  void reset();
  /// Feed the current speed [m/s]; gates stationary-only collection.
  void setSpeed(double speed_mps);

  /// Seed the mount orientation from the config [deg].
  void setMountPrior(double roll_deg, double pitch_deg, double yaw_deg);

  /// One raw IMU sample in. \return Vehicle-frame yaw rate [rad/s] once calibrated, nothing before.
  std::optional<double> push(const RawImuSample& raw);

  /// \return True when a mount prior was set.
  bool hasPrior() const { return has_prior_; }
  /// \return True once gravity fixed the orientation.
  bool orientationLocked() const { return orientation_locked_; }
  /// \return True when samples can be resolved into the vehicle frame.
  bool ready() const { return has_prior_ || orientation_locked_; }
  /// \return Gyro bias estimate [rad/s].
  const Vec3& bias() const { return bias_; }
  /// \return Phone-to-vehicle rotation.
  const Mat3& rotation() const { return R_; }

  /** Lateral specific force in the vehicle frame (x forward, y right, z down) for the last sample, i.e.
   *  the accelerometer's y component after calibration. This is what `RoadRollEstimator` needs; it is only
   *  meaningful once a heading exists, which is what `hasHeading` reports. */
  double lastLatAccel() const { return last_lat_accel_; }
  /// \return True when yaw about gravity is known (prior only).
  bool hasHeading() const { return has_prior_; }
  /// \return Accelerometer samples buffered for orientation.
  int orient_samples() const { return static_cast<int>(accel_buf_.size()); }
  /// \return Gyro samples buffered for bias.
  int bias_samples() const { return static_cast<int>(gyro_buf_.size()); }

  /// \return Rotation aligning \p accel with gravity; yaw about gravity stays free.
  static Mat3 rotationFromGravity(const Vec3& accel);

  /** Roll and pitch from gravity, heading kept from `heading_ref`. */
  static Mat3 rotationFromGravityKeepingHeading(const Vec3& accel, const Mat3& heading_ref);
  /// Component overload.
  static Mat3 rotationFromGravity(double ax, double ay, double az) { return rotationFromGravity(Vec3(ax, ay, az)); }
  /// \return Rotation from mount roll/pitch/yaw [deg].
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
