#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "utils/adas_topics.h"
#include "utils/math_utils.h"

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
  int orient_samples() const { return static_cast<int>(accel_buf_.size()); }
  int bias_samples() const { return static_cast<int>(gyro_buf_.size()); }

  static Mat3 rotationFromGravity(const Vec3& accel);
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

  std::vector<Vec3> accel_buf_;
  std::vector<Vec3> gyro_buf_;

  double gyro_quiet_max_ = 0.08;
  double accel_g_err_max_ = 1.5;
  double bias_ema_alpha_ = 0.02;
};

}  // namespace adas
