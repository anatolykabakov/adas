#pragma once

#include <optional>

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/utils/imu_calibrator.h"

namespace adas {

namespace services {

class ImuCalib : public adas::middleware::Service {
public:
  struct Config {
    double speed_threshold_kmh = 0.5;
    int min_samples = 50;
    bool invert_yaw_rate = true;
    double mount_roll_deg = 0.0;
    double mount_pitch_deg = 0.0;
    double mount_yaw_deg = 0.0;
    bool has_mount_prior = false;
  };

  ImuCalib() : ImuCalib(Config{}) {}
  explicit ImuCalib(Config config);

  void configure() override;
  void reset() override;

  void setMountPrior(double roll_deg, double pitch_deg, double yaw_deg);

  ImuCalibrator& calibrator() { return calib_; }
  const ImuCalibrator& calibrator() const { return calib_; }
  const ImuSample& last() const { return last_; }
  const Config& config() const { return config_; }

  std::optional<double> push(const RawImuSample& raw, double speed_mps);

private:
  void onChassis(const ChassisSample& msg);
  void onRawImu(const RawImuSample& msg);
  void publishYaw(int64_t timestamp_us, double yaw_rate);

  Config config_;
  ImuCalibrator calib_;
  double speed_threshold_kmh_ = 0.5;
  double prior_roll_ = 0, prior_pitch_ = 0.0, prior_yaw_ = 0;
  bool have_prior_angles_ = false;
  ChassisSample chassis_;
  bool have_chassis_ = false;
  ImuSample last_;
  bool logged_lock_ = false;
};

}  // namespace services

}  // namespace adas
