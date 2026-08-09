#include "services/imu_calib_service.h"

#include <algorithm>

#include "utils/logger.h"

namespace adas {

ImuCalibService::ImuCalibService(Config config)
  : config_(config)
  , calib_(config.speed_threshold_kmh / 3.6, config.min_samples, std::max(400, config.min_samples * 4),
           config.invert_yaw_rate)
  , speed_threshold_kmh_(config.speed_threshold_kmh)
{
  if (config.has_mount_prior)
    setMountPrior(config.mount_roll_deg, config.mount_pitch_deg, config.mount_yaw_deg);
}

void ImuCalibService::configure()
{
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<RawImuSample>(topics::kImuRaw, [this](const RawImuSample& m) { onRawImu(m); });
  LOGI("ImuCalibService: %s + %s → %s (prior=%d lock_standstill_<%.2fkm/h)", topics::kVehicleChassis, topics::kImuRaw,
       topics::kImuYaw, calib_.hasPrior() ? 1 : 0, speed_threshold_kmh_);
}

void ImuCalibService::reset()
{
  const bool had = calib_.hasPrior();
  const auto R = calib_.rotation();
  calib_.reset();
  if (had) {
    calib_.setMountPrior(0, 0, 0);

    (void)R;
  }
  have_chassis_ = false;
  last_ = ImuSample{};
  logged_lock_ = false;
}

void ImuCalibService::setMountPrior(double roll_deg, double pitch_deg, double yaw_deg)
{
  calib_.setMountPrior(roll_deg, pitch_deg, yaw_deg);
  LOGI("ImuCalibService: mount prior roll=%.1f pitch=%.1f yaw=%.1f", roll_deg, pitch_deg, yaw_deg);
}

void ImuCalibService::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
  calib_.setSpeed(msg.speed_mps);
}

void ImuCalibService::onRawImu(const RawImuSample& msg)
{
  if (have_chassis_)
    calib_.setSpeed(chassis_.speed_mps);
  auto yr = calib_.push(msg);
  if (!yr)
    return;
  if (calib_.orientationLocked() && !logged_lock_) {
    LOGI("ImuCalibService: orientation locked (bias gz=%.5f)", calib_.bias().z());
    logged_lock_ = true;
  }
  publishYaw(msg.timestamp_us, *yr);
}

void ImuCalibService::publishYaw(int64_t timestamp_us, double yaw_rate)
{
  last_.timestamp_us = timestamp_us;
  last_.yaw_rate = yaw_rate;
  last_.lat_accel = calib_.lastLatAccel();
  last_.lat_accel_valid = calib_.hasHeading();
  last_.valid = true;
  publish(topics::kImuYaw, last_);
}

std::optional<double> ImuCalibService::push(const RawImuSample& raw, double speed_mps)
{
  calib_.setSpeed(speed_mps);
  auto yr = calib_.push(raw);
  if (yr)
    publishYaw(raw.timestamp_us, *yr);
  return yr;
}

}  // namespace adas
