#include "adas/services/localization.h"

#include "messages.pb.h"
#include "adas/utils/logger.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
Localization::Localization(Config config)
  : config_(config), loc_(config.wheelbase_m, config.gps_noise_pos, config.gps_update_interval, true)
{
  imu_calib_ = ImuCalibrator(config.imu_speed_threshold_kmh / 3.6, config.imu_min_samples,
                             std::max(400, config.imu_min_samples * 4), config.imu_invert_yaw_rate);
  if (config.imu_has_mount_prior)
    imu_calib_.setMountPrior(config.imu_mount_roll_deg, config.imu_mount_pitch_deg, config.imu_mount_yaw_deg);
  road_roll_.setConfig(config.road_roll);
  params_.setConfig(config.params);
  loc_.max_gps_accuracy_m = config.gps_max_accuracy_m;
  loc_.ekf().setYawRateIsAState(true);
}

void Localization::registerParameters()
{
  registerParameter<double>(
      "gps_max_accuracy_m",
      [this](const double& v) {
        config_.gps_max_accuracy_m = v;
        loc_.max_gps_accuracy_m = v;
      },
      [this] { return config_.gps_max_accuracy_m; });

  const auto learner = [this](const char* name, double& field) {
    registerParameter<double>(
        name,
        [this, &field](const double& v) {
          field = v;
          params_.setConfig(config_.params);
        },
        [&field] { return field; });
  };
  auto& pl = config_.params;
  learner("params_stiffness_init", pl.stiffness_init);
  learner("params_steer_ratio_init", pl.steer_ratio_init);
  learner("params_angle_offset_init_deg", pl.angle_offset_init_deg);
  learner("params_stiffness_p0_std", pl.stiffness_p0_std);
  learner("params_stiffness_process_std", pl.stiffness_process_std);
  learner("params_steer_ratio_process_std", pl.steer_ratio_process_std);
  learner("params_min_speed_ms", pl.min_speed_ms);
  learner("params_max_lateral_jerk", pl.max_lateral_jerk);
  learner("params_max_roll_std_deg", pl.max_roll_std_deg);
  registerParameter<bool>(
      "params_use_roll",
      [this](const bool& v) {
        config_.params.use_roll = v;
        params_.setConfig(config_.params);
      },
      [this] { return config_.params.use_roll; });
}

void Localization::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, config_.steer_ratio));
  });
  subscribe<adas::proto::GPSLocation>(topics::kGpsLocation,
                                      [this](const adas::proto::GPSLocation& payload) { onGpsProto(payload); });
  subscribe<adas::proto::CameraOdometry>(
      topics::kCameraOdometry, [this](const adas::proto::CameraOdometry& payload) { onCameraOdometryProto(payload); });
  registerParameters();
  subscribe<adas::proto::IMUData>(topics::kImu, [this](const adas::proto::IMUData& payload) { onRawImu(payload); });
  LOGI("Localization → %s  camera odometry %s", topics::kLocalizationPose,
       config_.sources.camera_odometry ? "on" : "off");
}

void Localization::reset()
{
  loc_.reset();
  have_chassis_ = false;
  have_cam_odo_ = false;
  last_t_us_ = 0;
  last_gps_log_us_ = 0;
  last_pose_ = LocalizationPose{};
  gps_ = GpsSample{};
  imu_ = ImuSample{};
  road_roll_.reset();
  params_.reset();
  last_imu_us_ = 0;
  cam_odo_ = CameraOdometrySample{};
}

void Localization::resetPose(double x, double y, double yaw, double v, double yaw_rate)
{
  loc_.reset(x, y, yaw, v, yaw_rate);
  last_pose_.x = x;
  last_pose_.y = y;
  last_pose_.yaw = yaw;
  last_pose_.v = v;
  last_pose_.yaw_rate = yaw_rate;
}

void Localization::onGpsProto(const adas::proto::GPSLocation& g)
{
  const bool ok_fix =
      g.fix_type() != adas::proto::GPSLocation::NO_FIX && g.fix_type() != adas::proto::GPSLocation::TIME_ONLY;
  GpsSample sample = gps_proj_.project(static_cast<int64_t>(g.timestamp()) * 1000, g.latitude(), g.longitude(), ok_fix,
                                       g.speed(), g.bearing());
  if (!sample.valid)
    return;
  sample.accuracy_m = g.horizontal_accuracy();
  sample.satellites = g.satellites_used();
  onGps(sample);
}

void Localization::onCameraOdometryProto(const adas::proto::CameraOdometry& odom)
{
  const auto sample = cameraOdometryToSample(odom);
  if (sample.valid)
    onCameraOdometry(sample);
}

void Localization::onGps(const GpsSample& msg)
{
  gps_ = msg;
  if (msg.valid && msg.timestamp_us - last_gps_log_us_ > 5'000'000) {
    last_gps_log_us_ = msg.timestamp_us;
    LOGI("Localization GPS enu=(%.1f,%.1f) course=%d yaw=%.2f v=%.1f gps_upd=%d rej=%d reseed=%d imu=%d", msg.x, msg.y,
         msg.course_valid ? 1 : 0, msg.yaw_enu, msg.speed_mps, loc_.ekf().gps_update_count,
         loc_.ekf().gps_rejected_count, loc_.ekf().gps_reseed_count, loc_.ekf().imu_update_count);
  }
}

void Localization::onRawImu(const adas::proto::IMUData& payload)
{
  const RawImuSample raw = imuToRaw(payload);
  if (have_chassis_)
    imu_calib_.setSpeed(chassis_.speed_mps);
  const auto yaw_rate = imu_calib_.push(raw);
  if (!yaw_rate)
    return;

  if (!imu_calib_.orientationLocked())
    return;
  if (!imu_lock_logged_) {
    LOGI("Localization: IMU orientation locked (bias gz=%.5f)", imu_calib_.bias().z());
    imu_lock_logged_ = true;
  }

  ImuSample msg;
  msg.timestamp_us = raw.timestamp_us;
  msg.yaw_rate = *yaw_rate;
  msg.lat_accel = imu_calib_.lastLatAccel();
  msg.lat_accel_valid = imu_calib_.hasHeading();
  msg.valid = true;
  imu_ = msg;

  const double dt =
      last_imu_us_ > 0 && msg.timestamp_us > last_imu_us_ ? (msg.timestamp_us - last_imu_us_) * 1e-6 : 0.0;
  last_imu_us_ = msg.timestamp_us;
  if (msg.lat_accel_valid && have_chassis_)
    road_roll_.update(chassis_.speed_mps, chassis_.yaw_rate, msg.lat_accel, dt);
}

void Localization::onCameraOdometry(const CameraOdometrySample& msg)
{
  cam_odo_ = msg;
  have_cam_odo_ = msg.valid;
}

void Localization::onChassis(const ChassisSample& msg)
{
  double dt = 0.05;
  if (last_t_us_ > 0 && msg.timestamp_us > last_t_us_) {
    dt = (msg.timestamp_us - last_t_us_) * 1e-6;
  }
  if (dt > 0.2)
    dt = 0.05;
  last_t_us_ = msg.timestamp_us;
  chassis_ = msg;
  have_chassis_ = true;

  loc_.step(dt, msg.speed_mps, msg.steer_rad, yawRateMeasurement(msg), freshGps(msg.timestamp_us), std::nullopt,
            camYawRate());

  // The learner rides the chassis tick because every one of its inputs is on this message. It is fed the
  // raw CAN yaw rate rather than `loc_.ekf().yawRate()` on purpose: the EKF's yaw rate is partly produced
  // by the bicycle model, whose stiffness and steering ratio are exactly what the learner estimates, so
  // closing that loop would have it fit its own output.
  params_.update(msg.speed_mps, msg.steering_angle_deg, msg.yaw_rate, road_roll_.rollDeg(), road_roll_.rollStdDeg(),
                 dt);
  publishPose(msg.timestamp_us);
}

std::optional<double> Localization::yawRateMeasurement(const ChassisSample& msg) const
{
  if (imu_.valid)
    return imu_.yaw_rate;
  if (std::abs(msg.yaw_rate) > 1e-9)
    return msg.yaw_rate;
  return std::nullopt;
}

std::optional<GpsSample> Localization::freshGps(int64_t chassis_ts_us) const
{
  if (!gps_.valid)
    return std::nullopt;
  // A fix stamped in the future by up to half a second is still the same instant: the two clocks are the
  // phone's and the receiver's, and the offset between them is not zero.
  const int64_t age = chassis_ts_us - gps_.timestamp_us;
  if (gps_.timestamp_us <= 0 || (age >= -500'000 && age <= config_.gps_max_age_us))
    return gps_;
  return std::nullopt;
}

std::optional<double> Localization::camYawRate() const
{
  if (!config_.sources.camera_odometry || !have_cam_odo_ || !cam_odo_.valid)
    return std::nullopt;
  if (cam_odo_.rot_std[2] >= 0.5)
    return std::nullopt;
  return cam_odo_.rot[2];
}

void Localization::publishPose(int64_t timestamp_us)
{
  LocalizationPose pose;
  pose.timestamp_us = timestamp_us;
  pose.x = loc_.x();
  pose.y = loc_.y();
  pose.yaw = loc_.yaw();
  pose.v = loc_.ekf().v();
  pose.yaw_rate = loc_.ekf().yawRate();
  pose.road_roll_deg = road_roll_.rollDeg();
  pose.road_roll_std_deg = road_roll_.rollStdDeg();
  pose.road_roll_valid = road_roll_.valid();
  pose.learned_stiffness_factor = params_.stiffnessFactor();
  pose.learned_steer_ratio = params_.steerRatio();
  pose.learned_angle_offset_deg = params_.angleOffsetDeg();
  pose.learned_stiffness_std = params_.stiffnessStd();
  pose.learned_steer_ratio_std = params_.steerRatioStd();
  pose.learned_params_valid = params_.valid();
  pose.learned_sample_count = params_.sampleCount();
  pose.odom_x = loc_.odomX().empty() ? 0.0 : loc_.odomX().back();
  pose.odom_y = loc_.odomY().empty() ? 0.0 : loc_.odomY().back();
  pose.ekf_x = loc_.ekfX().empty() ? pose.x : loc_.ekfX().back();
  pose.ekf_y = loc_.ekfY().empty() ? pose.y : loc_.ekfY().back();
  last_pose_ = pose;

  publish(topics::kLocalizationPose, createLocalizationPose(pose, timestamp_us));
}

std::tuple<double, double, double> Localization::step(double dt, double speed_mps, double steer_rad,
                                                      std::optional<double> yaw_rate, std::optional<double> gps_x,
                                                      std::optional<double> gps_y, std::optional<double> ref_x,
                                                      std::optional<double> ref_y)
{
  std::optional<GpsSample> gps;
  if (gps_x && gps_y) {
    GpsSample g;
    g.x = *gps_x;
    g.y = *gps_y;
    g.valid = true;
    gps = g;
  }
  std::optional<Vec2> ref;
  if (ref_x && ref_y)
    ref = Vec2{*ref_x, *ref_y};
  return loc_.step(dt, speed_mps, steer_rad, yaw_rate, gps, ref, std::nullopt);
}

}  // namespace services
}  // namespace adas
