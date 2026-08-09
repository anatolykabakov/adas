#include "services/localization_service.h"

#include "messages.pb.h"
#include "utils/logger.h"

namespace adas {

LocalizationService::LocalizationService(Config config)
  : config_(config), loc_(config.wheelbase_m, config.gps_noise_pos, config.gps_update_interval, true)
{
  loc_.invert_cam_yaw_rate = config.invert_cam_yaw_rate;
  road_roll_.setConfig(config.road_roll);
  params_.setConfig(config.params);
  loc_.use_gps_position = config.sources.gps_position;
  loc_.use_gps_course = config.sources.gps_course;
  loc_.use_gps_velocity = config.sources.gps_velocity;
  // The bicycle model is a measurement now, not the source of truth — see `VehicleEKF::setYawRateIsAState`.
  loc_.ekf().setYawRateIsAState(true);
  if (!config.sources.bicycle_model) {
    // Push its noise far out rather than adding a branch in the hot path: an observation with an
    // enormous R changes nothing, and the state still exists for the gyro to correct.
    loc_.ekf().setYawRateIsAState(true, 1e3);
  }
}

void LocalizationService::configure()
{
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<GpsSample>(topics::kGpsLocation, [this](const GpsSample& m) { onGps(m); });
  subscribe<ImuSample>(topics::kImuYaw, [this](const ImuSample& m) { onImu(m); });
  if (config_.sources.camera_odometry) {
    subscribe<CameraOdometrySample>(topics::kCameraOdometry,
                                    [this](const CameraOdometrySample& m) { onCameraOdometry(m); });
  }
  const auto& src = config_.sources;
  LOGI("LocalizationService → %s  sources: gps[pos=%d course=%d vel=%d] imu=%d chassis_yaw=%d cam_odo=%d "
       "bicycle=%d learn_params=%d",
       topics::kLocalizationPose, src.gps_position, src.gps_course, src.gps_velocity, src.imu_yaw_rate,
       src.chassis_yaw_rate, src.camera_odometry, src.bicycle_model, config_.learn_vehicle_params);
}

void LocalizationService::reset()
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

void LocalizationService::resetPose(double x, double y, double yaw, double v, double yaw_rate)
{
  loc_.reset(x, y, yaw, v, yaw_rate);
  last_pose_.x = x;
  last_pose_.y = y;
  last_pose_.yaw = yaw;
  last_pose_.v = v;
  last_pose_.yaw_rate = yaw_rate;
}

void LocalizationService::onGps(const GpsSample& msg)
{
  gps_ = msg;
  if (msg.valid && msg.timestamp_us - last_gps_log_us_ > 5'000'000) {
    last_gps_log_us_ = msg.timestamp_us;
    LOGI("Localization GPS enu=(%.1f,%.1f) course=%d yaw=%.2f v=%.1f gps_upd=%d rej=%d reseed=%d imu=%d", msg.x, msg.y,
         msg.course_valid ? 1 : 0, msg.yaw_enu, msg.speed_mps, loc_.ekf().gps_update_count,
         loc_.ekf().gps_rejected_count, loc_.ekf().gps_reseed_count, loc_.ekf().imu_update_count);
  }
}

void LocalizationService::onImu(const ImuSample& msg)
{
  imu_ = msg;

  // Road bank rides on the IMU stream rather than the chassis tick: it needs the lateral specific force,
  // which only this message carries, and its own 10 s time constant makes the exact cadence irrelevant.
  const double dt =
      last_imu_us_ > 0 && msg.timestamp_us > last_imu_us_ ? (msg.timestamp_us - last_imu_us_) * 1e-6 : 0.0;
  last_imu_us_ = msg.timestamp_us;
  if (msg.lat_accel_valid && have_chassis_)
    road_roll_.update(chassis_.speed_mps, chassis_.yaw_rate, msg.lat_accel, dt);
}

void LocalizationService::onCameraOdometry(const CameraOdometrySample& msg)
{
  cam_odo_ = msg;
  have_cam_odo_ = msg.valid;
}

void LocalizationService::onChassis(const ChassisSample& msg)
{
  double dt = 0.05;
  if (last_t_us_ > 0 && msg.timestamp_us > last_t_us_) {
    dt = (msg.timestamp_us - last_t_us_) * 1e-6;
  }
  // Clamp absurd dt (USB stalls / clock glitches) so one tick cannot teleport.
  if (dt > 0.2)
    dt = 0.05;
  last_t_us_ = msg.timestamp_us;
  chassis_ = msg;
  have_chassis_ = true;

  // Yaw rate: phone gyro first, then the ESP sensor from CAN. Each is switchable so that a run can
  // answer "what would the heading do without this one" instead of leaving it to argument.
  std::optional<double> yr;
  if (config_.sources.imu_yaw_rate && imu_.valid) {
    yr = imu_.yaw_rate;
  } else if (config_.sources.chassis_yaw_rate && std::abs(msg.yaw_rate) > 1e-9) {
    yr = msg.yaw_rate;
  }

  // Only fuse fresh GPS — stale first-fix at ENU (0,0) used to pin EKF at origin.
  std::optional<GpsSample> gps;
  if (gps_.valid) {
    const int64_t age = msg.timestamp_us - gps_.timestamp_us;
    if (gps_.timestamp_us <= 0 || (age >= -500'000 && age <= config_.gps_max_age_us))
      gps = gps_;
  }

  std::optional<double> cam_w;
  if (config_.sources.camera_odometry && have_cam_odo_ && cam_odo_.valid) {
    const double rstd = cam_odo_.rot_std[2];
    if (rstd < 0.5)
      cam_w = cam_odo_.rot[2];
  }

  loc_.step(dt, msg.speed_mps, msg.steer_rad, yr, gps, std::nullopt, cam_w);

  // The learner rides the chassis tick because every one of its inputs is on this message. It is fed the
  // raw CAN yaw rate rather than `loc_.ekf().yawRate()` on purpose: the EKF's yaw rate is partly produced by
  // the very bicycle model whose parameters are being learned, so using it would close a loop and the filter
  // would happily confirm whatever it already believed.
  if (config_.learn_vehicle_params) {
    params_.update(msg.speed_mps, msg.steering_angle_deg, msg.yaw_rate, road_roll_.rollDeg(), road_roll_.rollStdDeg(),
                   dt);
  }
  publishPose(msg.timestamp_us);
}

void LocalizationService::publishPose(int64_t timestamp_us)
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
  pose.learned_params_valid = config_.learn_vehicle_params && params_.valid();
  pose.learned_sample_count = params_.sampleCount();
  pose.odom_x = loc_.odomX().empty() ? 0.0 : loc_.odomX().back();
  pose.odom_y = loc_.odomY().empty() ? 0.0 : loc_.odomY().back();
  pose.ekf_x = loc_.ekfX().empty() ? pose.x : loc_.ekfX().back();
  pose.ekf_y = loc_.ekfY().empty() ? pose.y : loc_.ekfY().back();
  last_pose_ = pose;

  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(timestamp_us / 1000);
  zmq.set_topic(topics::kLocalizationPose);
  auto* p = zmq.mutable_localization_pose();
  p->set_timestamp(timestamp_us / 1000);
  p->set_x(pose.x);
  p->set_y(pose.y);
  p->set_yaw(pose.yaw);
  p->set_road_roll_deg(pose.road_roll_deg);
  p->set_road_roll_std_deg(pose.road_roll_std_deg);
  p->set_road_roll_valid(pose.road_roll_valid);
  p->set_learned_stiffness_factor(pose.learned_stiffness_factor);
  p->set_learned_steer_ratio(pose.learned_steer_ratio);
  p->set_learned_angle_offset_deg(pose.learned_angle_offset_deg);
  p->set_learned_stiffness_std(pose.learned_stiffness_std);
  p->set_learned_steer_ratio_std(pose.learned_steer_ratio_std);
  p->set_learned_params_valid(pose.learned_params_valid);
  p->set_learned_sample_count(pose.learned_sample_count);
  p->set_v(pose.v);
  p->set_yaw_rate(pose.yaw_rate);
  p->set_odom_x(pose.odom_x);
  p->set_odom_y(pose.odom_y);
  p->set_ekf_x(pose.ekf_x);
  p->set_ekf_y(pose.ekf_y);
  publish(topics::kLocalizationPose, zmq);
}

std::tuple<double, double, double> LocalizationService::step(double dt, double speed_mps, double steer_rad,
                                                             std::optional<double> yaw_rate,
                                                             std::optional<double> gps_x, std::optional<double> gps_y,
                                                             std::optional<double> ref_x, std::optional<double> ref_y)
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

}  // namespace adas
