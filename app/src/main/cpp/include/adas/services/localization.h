#pragma once

#include <optional>
#include <tuple>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/imu_calibrator.h"
#include "adas/utils/gps_local_projector.h"
#include "adas/utils/proto_convert.h"
#include "adas/utils/math_utils.h"
#include "adas/utils/online_localizer.h"
#include "adas/utils/params_learner.h"
#include "adas/utils/road_roll_estimator.h"

namespace adas {
namespace services {
/**
 * \brief Fuses wheel speed, yaw rate and GNSS into one pose.
 *
 * \details The estimate is an EKF in a local ENU plane anchored at the first fix. Wheel speed and yaw
 * rate carry it between fixes; GNSS bounds the drift. Each measurement is a separate switch in
 * `Config::Sources`, because a fused pose that looks healthy says nothing about which sensor is holding
 * it up, and turning them off one at a time is the only cheap way to find out.
 *
 * The service also runs the road-bank estimator and the vehicle-parameter learner: both need exactly the
 * inputs that arrive on the chassis tick, and both publish through the pose message.
 */
class Localization : public adas::middleware::Service {
public:
  struct Config {
    double wheelbase_m = 2.636;  ///< Wheelbase [m], the bicycle model's only geometry.
    double gps_noise_pos = 0.5;  ///< Assumed position noise [m] when the receiver reports no accuracy of its own.
    /// Minimum interval between position updates [s]; fixes arriving faster are dropped.
    double gps_update_interval = 0.2;
    /// A fix older than this is not used [us]: at 25 m/s, 2.5 s is 62 m of stale position.
    int64_t gps_max_age_us = 2'500'000;

    double gps_max_accuracy_m = 25.0;  ///< Fixes reporting worse accuracy are not used at all [m]; 0 disables the gate.

    /** Which measurements the filter is allowed to use.
     *
     *  Each source is a separate switch on purpose. A fused estimate that looks healthy tells you
     *  nothing about which sensor is carrying it — and when several sources agree, a broken one hides
     *  behind the others. Turning them off one at a time is the only cheap way to find out what the
     *  filter would do without each, and it is how the yaw-rate defect was finally quantified: the
     *  heading error was invisible with GPS on, and 38° in five seconds with it off.
     *
     *  This is also the knob a student needs. "Disable GPS and watch the heading drift" is an
     *  experiment, not a thought experiment, once it is a line of config.
     *
     *  Defaults keep everything on, i.e. the behaviour before these flags existed. */
    struct Sources {
      bool camera_odometry = true;  ///< Use the camera-odometry yaw rate as a measurement.
    } sources{};

    RoadRollEstimator::Config road_roll{};  ///< Road-bank estimator settings.

    ParamsLearner::Config params{};  ///< Vehicle-parameter learner settings.

    /// Below this the IMU calibrator ignores samples [km/h]: gravity cannot resolve heading.
    double imu_speed_threshold_kmh = 0.5;
    int imu_min_samples = 50;          ///< Samples needed before the mount estimate counts as usable.
    bool imu_invert_yaw_rate = true;   ///< The phone gyro's sign is the opposite of the car's on this mount.
    double imu_mount_roll_deg = 0.0;   ///< Mount prior, roll [deg]; used only with `imu_has_mount_prior`.
    double imu_mount_pitch_deg = 0.0;  ///< Mount prior, pitch [deg].
    double imu_mount_yaw_deg = 0.0;    ///< Mount prior, yaw [deg].
    bool imu_has_mount_prior = false;  ///< Trust the mount prior above instead of estimating from scratch.

    double steer_ratio = 15.7;  ///< Steering ratio used to turn the wheel angle into a road-wheel angle.
  };

  Localization() : Localization(Config{}) {}
  explicit Localization(Config config);

  std::string_view getName() const override { return "localization"; }

  void configure() override;
  void registerParameters();
  void reset() override;

  void resetPose(double x, double y, double yaw, double v = 0, double yaw_rate = 0);

  std::tuple<double, double, double> step(double dt, double speed_mps, double steer_rad,
                                          std::optional<double> yaw_rate = std::nullopt,
                                          std::optional<double> gps_x = std::nullopt,
                                          std::optional<double> gps_y = std::nullopt,
                                          std::optional<double> ref_x = std::nullopt,
                                          std::optional<double> ref_y = std::nullopt);

  OnlineLocalizer& localizer() { return loc_; }
  const OnlineLocalizer& localizer() const { return loc_; }
  const LocalizationPose& lastPose() const { return last_pose_; }
  const Config& config() const { return config_; }

private:
  void onChassis(const ChassisSample& msg);
  /// Yaw rate to feed the filter: the phone gyro when it is valid, otherwise the ESP sensor from CAN.
  std::optional<double> yawRateMeasurement(const ChassisSample& msg) const;
  /// The last fix, if it is close enough in time to this chassis frame to describe the same instant.
  std::optional<GpsSample> freshGps(int64_t chassis_ts_us) const;
  /// Camera-odometry yaw rate, if the source is enabled and the model is confident in it.
  std::optional<double> camYawRate() const;
  void onGpsProto(const adas::proto::GPSLocation& gps);
  void onCameraOdometryProto(const adas::proto::CameraOdometry& odom);
  void onGps(const GpsSample& msg);
  void onRawImu(const adas::proto::IMUData& imu);
  void onCameraOdometry(const CameraOdometrySample& msg);
  void publishPose(int64_t timestamp_us);

  Config config_;
  OnlineLocalizer loc_;
  RoadRollEstimator road_roll_{};
  ParamsLearner params_{};
  ImuCalibrator imu_calib_;
  bool imu_lock_logged_ = false;
  GpsLocalProjector gps_proj_{};
  int64_t last_imu_us_ = 0;
  ChassisSample chassis_;
  GpsSample gps_;
  ImuSample imu_;
  CameraOdometrySample cam_odo_;
  bool have_chassis_ = false;
  bool have_cam_odo_ = false;
  int64_t last_t_us_ = 0;
  int64_t last_gps_log_us_ = 0;
  LocalizationPose last_pose_;
};

}  // namespace services

}  // namespace adas
