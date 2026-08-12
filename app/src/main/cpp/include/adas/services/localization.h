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
class Localization : public adas::middleware::Service {
public:
  struct Config {
    double wheelbase_m = 2.636;
    double gps_noise_pos = 0.5;
    double gps_update_interval = 0.2;
    bool invert_cam_yaw_rate = false;
    int64_t gps_max_age_us = 2'500'000;

    double gps_max_accuracy_m = 25.0;

    bool gps_scale_noise_by_accuracy = true;

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
      bool gps_position = true;
      bool gps_course = true;
      bool gps_velocity = true;
      bool imu_yaw_rate = true;
      bool chassis_yaw_rate = true;
      bool camera_odometry = true;
      bool bicycle_model = true;
    } sources{};

    RoadRollEstimator::Config road_roll{};

    bool learn_vehicle_params = false;
    ParamsLearner::Config params{};

    double imu_speed_threshold_kmh = 0.5;
    int imu_min_samples = 50;
    bool imu_invert_yaw_rate = true;
    double imu_mount_roll_deg = 0.0;
    double imu_mount_pitch_deg = 0.0;
    double imu_mount_yaw_deg = 0.0;
    bool imu_has_mount_prior = false;

    double steer_ratio = 15.7;
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
