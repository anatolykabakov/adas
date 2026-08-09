#pragma once

#include <optional>
#include <tuple>

#include "middleware/middleware.hpp"
#include "utils/adas_topics.h"
#include "utils/math_utils.h"
#include "utils/online_localizer.h"
#include "utils/params_learner.h"
#include "utils/road_roll_estimator.h"

namespace adas {

class LocalizationService : public adas::Service {
public:
  struct Config {
    double wheelbase_m = 2.636;
    double gps_noise_pos = 0.5;
    double gps_update_interval = 0.2;
    /// Camera-odometry yaw rate is measured with the opposite sign to CAN on this car: correlation
    /// -0.994 over 39k ticks of 2026_08_08_23_00_28. Until that is understood the source is off by
    /// default (see localization.use_camera_odometry) — see docs/REVIEW_2026_08_09.md item 1.
    bool invert_cam_yaw_rate = false;
    int64_t gps_max_age_us = 2'500'000;

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
      /** GPS position updates (`updateGps`). Off: the pose is dead reckoning in the ENU plane it was
       *  seeded in, and position error grows without bound. */
      bool gps_position = true;
      /** GPS course → heading (`updateGpsYaw`). Off: heading comes from the yaw-rate chain alone.
       *  This is the switch that reveals whether the yaw-rate handling is sound. */
      bool gps_course = true;
      /** GPS velocity (`updateGpsVel`). It is called, and with speed now a state the correction survives
       *  the next tick — but the wheel speed arrives twenty times more often at the same assumed noise and
       *  drags the estimate back, so the 1.2 % wheel scale is still not learnable. That needs the scale as
       *  a state; see `docs/BACKLOG.md` §5a. */
      bool gps_velocity = true;
      /** Phone gyro yaw rate from `sensors/imu_yaw`. Off: the chassis yaw rate from CAN is used, and
       *  if that is absent too, only the bicycle model remains. */
      bool imu_yaw_rate = true;
      /** Yaw rate from `vehicle/chassis` (the ESP sensor) as fallback when the phone gyro is absent.
       *  Measured against the phone gyro at a ratio of 1.017, so it is a good sensor — this flag is
       *  here to isolate it, not because it is suspect. */
      bool chassis_yaw_rate = true;
      /** Camera-odometry yaw rate from `model/camera_odometry`. Measured against the ESP sensor at
       *  0.849, i.e. the outlier of the three, consistent with the model's metric scale. */
      bool camera_odometry = true;
      /** Bicycle model as a weak yaw-rate measurement. Off: with no gyro and no chassis rate the
       *  heading stops moving, which is a legitimate thing to want to see. */
      bool bicycle_model = true;
    } sources{};


    /** Road bank from the lateral accelerometer — the input `paramsd` needs to tell a banked road from an
     *  understeering car. See `utils/road_roll_estimator.h` for the measurement it is built on. */
    RoadRollEstimator::Config road_roll{};

    /** Learn the vehicle parameters from steering angle, speed and yaw rate — the ported part of `paramsd`.
     *
     *  Default off, and it stays off until a drive says otherwise. The learner is cheap and its output is
     *  published either way, so the honest sequence is: enable it, drive, compare the learned stiffness
     *  against the configured 0.64 in the bag, and only then let the controller read it. Enabling the
     *  learner does not by itself change a single command — that needs `lane_keep.use_learned_params`. */
    bool learn_vehicle_params = false;
    ParamsLearner::Config params{};
  };

  LocalizationService() : LocalizationService(Config{}) {}
  explicit LocalizationService(Config config);

  std::string_view getName() const override { return "localization"; }

  void configure() override;
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
  void onGps(const GpsSample& msg);
  void onImu(const ImuSample& msg);
  void onCameraOdometry(const CameraOdometrySample& msg);
  void publishPose(int64_t timestamp_us);

  Config config_;
  OnlineLocalizer loc_;
  RoadRollEstimator road_roll_{};
  ParamsLearner params_{};
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

}  // namespace adas
