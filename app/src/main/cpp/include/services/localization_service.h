#pragma once

#include <optional>
#include <tuple>

#include "middleware/middleware.hpp"
#include "utils/adas_topics.h"
#include "utils/math_utils.h"
#include "utils/online_localizer.h"

namespace adas {

class LocalizationService : public adas::Service {
public:
  struct Config {
    double wheelbase_m = 2.636;
    double gps_noise_pos = 0.5;
    double gps_update_interval = 0.2;
    bool use_cam_odo = true;
    bool invert_cam_yaw_rate = false;
    int64_t gps_max_age_us = 2'500'000;
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
