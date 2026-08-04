#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "utils/adas_topics.h"
#include "utils/math_utils.h"
#include "utils/vehicle_ekf.h"

namespace adas {

class OnlineLocalizer {
public:
  OnlineLocalizer(double wheelbase = 2.636, double gps_noise_pos = 0.5, double gps_update_interval = 0.2,
                  bool imu_every_step = true);

  void reset(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0);

  std::tuple<double, double, double> step(double dt, double speed_mps, double steer_rad,
                                          std::optional<double> yaw_rate = std::nullopt,
                                          std::optional<GpsSample> gps = std::nullopt,
                                          std::optional<Vec2> ref_xy = std::nullopt,
                                          std::optional<double> cam_yaw_rate = std::nullopt);

  double x() const { return ekf_.x(); }
  double y() const { return ekf_.y(); }
  double yaw() const { return ekf_.yaw(); }
  VehicleEKF& ekf() { return ekf_; }
  const VehicleEKF& ekf() const { return ekf_; }

  const std::vector<double>& refX() const { return ref_x_; }
  const std::vector<double>& refY() const { return ref_y_; }
  const std::vector<double>& odomX() const { return odom_traj_x_; }
  const std::vector<double>& odomY() const { return odom_traj_y_; }
  const std::vector<double>& ekfX() const { return ekf_traj_x_; }
  const std::vector<double>& ekfY() const { return ekf_traj_y_; }

  double imu_agree_rad_s = 0.35;
  double cam_odo_R = 0.05;
  bool invert_cam_yaw_rate = false;

private:
  void record(double rx, double ry, double ox, double oy, double ex, double ey);
  void snapYaw(double yaw_enu);

  VehicleEKF ekf_;
  double wheelbase_ = 2.636;
  double gps_update_interval_ = 0.2;
  bool imu_every_step_ = true;
  bool initialized_ = false;
  bool yaw_seeded_ = false;
  double odom_x_ = 0, odom_y_ = 0, odom_yaw_ = 0;
  double t_ = 0, last_gps_t_ = -1e9;
  int64_t last_gps_msg_us_ = -1;

  std::vector<double> ref_x_, ref_y_;
  std::vector<double> odom_traj_x_, odom_traj_y_;
  std::vector<double> ekf_traj_x_, ekf_traj_y_;
};

}  // namespace adas
