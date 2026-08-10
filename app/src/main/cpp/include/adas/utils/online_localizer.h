#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "adas/utils/adas_topics.h"
#include "adas/utils/math_utils.h"
#include "adas/utils/vehicle_ekf.h"

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

  /** Which GPS measurements are allowed through. Each one is separate because a fused estimate does not
   *  say which sensor carries it, and switching sources off one at a time is the only cheap way to find
   *  out what the filter would do without each. `Localization::Config::Sources` drives these. */
  double max_gps_speed_mismatch_mps = 12.0;

  int reseed_agree_count = 3;
  double reseed_agree_radius_m = 8.0;
  int gps_unphysical_count = 0;

  bool use_gps_position = true;
  bool use_gps_course = true;
  bool use_gps_velocity = true;

  /** Variance of the GPS velocity measurement.
   *
   *  It was 1.0, i.e. an assumed 1 m/s of noise, and at that value the measurement is decorative: the
   *  wheel speed arrives twenty times more often with an assumed 0.1 m/s, so GPS velocity contributed
   *  about one part in eight thousand and the fused speed was the wheel speed to six decimals.
   *
   *  Measured, Doppler is far better than 1 m/s. The residual between GNSS Doppler and the
   *  scale-corrected wheel speed is 0.066-0.101 m/s median on two runs, and that residual contains both
   *  sensors plus the 1 Hz sampling — so 0.1 m/s is a fair bound for Doppler alone, not 1.0.
   *
   *  A caveat that keeps this from being tightened further: `updateGpsVel` observes
   *  `[vx, vy] = v·[cos ψ, sin ψ]`, so it corrects heading as well as speed. Making it very confident
   *  would let it fight the yaw-rate chain. 0.1 m/s is where the evidence is; below that, measure first. */
  double gps_vel_R = 0.1 * 0.1;

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
  double last_fix_x_ = 0, last_fix_y_ = 0, last_fix_t_ = -1e9;
  bool have_last_fix_ = false;
  double pending_x_ = 0, pending_y_ = 0;
  int pending_agree_ = 0;

  std::vector<double> ref_x_, ref_y_;
  std::vector<double> odom_traj_x_, odom_traj_y_;
  std::vector<double> ekf_traj_x_, ekf_traj_y_;
};

}  // namespace adas
