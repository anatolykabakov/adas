#include "adas/utils/online_localizer.h"

#include <cmath>

namespace adas {

OnlineLocalizer::OnlineLocalizer(double wheelbase, double gps_noise_pos, double gps_update_interval,
                                 bool imu_every_step)
  : ekf_(wheelbase, gps_noise_pos, 0.05)
  , wheelbase_(wheelbase)
  , gps_update_interval_(gps_update_interval)
  , imu_every_step_(imu_every_step)
{
}

void OnlineLocalizer::record(double rx, double ry, double ox, double oy, double ex, double ey)
{
  ref_x_.push_back(rx);
  ref_y_.push_back(ry);
  odom_traj_x_.push_back(ox);
  odom_traj_y_.push_back(oy);
  ekf_traj_x_.push_back(ex);
  ekf_traj_y_.push_back(ey);
}

void OnlineLocalizer::snapYaw(double yaw_enu)
{
  ekf_.reset(ekf_.x(), ekf_.y(), yaw_enu, ekf_.v(), ekf_.yawRate(), 5.0, 0.15, 1.0, 0.1);
  odom_yaw_ = yaw_enu;
  yaw_seeded_ = true;
}

void OnlineLocalizer::reset(double x, double y, double yaw, double v, double yaw_rate)
{
  ekf_.reset(x, y, yaw, v, yaw_rate, 1.0, 0.2, 1.0, 0.1);
  odom_x_ = x;
  odom_y_ = y;
  odom_yaw_ = yaw;
  t_ = 0;
  last_gps_t_ = -1e9;
  last_gps_msg_us_ = -1;
  initialized_ = true;
  yaw_seeded_ = std::abs(yaw) > 1e-6;
  ref_x_.clear();
  ref_y_.clear();
  odom_traj_x_.clear();
  odom_traj_y_.clear();
  ekf_traj_x_.clear();
  ekf_traj_y_.clear();
  record(x, y, x, y, x, y);
}

std::tuple<double, double, double> OnlineLocalizer::step(double dt, double speed_mps, double steer_rad,
                                                         std::optional<double> yaw_rate, std::optional<GpsSample> gps,
                                                         std::optional<Vec2> ref_xy, std::optional<double> cam_yaw_rate)
{
  if (!initialized_) {
    double seed_x = 0, seed_y = 0, seed_yaw = 0;
    if (ref_xy) {
      seed_x = ref_xy->x();
      seed_y = ref_xy->y();
    } else if (gps && gps->valid) {
      seed_x = gps->x;
      seed_y = gps->y;
      if (gps->course_valid)
        seed_yaw = gps->yaw_enu;
    }
    reset(seed_x, seed_y, seed_yaw, speed_mps, yaw_rate.value_or(0.0));
    yaw_seeded_ = (gps && gps->course_valid) || std::abs(seed_yaw) > 1e-6;
  }

  dt = std::max(dt, 1e-4);
  const double v = speed_mps;
  const double delta = steer_rad;
  double yr_bicycle = 0.0;
  if (std::abs(delta) > 1e-3 && std::abs(v) > 0.01) {
    yr_bicycle = v * std::tan(delta) / wheelbase_;
  }
  odom_x_ += v * std::cos(odom_yaw_) * dt;
  odom_y_ += v * std::sin(odom_yaw_) * dt;
  odom_yaw_ = normalizeAngle(odom_yaw_ + yr_bicycle * dt);

  ekf_.predict(v, delta, dt);

  if (yaw_seeded_ && yaw_rate && imu_every_step_) {
    const double yr = *yaw_rate;
    if (std::abs(v) < 0.5 || std::abs(yr - yr_bicycle) < imu_agree_rad_s) {
      ekf_.updateImu(yr);
    }
  }

  if (yaw_seeded_ && cam_yaw_rate && std::abs(v) > 1.0) {
    double w = *cam_yaw_rate;
    if (invert_cam_yaw_rate)
      w = -w;
    if (std::abs(w) < 1.5 && std::abs(w - yr_bicycle) < imu_agree_rad_s) {
      ekf_.updateCamOdoYawRate(w, cam_odo_R);
    }
  }

  t_ += dt;
  const bool gps_new = gps && gps->valid && (gps->timestamp_us <= 0 || gps->timestamp_us > last_gps_msg_us_);
  if (gps_new && (t_ - last_gps_t_) >= gps_update_interval_) {
    if (gps->timestamp_us > 0)
      last_gps_msg_us_ = gps->timestamp_us;

    if (!yaw_seeded_ && gps->course_valid && use_gps_course)
      snapYaw(gps->yaw_enu);

    // With position off the filter is dead reckoning: no update, and no reseed either, so the pose is
    // whatever the yaw-rate chain and the wheel speed make of it. That is the point of the switch.
    const auto pos = use_gps_position ? ekf_.updateGps(gps->x, gps->y, 25.0, 50.0) : VehicleEKF::GpsPosResult::Accepted;
    if (pos != VehicleEKF::GpsPosResult::Rejected) {
      last_gps_t_ = t_;

      if (gps->course_valid) {
        if (use_gps_course) {
          const double yaw_err = std::abs(normalizeAngle(gps->yaw_enu - ekf_.yaw()));
          if (pos == VehicleEKF::GpsPosResult::Reseeded || yaw_err > 0.5) {
            snapYaw(gps->yaw_enu);
          } else {
            ekf_.updateGpsYaw(gps->yaw_enu, 0.05);
          }
        }
        if (use_gps_velocity)
          ekf_.updateGpsVel(gps->vx, gps->vy, gps_vel_R);
      }

      if (pos == VehicleEKF::GpsPosResult::Reseeded) {
        odom_x_ = ekf_.x();
        odom_y_ = ekf_.y();
        odom_yaw_ = ekf_.yaw();
      }
    }
  }

  const double ex = ekf_.x(), ey = ekf_.y();
  double rx = ex, ry = ey;
  if (ref_xy) {
    rx = ref_xy->x();
    ry = ref_xy->y();
  } else if (gps && gps->valid) {
    rx = gps->x;
    ry = gps->y;
  }
  record(rx, ry, odom_x_, odom_y_, ex, ey);
  return {ex, ey, ekf_.yaw()};
}

}  // namespace adas
