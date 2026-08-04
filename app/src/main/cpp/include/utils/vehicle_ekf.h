#pragma once

#include <Eigen/Dense>

#include "utils/math_utils.h"

namespace adas {

class VehicleEKF {
public:
  using Vec5 = Eigen::Matrix<double, 5, 1>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;
  using Mat2 = Eigen::Matrix2d;
  using Mat52 = Eigen::Matrix<double, 5, 2>;
  using Mat25 = Eigen::Matrix<double, 2, 5>;

  enum class GpsPosResult { Accepted, Rejected, Reseeded };

  explicit VehicleEKF(double wheelbase = 2.636, double gps_noise_pos = 5.0, double imu_noise_yaw_rate = 0.02);

  void reset(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0, double pos_unc = 10.0,
             double yaw_unc = 0.5, double v_unc = 2.0, double yaw_rate_unc = 0.1);

  void predict(double v_measured, double steering_angle, double dt);

  GpsPosResult updateGps(double gps_x, double gps_y, double max_innovation = 50.0, double reseed_innovation = 100.0);

  bool updateGpsYaw(double yaw_enu, double R_yaw = 0.05, bool force = false);

  bool updateGpsVel(double vx_east, double vy_north, double R_vel = 1.0);

  void updateImu(double yaw_rate_imu) { applyYawRateUpdate(yaw_rate_imu, R_imu_, false); }

  void updateCamOdoYawRate(double yaw_rate_cam, double R = 0.05) { applyYawRateUpdate(yaw_rate_cam, R, true); }

  double x() const { return state_(0); }
  double y() const { return state_(1); }
  double yaw() const { return state_(2); }
  double v() const { return state_(3); }
  double yawRate() const { return state_(4); }

  int prediction_count = 0;
  int gps_update_count = 0;
  int gps_rejected_count = 0;
  int gps_reseed_count = 0;
  int gps_yaw_update_count = 0;
  int gps_vel_update_count = 0;
  int imu_update_count = 0;
  int cam_odo_update_count = 0;

  double wheelbase = 2.636;

private:
  void applyYawRateUpdate(double yaw_rate_meas, double R, bool count_as_cam);

  Vec5 state_ = Vec5::Zero();
  Mat5 P_ = Mat5::Zero();
  Mat5 Q_ = Mat5::Zero();
  Mat2 R_gps_ = Mat2::Zero();
  double R_imu_ = 0.0004;
  int consecutive_gps_rejects_ = 0;
};

}  // namespace adas
