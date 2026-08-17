#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "adas/utils/adas_topics.h"
#include "adas/utils/math_utils.h"
#include "adas/utils/vehicle_ekf.h"

namespace adas {
/**
 * \brief The pose estimator: wheel speed, yaw rate and GNSS into one local pose.
 *
 * \details Wraps `VehicleEKF` with the policy the filter itself should not contain - which fixes are
 * trustworthy, when a heading may be snapped, and when a diverged position may be reseeded. Those
 * decisions live here because they are calibrated against recorded drives, not derived from the model.
 */
class OnlineLocalizer {
public:
  OnlineLocalizer(double wheelbase = 2.636, double gps_noise_pos = 0.5, double gps_update_interval = 0.2,
                  bool imu_every_step = true);

  /// Reseed the pose: \param[in] x east [m], \param[in] y north [m], \param[in] yaw ENU heading [rad].
  void reset(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0);

  /**
   * \brief One filter step: predict, then apply whichever measurements arrived.
   *
   * \param[in] dt Step [s].
   * \param[in] speed_mps Wheel speed [m/s].
   * \param[in] steer_rad Road-wheel angle [rad]. Must be real: the gyro is gated on agreement with the
   * bicycle-model yaw rate, so a zero here rejects every turn sharper than 20 deg/s.
   * \param[in] yaw_rate Gyro yaw rate [rad/s], if available.
   * \param[in] gps A fix, if one arrived this step.
   * \param[in] ref_xy Reference position, recorded for offline plots only; never a measurement.
   * \param[in] cam_yaw_rate Camera-odometry yaw rate [rad/s], if that source is enabled.
   * \return Position and heading after the step: x [m], y [m], yaw [rad].
   */
  std::tuple<double, double, double> step(double dt, double speed_mps, double steer_rad,
                                          std::optional<double> yaw_rate = std::nullopt,
                                          std::optional<GpsSample> gps = std::nullopt,
                                          std::optional<Vec2> ref_xy = std::nullopt,
                                          std::optional<double> cam_yaw_rate = std::nullopt);

  /// Fused position [m] and heading [rad] in the local plane.
  double x() const { return ekf_.x(); }
  double y() const { return ekf_.y(); }
  double yaw() const { return ekf_.yaw(); }
  /// The filter underneath, for tests and for tuning its noise directly.
  VehicleEKF& ekf() { return ekf_; }
  const VehicleEKF& ekf() const { return ekf_; }

  /// Recorded trajectories for offline plots: the reference, dead reckoning, and the fused estimate.
  const std::vector<double>& refX() const { return ref_x_; }
  const std::vector<double>& refY() const { return ref_y_; }
  const std::vector<double>& odomX() const { return odom_traj_x_; }
  const std::vector<double>& odomY() const { return odom_traj_y_; }
  const std::vector<double>& ekfX() const { return ekf_traj_x_; }
  const std::vector<double>& ekfY() const { return ekf_traj_y_; }

  double imu_agree_rad_s = 0.35;
  double cam_odo_R = 0.05;

  double max_gps_speed_mismatch_mps = 12.0;

  int reseed_agree_count = 3;
  double reseed_agree_radius_m = 8.0;
  double max_gps_accuracy_m = 25.0;
  /** \brief Heading snap while moving [rad], and how long the error must persist [s].
   *
   *  Without it the only way to snap was `reseed_agree_count` consecutive fixes agreeing within
   *  `reseed_agree_radius_m` — a standstill condition, since at 25 m/s consecutive fixes are 25 m apart
   *  and it never held. Heading error then grew unbounded: drive 2026_08_13_23_01_56 accumulated 62°
   *  over 320 s. */
  double yaw_snap_err_rad = 0.5;
  double yaw_snap_hold_s = 3.0;
  /** \brief How long position updates may be rejected before reseeding while moving [s].
   *
   *  `reseed_agree_count` agreeing fixes is a standstill condition, so a runaway position used to wait
   *  for the first stop: on 2026_08_13_23_01_56 it came back from 2.8 km to 6 m only after 350 s. A
   *  position rejected for longer than this while the reported accuracy is good is a diverged state, not
   *  a receiver outlier. */
  double pos_reseed_hold_s = 3.0;
  int gps_unphysical_count = 0;

  double gps_vel_R = 0.1 * 0.1;

public:
  int gpsRejectedAccuracy() const { return gps_rejected_accuracy_; }

private:
  void record(double rx, double ry, double ox, double oy, double ex, double ey);
  void snapYaw(double yaw_enu);

  VehicleEKF ekf_;
  double wheelbase_ = 2.636;
  double gps_noise_pos_ = 0.5;
  int gps_rejected_accuracy_ = 0;
  double gps_update_interval_ = 0.2;
  bool imu_every_step_ = true;
  bool initialized_ = false;
  bool yaw_seeded_ = false;
  /// When the heading started disagreeing with GPS beyond the threshold; 0 means it agrees.
  double yaw_bad_since_ = 0.0;
  /// When position updates started being rejected while accuracy was good; 0 means they are accepted.
  double pos_bad_since_ = 0.0;
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
