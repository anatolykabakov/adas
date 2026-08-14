#pragma once

#include <Eigen/Dense>

#include "adas/utils/math_utils.h"

namespace adas {
/**
 * \brief Extended Kalman filter over the vehicle state: position, heading, speed, yaw rate.
 *
 * \details The prediction is a bicycle model; corrections arrive as separate `update*` calls, one per
 * sensor, each with its own noise. Policy - which measurement is trustworthy, when to reseed - lives in
 * `OnlineLocalizer`, so this class stays a filter and nothing else.
 */
class VehicleEKF {
public:
  using Vec5 = Eigen::Matrix<double, 5, 1>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;
  using Mat2 = Eigen::Matrix2d;
  using Mat52 = Eigen::Matrix<double, 5, 2>;
  using Mat25 = Eigen::Matrix<double, 2, 5>;

  enum class GpsPosResult { Accepted, Rejected, Reseeded };

  /**
   * \param[in] wheelbase Wheelbase [m], the bicycle model's only geometry.
   * \param[in] gps_noise_pos Default position measurement noise [m], used when a fix reports none.
   * \param[in] imu_noise_yaw_rate Gyro noise [rad/s].
   */
  explicit VehicleEKF(double wheelbase = 2.636, double gps_noise_pos = 5.0, double imu_noise_yaw_rate = 0.02);

  void reset(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0, double pos_unc = 10.0,
             double yaw_unc = 0.5, double v_unc = 2.0, double yaw_rate_unc = 0.1);

  /**
   * \brief Propagate the state one step with the bicycle model.
   *
   * \param[in] v_measured Wheel speed [m/s].
   * \param[in] steering_angle Road-wheel angle [rad], not the steering wheel.
   * \param[in] dt Step [s]; a gap longer than a tick must be passed as it is, not split.
   */
  void predict(double v_measured, double steering_angle, double dt);

  /** Treat the bicycle model as a *measurement* of yaw rate instead of the truth.
   *
   *  The old `predict` advanced heading with `v·tan(δ)/L` and then **overwrote** the yaw-rate state
   *  with it, so whatever the gyro had taught the filter was discarded every step. Measurement
   *  reached heading only through cross-covariance, with weight `dt·(1 − K₄) ≈ 0.12·dt` on our
   *  `Q₄₄ = 0.05²` and `R_imu = 0.02²` — heading came out 0.88 · bicycle + 0.12 · measured.
   *
   *  That would be fine if the bicycle model were right. It is not: measured understeer on this car
   *  is `κ_fact/κ_kin` = 0.54 at 22 m/s, so the prediction over-turns by 1.85× and heading rotates
   *  1.75× faster than truth. On the road GPS heading hides it (`updateGpsYaw` hard-snaps above
   *  0.5 rad); without GPS it drifts.
   *
   *  With this on, yaw rate is a state with its own random walk, heading integrates the state, and
   *  the bicycle model enters as a weak measurement — weak because `model_noise_yaw_rate` reflects
   *  how wrong it is, against the gyro's 0.02 rad/s. The gyro is trusted, the model fills gaps.
   *
   *  Off restores the previous behaviour exactly, for A/B on recordings. */
  void setYawRateIsAState(bool on, double model_noise_yaw_rate = 0.15)
  {
    yaw_rate_is_state_ = on;
    R_model_ = model_noise_yaw_rate * model_noise_yaw_rate;
  }
  /// True when yaw rate is estimated rather than taken from the bicycle model each step.
  bool yawRateIsAState() const { return yaw_rate_is_state_; }

  /** Treat the wheel speed as a *measurement* of speed instead of assigning it to the state.
   *
   *  Same defect as the yaw rate, and it hid the same way. `predict` used to do `state_(3) = v_measured`
   *  on every tick — roughly every 10 ms — while `updateGpsVel` runs at most every 0.2 s and only when
   *  the GPS course is valid. So the velocity update was overwritten about twenty times before the next
   *  GPS sample arrived, and the fused speed inherited the wheel-speed scale exactly: measured against
   *  GNSS Doppler, `localization/pose.v` gives 1.011 and raw CAN gives 1.012.
   *
   *  With this on, speed is a state with a random walk and the wheel speed is a measurement with
   *  `wheel_noise_speed`. GPS velocity then survives to the next tick.
   *
   *  What this does **not** do is make the 1.2 % scale identifiable. Two measurements of the same
   *  quantity with one of them biased by an unknown constant cannot separate the bias from the truth —
   *  that needs the scale itself as a state, which is where `paramsd` lives. This change is what makes
   *  such a state possible; it is not a substitute for it. */
  void setSpeedIsAState(bool on, double wheel_noise_speed = 0.1)
  {
    speed_is_state_ = on;
    R_wheel_ = wheel_noise_speed * wheel_noise_speed;
  }
  /// True when speed is estimated, which is what makes the wheel-scale error observable.
  bool speedIsAState() const { return speed_is_state_; }

  /**
   * \brief Correct position from a GNSS fix, or reseed from it when the state has diverged.
   *
   * \param[in] gps_x East [m] in the same local plane as the state.
   * \param[in] gps_y North [m].
   * \param[in] max_innovation Fixes further than this from the prediction are rejected [m].
   * \param[in] reseed_innovation Beyond this the fix is treated as ground truth and the state is moved
   * onto it. Must sit outside `max_innovation`, or a rejected fix can still teleport the filter.
   * \param[in] allow_reseed Caller's permission to reseed at all; the policy for it lives one layer up.
   * \param[in] R_pos Position variance [m^2]; negative uses the constructor's default.
   * \return Whether the fix was accepted, rejected, or used as a reseed.
   */
  GpsPosResult updateGps(double gps_x, double gps_y, double max_innovation = 50.0, double reseed_innovation = 100.0,
                         bool allow_reseed = true, double R_pos = -1.0);

  /**
   * \brief Correct heading from the GNSS course.
   *
   * \param[in] yaw_enu Course as an ENU heading [rad]: 0 is east, positive counter-clockwise.
   * \param[in] R_yaw Measurement variance [rad^2].
   * \param[in] force Apply even when the innovation looks too large, for a deliberate snap.
   * \return True when the update was applied.
   */
  bool updateGpsYaw(double yaw_enu, double R_yaw = 0.05, bool force = false);

  /**
   * \brief Correct speed and heading from the Doppler velocity vector.
   *
   * \param[in] vx_east East component [m/s].
   * \param[in] vy_north North component [m/s].
   * \param[in] R_vel Variance [m^2/s^2]. This observes `v * [cos psi, sin psi]`, so it corrects heading
   * as well as speed — making it very confident lets it fight the yaw-rate chain.
   * \return True when the update was applied.
   */
  bool updateGpsVel(double vx_east, double vy_north, double R_vel = 1.0);

  /// \param[in] yaw_rate_imu Gyro yaw rate in the car's frame [rad/s], positive counter-clockwise.
  void updateImu(double yaw_rate_imu) { applyYawRateUpdate(yaw_rate_imu, R_imu_, YawRateSource::Imu); }

  void updateCamOdoYawRate(double yaw_rate_cam, double R = 0.05)
  {
    applyYawRateUpdate(yaw_rate_cam, R, YawRateSource::CameraOdometry);
  }

  /// East [m] in the local plane the filter was seeded in.
  double x() const { return state_(0); }
  /// North [m].
  double y() const { return state_(1); }
  /// Heading [rad], ENU: 0 east, positive counter-clockwise.
  double yaw() const { return state_(2); }
  /// Forward speed [m/s].
  double v() const { return state_(3); }
  /// Yaw rate [rad/s].
  double yawRate() const { return state_(4); }

  int prediction_count = 0;
  int gps_update_count = 0;
  int gps_rejected_count = 0;
  int gps_reseed_count = 0;
  int gps_yaw_update_count = 0;
  int gps_vel_update_count = 0;
  int imu_update_count = 0;
  int cam_odo_update_count = 0;
  int model_update_count = 0;
  int wheel_speed_update_count = 0;

  double wheelbase = 2.636;

private:
  enum class YawRateSource { Imu, CameraOdometry, BicycleModel };
  void applySpeedUpdate(double v_meas, double R);
  void applyYawRateUpdate(double yaw_rate_meas, double R, YawRateSource source);

  Vec5 state_ = Vec5::Zero();
  Mat5 P_ = Mat5::Zero();
  Mat5 Q_ = Mat5::Zero();
  Mat2 R_gps_ = Mat2::Zero();
  double R_imu_ = 0.0004;
  double R_model_ = 0.15 * 0.15;
  double R_wheel_ = 0.1 * 0.1;
  bool yaw_rate_is_state_ = true;
  bool speed_is_state_ = true;
  int consecutive_gps_rejects_ = 0;
};

}  // namespace adas
