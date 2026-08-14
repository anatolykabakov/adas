#pragma once

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/utils/proto_convert.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"

namespace adas {
namespace services {
/**
 * \brief Learns the camera mounting from what the road looks like while driving.
 *
 * \details Pitch and yaw of a phone on a windshield mount are never known in advance, and a degree of
 * yaw moves a point 50 m ahead by nearly a metre — so the overlay, the lane offset and every projection
 * downstream depend on this estimate. Two independent measurements feed it: the vanishing point of the
 * lane lines (`sensors/lane_uv`) and the camera-odometry translation direction, which points along the
 * car's motion when the car is going straight.
 *
 * Publishes `calibration/camera` continuously, including while unconverged, so a bag always records the
 * geometry the run actually used.
 */
class CameraCalib : public adas::middleware::Service {
public:
  struct Config {
    double pitch_deg = 0.0;     ///< Seed pitch [deg], negative looking down.
    double yaw_deg = 0.0;       ///< Seed yaw [deg], positive to the left.
    double height_m = 1.40;     ///< Camera height above the road [m].
    double fx = 930.0;          ///< Focal length along x [px], for the frames actually delivered.
    double fy = 930.0;          ///< Focal length along y [px].
    double cx = 640.0;          ///< Principal point x [px].
    double cy = 360.0;          ///< Principal point y [px].
    int history_len = 50;       ///< Samples kept in the estimate; also what "converged" is measured against.
    double steer_ratio = 15.7;  ///< Steering ratio, to tell a straight road from a turn.
  };

  CameraCalib() : CameraCalib(Config{}) {}
  explicit CameraCalib(Config config);

  void configure() override;
  void reset() override;

  /**
   * \brief Camera intrinsics of the frames being fed in.
   *
   * \param[in] fx Focal length along x [px], for the frame size actually delivered.
   * \param[in] fy Focal length along y [px].
   * \param[in] cx Principal point x [px].
   * \param[in] cy Principal point y [px].
   *
   * \note These must match the frames, not the sensor: a 1280x720 calibration applied to a 640x360 stream
   * is off by a factor of two, and every projection made with it lands beside the road.
   */
  void setIntrinsics(double fx, double fy, double cx, double cy);
  /// \param[in] height_m Camera height above the road [m].
  void setHeight(double height_m);
  /**
   * \brief Seed the estimate, e.g. from a previous session.
   *
   * \param[in] pitch_deg Pitch [deg], negative looking down.
   * \param[in] yaw_deg Yaw [deg], positive to the left.
   */
  void setEstimate(double pitch_deg, double yaw_deg);
  /// \param[in] v_ego_mps Ego speed [m/s]. Both measurements are only meaningful while moving.
  void setVEgo(double v_ego_mps);

  /**
   * \brief Update from camera odometry: on a straight road the translation points along the optical axis.
   *
   * \param[in] odom One camera-odometry sample, including its per-axis standard deviations.
   * \param[in] v_ego_mps Ego speed [m/s]; negative reuses the value from \ref setVEgo.
   * \return True when the sample was accepted into the estimate.
   */
  bool updateFromPose(const CameraOdometrySample& odom, double v_ego_mps = -1.0);

  /**
   * \brief Update from the vanishing point of the lane lines.
   *
   * \param[in] left_uv Left lane line in image pixels.
   * \param[in] right_uv Right lane line in image pixels.
   * \param[in] timestamp_us Frame timestamp [us]; 0 means unknown and only bypasses the rate limit.
   * \return True when the two lines met inside the frame and the estimate was updated.
   */
  bool updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv, int64_t timestamp_us = 0);

  const CameraCalibrationState& last() const { return last_; }
  const Config& config() const { return config_; }
  PoseCalibrator& poseCalibrator() { return pose_calib_; }
  VanishingPointCalibrator& calibrator() { return vp_calib_; }
  int historyPending() const { return vp_calib_.historySize(); }
  int calPercent() const { return pose_calib_.calPercent(); }

private:
  void onLaneUv(const LaneUvMsg& msg);
  void onCameraOdometryProto(const adas::proto::CameraOdometry& odom);
  void onCameraOdometry(const CameraOdometrySample& msg);
  void onChassis(const ChassisSample& msg);
  void syncLastFromPose(int64_t timestamp_us);

  Config config_;
  PoseCalibrator pose_calib_;
  VanishingPointCalibrator vp_calib_;
  double height_m_ = 1.40;
  double fx_ = 930.0, fy_ = 930.0, cx_ = 640.0, cy_ = 360.0;
  CameraCalibrationState last_;
};

}  // namespace services

}  // namespace adas
