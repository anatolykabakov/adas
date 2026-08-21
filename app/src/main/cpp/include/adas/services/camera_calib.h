#pragma once

#include "camera_intrinsics.pb.h"

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/utils/proto_convert.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"

namespace adas {
namespace services {
/** Learns the camera mounting from what the road looks like while driving. */
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

  /// Constructs with the default config.
  CameraCalib() : CameraCalib(Config{}) {}
  /// \param[in] config Mount priors and calibration settings.
  explicit CameraCalib(Config config);

  void configure() override;
  void reset() override;

  /**
   * \brief Camera intrinsics of the frames being fed in.
   * \param[in] fx Focal length along x [px], for the frame size actually delivered.
   * \param[in] fy Focal length along y [px].
   * \param[in] cx Principal point x [px].
   * \param[in] cy Principal point y [px].
   */
  void setIntrinsics(double fx, double fy, double cx, double cy);

  /// Which source the running intrinsics came from, as `CameraIntrinsics.Source`.
  int intrinsicsSource() const { return intrinsics_source_; }
  /// \param[in] height_m Camera height above the road [m].
  void setHeight(double height_m);
  /**
   * \brief Seed the estimate, e.g.
   * \param[in] pitch_deg Pitch [deg], negative looking down.
   * \param[in] yaw_deg Yaw [deg], positive to the left.
   */
  void setEstimate(double pitch_deg, double yaw_deg);
  /// \param[in] v_ego_mps Ego speed [m/s]. Both measurements are only meaningful while moving.
  void setVEgo(double v_ego_mps);

  /**
   * \brief Update from camera odometry: on a straight road the translation points along the optical axis.
   * \param[in] odom One camera-odometry sample, including its per-axis standard deviations.
   * \param[in] v_ego_mps Ego speed [m/s]; negative reuses the value from \ref setVEgo.
   * \return True when the sample was accepted into the estimate.
   */
  bool updateFromPose(const CameraOdometrySample& odom, double v_ego_mps = -1.0);

  /**
   * \brief Update from the vanishing point of the lane lines.
   * \param[in] left_uv Left lane line in image pixels.
   * \param[in] right_uv Right lane line in image pixels.
   * \param[in] timestamp_us Frame timestamp [us]; 0 means unknown and only bypasses the rate limit.
   * \return True when the two lines met inside the frame and the estimate was updated.
   */
  bool updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv, int64_t timestamp_us = 0);

  /// \return The calibration last published.
  const CameraCalibrationState& last() const { return last_; }
  /// \return The config in force.
  const Config& config() const { return config_; }
  /// \return The pose-based calibrator, for tests and offline tools.
  PoseCalibrator& poseCalibrator() { return pose_calib_; }
  /// \return The vanishing-point calibrator, for tests and offline tools.
  VanishingPointCalibrator& calibrator() { return vp_calib_; }
  /// \return Frames collected toward the next vanishing-point estimate.
  int historyPending() const { return vp_calib_.historySize(); }
  /// \return Pose-calibration progress [0..100].
  int calPercent() const { return pose_calib_.calPercent(); }

private:
  void onLaneUv(const LaneUvMsg& msg);
  void onIntrinsics(const adas::proto::CameraIntrinsics& msg);
  void onCameraOdometryProto(const adas::proto::CameraOdometry& odom);
  void onCameraOdometry(const CameraOdometrySample& msg);
  void onChassis(const ChassisSample& msg);
  void syncLastFromPose(int64_t timestamp_us);

  Config config_;
  PoseCalibrator pose_calib_;
  VanishingPointCalibrator vp_calib_;
  double height_m_ = 1.40;
  double fx_ = 930.0, fy_ = 930.0, cx_ = 640.0, cy_ = 360.0;
  /// Which source the current intrinsics came from; 0 until the device reports its own.
  int intrinsics_source_ = 0;
  CameraCalibrationState last_;
};

}  // namespace services

}  // namespace adas
