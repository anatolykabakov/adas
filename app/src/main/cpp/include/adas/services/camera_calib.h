#pragma once

#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"

namespace adas {

namespace services {

class CameraCalib : public adas::middleware::Service {
public:
  struct Config {
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double height_m = 1.40;
    double fx = 930.0;
    double fy = 930.0;
    double cx = 640.0;
    double cy = 360.0;
    int history_len = 50;
  };

  CameraCalib() : CameraCalib(Config{}) {}
  explicit CameraCalib(Config config);

  void configure() override;
  void reset() override;

  void setIntrinsics(double fx, double fy, double cx, double cy);
  void setHeight(double height_m);
  void setEstimate(double pitch_deg, double yaw_deg);
  void setVEgo(double v_ego_mps);

  bool updateFromPose(const CameraOdometrySample& odom, double v_ego_mps = -1.0);

  bool updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv, int64_t timestamp_us = 0);

  const CameraCalibrationState& last() const { return last_; }
  const Config& config() const { return config_; }
  PoseCalibrator& poseCalibrator() { return pose_calib_; }
  VanishingPointCalibrator& calibrator() { return vp_calib_; }
  int historyPending() const { return vp_calib_.historySize(); }
  int calPercent() const { return pose_calib_.calPercent(); }

private:
  void onLaneUv(const LaneUvMsg& msg);
  void onCameraOdometry(const CameraOdometrySample& msg);
  void onChassis(const ChassisSample& msg);
  void syncLastFromPose(int64_t timestamp_us);
  void publishState(int64_t timestamp_us);
  void publishDebug(int64_t timestamp_us, const char* source);

  Config config_;
  PoseCalibrator pose_calib_;
  VanishingPointCalibrator vp_calib_;
  double height_m_ = 1.40;
  double fx_ = 930.0, fy_ = 930.0, cx_ = 640.0, cy_ = 360.0;
  CameraCalibrationState last_;
};

}  // namespace services

}  // namespace adas
