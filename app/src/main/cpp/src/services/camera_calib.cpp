#include "adas/services/camera_calib.h"

#include "adas/utils/proto_convert.h"

#include <cmath>

#include "messages.pb.h"
#include "adas/utils/logger.h"

namespace adas {
namespace services {
CameraCalib::CameraCalib(Config config)
  : config_(config)
  , pose_calib_(config.pitch_deg, config.yaw_deg, config.height_m)
  , vp_calib_(config.history_len, config.pitch_deg, config.yaw_deg)
  , height_m_(config.height_m)
  , fx_(config.fx)
  , fy_(config.fy)
  , cx_(config.cx)
  , cy_(config.cy)
{
  last_.pitch_deg = config.pitch_deg;
  last_.yaw_deg = config.yaw_deg;
  last_.camera_height_m = config.height_m;
  last_.fx = config.fx;
  last_.fy = config.fy;
  last_.cx = config.cx;
  last_.cy = config.cy;
}

void CameraCalib::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, config_.steer_ratio));
  });
  subscribe<adas::proto::CameraOdometry>(
      topics::kCameraOdometry, [this](const adas::proto::CameraOdometry& payload) { onCameraOdometryProto(payload); });
  subscribe<LaneUvMsg>(topics::kCalibLaneUv, [this](const LaneUvMsg& m) { onLaneUv(m); });
  // The device tells us its own lens once the camera opens. Until this subscription existed the
  // message was published and nobody listened, so the whole system ran on the config's prior — a
  // number typed in for whichever phone it was typed in for.
  subscribe<adas::proto::CameraIntrinsics>(topics::kCameraIntrinsics,
                                           [this](const adas::proto::CameraIntrinsics& m) { onIntrinsics(m); });
  LOGI("CameraCalib: %s + chassis (+ optional %s, %s) → %s + %s", topics::kCameraOdometry, topics::kCalibLaneUv,
       topics::kCameraIntrinsics, topics::kCameraCalib, topics::kCameraCalibDebug);
}

/**
 * \brief Adopt the intrinsics the device reports, once they look usable.
 *
 * \details The config value is a prior, not a measurement: it is right for the phone it was written
 * for and silently wrong for any other. A report is taken only when it carries a plausible focal
 * length — a zero or a near-zero means the device has the field but never filled it, which is common
 * enough that Camera2 documents it.
 */
void CameraCalib::onIntrinsics(const adas::proto::CameraIntrinsics& msg)
{
  if (msg.intrinsic_calibration_size() < 4)
    return;
  const double fx = msg.intrinsic_calibration(0);
  const double fy = msg.intrinsic_calibration(1);
  const double cx = msg.intrinsic_calibration(2);
  const double cy = msg.intrinsic_calibration(3);
  if (!(fx > 1.0) || !(fy > 1.0))
    return;

  // Units. The numbers must belong to the same frame the rest of the system works in, and this is
  // not a formality: on 2026-08-16 the intrinsics of the downscaled bag frame arrived here (fx
  // 475.5, principal point 320x180), were taken as parameters of the 1280x720 frame, and the model
  // warp built a projection at half the focal length with the centre in a corner. There are no lane
  // lines in such a frame, and the drive merely looks unsuccessful.
  //
  // The check uses what the message says about itself: the principal point must sit near the middle
  // of the declared frame. The tolerance is generous — a real principal point drifts off centre, but
  // not by half a frame.
  const double w = msg.capture_width();
  const double h = msg.capture_height();
  if (w > 1.0 && h > 1.0) {
    const double dx = std::fabs(cx - w * 0.5) / w;
    const double dy = std::fabs(cy - h * 0.5) / h;
    if (dx > 0.25 || dy > 0.25) {
      LOGW("CameraCalib: intrinsics rejected — centre (%.1f, %.1f) is not mid-frame for %.0fx%.0f; "
           "they look computed for a different frame",
           cx, cy, w, h);
      return;
    }
  } else {
    LOGW("CameraCalib: intrinsics without a frame size — nothing to check the units against, rejected");
    return;
  }

  if (fx == fx_ && fy == fy_ && cx == cx_ && cy == cy_)
    return;

  LOGI("CameraCalib: intrinsics from the device (source %d): fx %.1f→%.1f fy %.1f→%.1f cx %.1f→%.1f cy %.1f→%.1f",
       static_cast<int>(msg.source()), fx_, fx, fy_, fy, cx_, cx, cy_, cy);
  setIntrinsics(fx, fy, cx, cy);
  intrinsics_source_ = static_cast<int>(msg.source());
}

void CameraCalib::reset()
{
  pose_calib_.reset(last_.pitch_deg, last_.yaw_deg, 0);
  vp_calib_.reset();
  vp_calib_.setEstimate(last_.pitch_deg, last_.yaw_deg);
  last_.calibration_success = false;
  last_.has_vp = false;
  last_.n_updates = 0;
  last_.cal_percent = 0;
  last_.cal_status = PoseCalibrator::Uncalibrated;
}

void CameraCalib::setIntrinsics(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  last_.fx = fx;
  last_.fy = fy;
  last_.cx = cx;
  last_.cy = cy;
}

void CameraCalib::setHeight(double height_m)
{
  height_m_ = height_m;
  pose_calib_.setHeight(height_m);
  last_.camera_height_m = height_m;
}

void CameraCalib::setEstimate(double pitch_deg, double yaw_deg)
{
  pose_calib_.reset(pitch_deg, yaw_deg, pose_calib_.validBlocks());
  vp_calib_.setEstimate(pitch_deg, yaw_deg);
  last_.pitch_deg = pitch_deg;
  last_.yaw_deg = yaw_deg;
}

void CameraCalib::setVEgo(double v_ego_mps) { pose_calib_.setVEgo(v_ego_mps); }

void CameraCalib::onChassis(const ChassisSample& msg) { pose_calib_.setVEgo(msg.speed_mps); }

void CameraCalib::onCameraOdometryProto(const adas::proto::CameraOdometry& odom)
{
  const auto sample = cameraOdometryToSample(odom);
  if (sample.valid)
    onCameraOdometry(sample);
}

void CameraCalib::onCameraOdometry(const CameraOdometrySample& msg) { updateFromPose(msg); }

bool CameraCalib::updateFromPose(const CameraOdometrySample& odom, double v_ego_mps)
{
  if (v_ego_mps >= 0.0)
    pose_calib_.setVEgo(v_ego_mps);
  const bool accepted = pose_calib_.handleCamOdom(odom);
  syncLastFromPose(odom.timestamp_us);

  if (accepted || pose_calib_.calibrated() || (pose_calib_.calPercent() % 5 == 0)) {
    publish(topics::kCameraCalib, createCameraCalibState(last_, odom.timestamp_us));
  }
  publish(topics::kCameraCalibDebug, createCameraCalibDebug(last_, pose_calib_, vp_calib_, odom.timestamp_us, "pose"));
  return accepted;
}

void CameraCalib::syncLastFromPose(int64_t timestamp_us)
{
  last_.pitch_deg = pose_calib_.pitchDeg();
  last_.yaw_deg = pose_calib_.yawDeg();
  last_.roll_deg = pose_calib_.rollDeg();
  last_.camera_height_m = pose_calib_.heightM();
  last_.calibration_success = pose_calib_.calibrated();
  last_.n_updates = pose_calib_.validBlocks();
  last_.cal_percent = pose_calib_.calPercent();
  last_.cal_status = static_cast<int>(pose_calib_.status());
  last_.timestamp_us = timestamp_us;
}

void CameraCalib::onLaneUv(const LaneUvMsg& msg) { updateFromUv(msg.left_uv, msg.right_uv, msg.timestamp_us); }

bool CameraCalib::updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv,
                               int64_t timestamp_us)
{
  const bool committed = vp_calib_.updateFromUv(left_uv, right_uv, fx_, fy_, cx_, cy_);

  if (committed || vp_calib_.hasVp()) {
    if (!pose_calib_.calibrated()) {
      last_.pitch_deg = vp_calib_.pitchDeg();
      last_.yaw_deg = vp_calib_.yawDeg();
      pose_calib_.reset(vp_calib_.pitchDeg(), vp_calib_.yawDeg());
      vp_calib_.setEstimate(vp_calib_.pitchDeg(), vp_calib_.yawDeg());
    }
    last_.calibration_success = vp_calib_.success() || pose_calib_.calibrated();
    last_.n_updates = vp_calib_.nUpdates();
    last_.has_vp = vp_calib_.hasVp();
    last_.vp_u = vp_calib_.vpU();
    last_.vp_v = vp_calib_.vpV();
    last_.timestamp_us = timestamp_us;
    publish(topics::kCameraCalib, createCameraCalibState(last_, timestamp_us));
    publish(topics::kCameraCalibDebug, createCameraCalibDebug(last_, pose_calib_, vp_calib_, timestamp_us, "vp"));
  }
  return committed;
}

}  // namespace services
}  // namespace adas
