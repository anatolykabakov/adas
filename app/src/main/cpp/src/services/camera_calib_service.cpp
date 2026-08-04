#include "services/camera_calib_service.h"

#include <cmath>

#include "messages.pb.h"
#include "utils/logger.h"

namespace adas {

CameraCalibService::CameraCalibService(Config config)
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

void CameraCalibService::configure()
{
  subscribe<CameraOdometrySample>(topics::kCameraOdometry,
                                  [this](const CameraOdometrySample& m) { onCameraOdometry(m); });
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<LaneUvMsg>(topics::kCalibLaneUv, [this](const LaneUvMsg& m) { onLaneUv(m); });
  LOGI("CameraCalibService: %s + chassis (+ optional %s) → %s + %s", topics::kCameraOdometry, topics::kCalibLaneUv,
       topics::kCameraCalib, topics::kCameraCalibDebug);
}

void CameraCalibService::reset()
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

void CameraCalibService::setIntrinsics(double fx, double fy, double cx, double cy)
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

void CameraCalibService::setHeight(double height_m)
{
  height_m_ = height_m;
  pose_calib_.setHeight(height_m);
  last_.camera_height_m = height_m;
}

void CameraCalibService::setEstimate(double pitch_deg, double yaw_deg)
{
  pose_calib_.reset(pitch_deg, yaw_deg, pose_calib_.validBlocks());
  vp_calib_.setEstimate(pitch_deg, yaw_deg);
  last_.pitch_deg = pitch_deg;
  last_.yaw_deg = yaw_deg;
}

void CameraCalibService::setVEgo(double v_ego_mps) { pose_calib_.setVEgo(v_ego_mps); }

void CameraCalibService::onChassis(const ChassisSample& msg) { pose_calib_.setVEgo(msg.speed_mps); }

void CameraCalibService::onCameraOdometry(const CameraOdometrySample& msg) { updateFromPose(msg); }

bool CameraCalibService::updateFromPose(const CameraOdometrySample& odom, double v_ego_mps)
{
  if (v_ego_mps >= 0.0)
    pose_calib_.setVEgo(v_ego_mps);
  const bool accepted = pose_calib_.handleCamOdom(odom);
  syncLastFromPose(odom.timestamp_us);

  if (accepted || pose_calib_.calibrated() || (pose_calib_.calPercent() % 5 == 0)) {
    publishState(odom.timestamp_us);
  }
  publishDebug(odom.timestamp_us, "pose");
  return accepted;
}

void CameraCalibService::syncLastFromPose(int64_t timestamp_us)
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

void CameraCalibService::onLaneUv(const LaneUvMsg& msg) { updateFromUv(msg.left_uv, msg.right_uv, msg.timestamp_us); }

bool CameraCalibService::updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv,
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
    publishState(timestamp_us);
    publishDebug(timestamp_us, "vp");
  }
  return committed;
}

void CameraCalibService::publishState(int64_t timestamp_us)
{
  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(timestamp_us / 1000);
  zmq.set_topic(topics::kCameraCalib);
  auto* c = zmq.mutable_camera_calib();
  c->set_timestamp(timestamp_us / 1000);
  c->set_roll_deg(last_.roll_deg);
  c->set_pitch_deg(last_.pitch_deg);
  c->set_yaw_deg(last_.yaw_deg);
  c->set_camera_height_m(last_.camera_height_m);
  c->set_fx(last_.fx);
  c->set_fy(last_.fy);
  c->set_cx(last_.cx);
  c->set_cy(last_.cy);
  c->set_calibration_success(last_.calibration_success);
  c->set_n_updates(last_.n_updates);
  c->set_vp_u(last_.vp_u);
  c->set_vp_v(last_.vp_v);
  c->set_has_vp(last_.has_vp);
  c->set_cal_percent(last_.cal_percent);
  c->set_cal_status(last_.cal_status);
  // Host/pyadas may construct the service without Middleware — keep state only.
  if (middleware())
    publish(topics::kCameraCalib, zmq);
}

void CameraCalibService::publishDebug(int64_t timestamp_us, const char* source)
{
  if (!middleware())
    return;

  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(timestamp_us / 1000);
  zmq.set_topic(topics::kCameraCalibDebug);
  auto* d = zmq.mutable_camera_calib_debug();
  d->set_timestamp(timestamp_us / 1000);
  d->set_source(source ? source : "none");
  d->set_status(PoseCalibrator::statusName(pose_calib_.status()));
  d->set_cal_status(static_cast<int>(pose_calib_.status()));
  d->set_cal_percent(pose_calib_.calPercent());
  d->set_calibration_success(pose_calib_.calibrated() || (last_.has_vp && vp_calib_.success()));
  d->set_roll_deg(last_.roll_deg);
  d->set_pitch_deg(last_.pitch_deg);
  d->set_yaw_deg(last_.yaw_deg);
  d->set_height_m(last_.camera_height_m);

  const auto& s = pose_calib_.lastSample();
  d->set_sample_accepted(s.accepted);
  d->set_v_ego(s.v_ego);
  d->set_odom_trans_x(s.odom_trans_x);
  d->set_odom_trans_y(s.odom_trans_y);
  d->set_odom_trans_z(s.odom_trans_z);
  d->set_odom_rot_z(s.odom_rot_z);
  d->set_odom_angle_std(s.odom_angle_std);
  d->set_gate_speed(s.gate_speed);
  d->set_gate_yaw_rate(s.gate_yaw_rate);
  d->set_gate_rpy_certain(s.gate_rpy_certain);
  d->set_gate_odom_valid(s.odom_valid);
  d->set_observed_pitch_deg(s.observed_pitch_deg);
  d->set_observed_yaw_deg(s.observed_yaw_deg);
  d->set_reject_reason(s.reject_reason ? s.reject_reason : "");

  const Vec3 spread = pose_calib_.calibSpread();
  d->set_spread_pitch_deg(spread.y() * 180.0 / M_PI);
  d->set_spread_yaw_deg(spread.z() * 180.0 / M_PI);
  d->set_spread_max_deg(spread.maxCoeff() * 180.0 / M_PI);
  d->set_valid_blocks(pose_calib_.validBlocks());
  d->set_block_idx(pose_calib_.blockIdx());
  d->set_sample_in_block(pose_calib_.sampleInBlock());
  d->set_old_rpy_weight(pose_calib_.oldRpyWeight());

  d->set_has_vp(vp_calib_.hasVp());
  d->set_vp_u(vp_calib_.vpU());
  d->set_vp_v(vp_calib_.vpV());
  d->set_vp_n_updates(vp_calib_.nUpdates());
  d->set_vp_history(vp_calib_.historySize());
  d->set_vp_success(vp_calib_.success());

  publish(topics::kCameraCalibDebug, zmq);
}

}  // namespace adas
