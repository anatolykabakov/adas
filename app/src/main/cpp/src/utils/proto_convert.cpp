#include "adas/utils/proto_convert.h"

namespace adas {

LaneKeepOutput laneKeepFromProto(const ai::flow::adas::LaneKeepState& p, int64_t timestamp_us)
{
  LaneKeepOutput o;
  o.timestamp_us = timestamp_us;
  o.steer_rad = p.steer_rad();
  o.steer_norm = p.steer_norm();
  o.lookahead_m = p.lookahead_m();
  o.target_x = p.target_x();
  o.target_y = p.target_y();
  o.has_target = p.has_target();
  o.curvature = p.curvature();
  o.status = p.status();
  // Published as "ok:mpc" / "ok:pp" (see LaneKeep::publishLaneKeep).
  const auto colon = o.status.find(':');
  if (colon != std::string::npos) {
    o.controller = o.status.substr(colon + 1);
    o.status = o.status.substr(0, colon);
  } else {
    o.controller = "pp";
  }
  if (o.controller == "mpc" || o.controller == "fp") {
    o.cte_m = o.target_x;
    o.epsi_rad = o.target_y;
  }
  return o;
}

LocalizationPose localizationFromProto(const ai::flow::adas::LocalizationPose& p, int64_t timestamp_us)
{
  LocalizationPose o;
  o.timestamp_us = timestamp_us;
  o.x = p.x();
  o.y = p.y();
  o.yaw = p.yaw();
  o.v = p.v();
  o.yaw_rate = p.yaw_rate();
  o.odom_x = p.odom_x();
  o.odom_y = p.odom_y();
  o.ekf_x = p.ekf_x();
  o.ekf_y = p.ekf_y();
  return o;
}

CameraCalibrationState cameraCalibFromProto(const ai::flow::adas::CameraCalibrationState& p, int64_t timestamp_us)
{
  CameraCalibrationState o;
  o.timestamp_us = timestamp_us;
  o.roll_deg = p.roll_deg();
  o.pitch_deg = p.pitch_deg();
  o.yaw_deg = p.yaw_deg();
  o.camera_height_m = p.camera_height_m();
  o.fx = p.fx();
  o.fy = p.fy();
  o.cx = p.cx();
  o.cy = p.cy();
  o.calibration_success = p.calibration_success();
  o.n_updates = p.n_updates();
  o.vp_u = p.vp_u();
  o.vp_v = p.vp_v();
  o.has_vp = p.has_vp();
  return o;
}

}  // namespace adas
