#pragma once

#include <cstdint>

#include "camera_calib.pb.h"
#include "adas/utils/long_planner.hpp"
#include "messages.pb.h"
#include "lane_keep.pb.h"
#include "localization.pb.h"
#include "adas/traffic/traffic_state.hpp"
#include "adas/services/camera_calib.h"
#include "adas/services/lane_keep.h"
#include "adas/utils/adas_topics.h"

namespace adas {

adas::proto::ZMQMessage createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm);
adas::proto::ZMQMessage createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                            double frame_dt_s, const services::LaneKeep::Config& config);
adas::proto::ZMQMessage createSteerCommand(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                           bool steer_output_enabled, bool have_desired);

adas::proto::ZMQMessage createLongPlan(const longplan::Input& in, const longplan::Plan& plan, int64_t now_ms);

adas::proto::ZMQMessage createCameraCalibState(const CameraCalibrationState& state, int64_t timestamp_us);
adas::proto::ZMQMessage createCameraCalibDebug(const CameraCalibrationState& state, const PoseCalibrator& pose,
                                               const VanishingPointCalibrator& vp, int64_t timestamp_us,
                                               const char* source);

adas::proto::ZMQMessage createTrafficVision(const traffic::State& state, const traffic::Assessment& a, int64_t now_ms);

LaneKeepOutput laneKeepFromProto(const adas::proto::LaneKeepState& p, int64_t timestamp_us);
LocalizationPose localizationFromProto(const adas::proto::LocalizationPose& p, int64_t timestamp_us);
CameraCalibrationState cameraCalibFromProto(const adas::proto::CameraCalibrationState& p, int64_t timestamp_us);

}  // namespace adas
