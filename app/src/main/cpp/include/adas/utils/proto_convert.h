#pragma once

#include <cstdint>

#include "camera_calib.pb.h"
#include "lane_keep.pb.h"
#include "localization.pb.h"
#include "adas/services/lane_keep.h"
#include "adas/utils/adas_topics.h"

namespace adas {

LaneKeepOutput laneKeepFromProto(const ai::flow::adas::LaneKeepState& p, int64_t timestamp_us);
LocalizationPose localizationFromProto(const ai::flow::adas::LocalizationPose& p, int64_t timestamp_us);
CameraCalibrationState cameraCalibFromProto(const ai::flow::adas::CameraCalibrationState& p, int64_t timestamp_us);

}  // namespace adas
