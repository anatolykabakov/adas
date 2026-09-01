#include "adas/services/internal_subscriber.h"

#include "messages.pb.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
void InternalSubscriber::configure()
{
  subscribe<adas::proto::LaneKeepState>(topics::kLaneKeep, [this](const adas::proto::LaneKeepState& m) {
    out_.emplace_back(laneKeepFromProto(m, m.timestamp() * 1000));
  });
  subscribe<adas::proto::LocalizationPose>(topics::kLocalizationPose, [this](const adas::proto::LocalizationPose& m) {
    out_.emplace_back(localizationFromProto(m, m.timestamp() * 1000));
  });
  subscribe<adas::proto::CameraCalibrationState>(topics::kCameraCalib,
                                                 [this](const adas::proto::CameraCalibrationState& m) {
                                                   out_.emplace_back(cameraCalibFromProto(m, m.timestamp() * 1000));
                                                 });
  subscribe<adas::proto::SteerCommand>(topics::kSteerCommand,
                                       [this](const adas::proto::SteerCommand& m) { out_.emplace_back(m); });
  subscribe<adas::proto::SafetyWarnState>(topics::kSafetyWarn,
                                          [this](const adas::proto::SafetyWarnState& m) { out_.emplace_back(m); });
  subscribe<adas::proto::LongPlanState>(topics::kLongPlan,
                                        [this](const adas::proto::LongPlanState& m) { out_.emplace_back(m); });
}

void InternalSubscriber::reset() { out_.clear(); }

std::vector<HostOutMsg> InternalSubscriber::popMessages()
{
  std::vector<HostOutMsg> out;
  out.reserve(out_.size());
  while (!out_.empty()) {
    out.push_back(std::move(out_.front()));
    out_.pop_front();
  }
  return out;
}

}  // namespace services
}  // namespace adas
