#include "services/internal_subscriber.h"

#include "messages.pb.h"
#include "utils/proto_convert.h"

namespace adas {

void InternalSubscriber::configure()
{
  subscribe<ai::flow::adas::ZMQMessage>(topics::kLaneKeep, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_lane_keep())
      return;
    out_.emplace_back(laneKeepFromProto(m.lane_keep(), m.timestamp() * 1000));
  });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kLocalizationPose, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_localization_pose())
      return;
    out_.emplace_back(localizationFromProto(m.localization_pose(), m.timestamp() * 1000));
  });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kCameraCalib, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_camera_calib())
      return;
    out_.emplace_back(cameraCalibFromProto(m.camera_calib(), m.timestamp() * 1000));
  });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kSteerCommand, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_steer_command())
      return;
    out_.emplace_back(m.steer_command());
  });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kSafetyWarn, [this](const ai::flow::adas::ZMQMessage& m) {
    if (!m.has_safety_warn())
      return;
    out_.emplace_back(m.safety_warn());
  });
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

}  // namespace adas
