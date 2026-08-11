#include "adas/services/internal_subscriber.h"

#include "messages.pb.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {

void InternalSubscriber::configure()
{
  subscribe<adas::proto::ZMQMessage>(topics::kLaneKeep, [this](const adas::proto::ZMQMessage& m) {
    if (!m.has_lane_keep())
      return;
    out_.emplace_back(laneKeepFromProto(m.lane_keep(), m.timestamp() * 1000));
  });
  subscribe<adas::proto::ZMQMessage>(topics::kLocalizationPose, [this](const adas::proto::ZMQMessage& m) {
    if (!m.has_localization_pose())
      return;
    out_.emplace_back(localizationFromProto(m.localization_pose(), m.timestamp() * 1000));
  });
  subscribe<adas::proto::ZMQMessage>(topics::kCameraCalib, [this](const adas::proto::ZMQMessage& m) {
    if (!m.has_camera_calib())
      return;
    out_.emplace_back(cameraCalibFromProto(m.camera_calib(), m.timestamp() * 1000));
  });
  subscribe<adas::proto::ZMQMessage>(topics::kSteerCommand, [this](const adas::proto::ZMQMessage& m) {
    if (!m.has_steer_command())
      return;
    out_.emplace_back(m.steer_command());
  });
  subscribe<adas::proto::ZMQMessage>(topics::kSafetyWarn, [this](const adas::proto::ZMQMessage& m) {
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

}  // namespace services
}  // namespace adas
