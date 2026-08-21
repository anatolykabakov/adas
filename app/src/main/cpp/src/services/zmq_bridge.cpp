#include "adas/services/zmq_bridge.h"

#include "adas/utils/logger.h"

namespace adas {
namespace services {
ZmqBridge::ZmqBridge(Config config)
  : config_(std::move(config)), endpoint_in_(config_.endpoint_in), endpoint_out_(config_.endpoint_out)
{
}

void ZmqBridge::configure()
{
  LOGI("Configuring ZmqBridge...");

  try {
    zmq_context_ = std::make_unique<zmq::context_t>(1);
    initSockets();
    scheduleTimer(10, [this]() { zmqPollTimerCallback(); }, "poll");
    LOGI("ZMQ bridge ready: SUB bind %s | PUB bind %s (%zu outbound topics)", endpoint_in_.c_str(),
         endpoint_out_.c_str(), kZmqOutboundTopics.size());
  } catch (const std::exception& e) {
    LOGE("Exception in ZmqBridge::configure(): %s", e.what());
    throw;
  }
}

void ZmqBridge::reset() {}

void ZmqBridge::initSockets()
{
  sub_in_ = std::make_unique<zmq::socket_t>(*zmq_context_, ZMQ_SUB);
  sub_in_->bind(endpoint_in_);
  sub_in_->set(zmq::sockopt::subscribe, "");
  sub_in_->set(zmq::sockopt::rcvtimeo, 0);
  poll_items_.clear();
  poll_items_.push_back(zmq::pollitem_t{*sub_in_, 0, ZMQ_POLLIN, 0});
  LOGI("✓ ZMQ SUB bound %s (inbound)", endpoint_in_.c_str());

  pub_out_ = std::make_unique<zmq::socket_t>(*zmq_context_, ZMQ_PUB);
  pub_out_->bind(endpoint_out_);
  LOGI("✓ ZMQ PUB bound %s (outbound)", endpoint_out_.c_str());

  using Zmq = adas::proto::ZMQMessage;
  forwardOutbound<adas::proto::CANData>(topics::kCanRx, &Zmq::mutable_can_data);
  forwardOutbound<adas::proto::PandaHealth>(topics::kPandaHealth, &Zmq::mutable_panda_health);
  forwardOutbound<adas::proto::CarState>(topics::kVehicleState, &Zmq::mutable_car_state);
  forwardOutbound<adas::proto::LatPlan>(topics::kLatPlan, &Zmq::mutable_lat_plan);
  forwardOutbound<adas::proto::LaneKeepState>(topics::kLaneKeep, &Zmq::mutable_lane_keep);
  forwardOutbound<adas::proto::LaneKeepDebug>(topics::kLaneKeepDebug, &Zmq::mutable_lane_keep_debug);
  forwardOutbound<adas::proto::LocalizationPose>(topics::kLocalizationPose, &Zmq::mutable_localization_pose);
  forwardOutbound<adas::proto::CameraCalibrationState>(topics::kCameraCalib, &Zmq::mutable_camera_calib);
  forwardOutbound<adas::proto::CameraCalibDebug>(topics::kCameraCalibDebug, &Zmq::mutable_camera_calib_debug);
  forwardOutbound<adas::proto::SteerCommand>(topics::kSteerCommand, &Zmq::mutable_steer_command);
  forwardOutbound<adas::proto::MiddlewareStats>(topics::kMiddlewareStats, &Zmq::mutable_middleware_stats);
  forwardOutbound<adas::proto::LongPlanState>(topics::kLongPlan, &Zmq::mutable_long_plan);
  forwardOutbound<adas::proto::SafetyWarnState>(topics::kSafetyWarn, &Zmq::mutable_safety_warn);
  forwardOutbound<adas::proto::TrafficVisionState>(topics::kTrafficVision, &Zmq::mutable_traffic_vision);
  forwardOutbound<adas::proto::LanePath>(topics::kVisionPath, &Zmq::mutable_lane_path);
}

void ZmqBridge::zmqPollTimerCallback()
{
  if (poll_items_.empty())
    return;
  const auto n = zmq::poll(poll_items_.data(), poll_items_.size(), std::chrono::milliseconds(1));
  if (n > 0 && (poll_items_[0].revents & ZMQ_POLLIN)) {
    // Drain the socket, do not take one message per tick.
    constexpr int kMaxPerTick = 32;
    for (int i = 0; i < kMaxPerTick; ++i) {
      if (!processInbound())
        break;
    }
  }
}

bool ZmqBridge::processInbound()
{
  std::string topic;
  adas::proto::ZMQMessage message;
  if (!recvEnvelope(topic, message))
    return false;
  publishPayload(topic, message);
  return true;
}

bool ZmqBridge::recvEnvelope(std::string& topic, adas::proto::ZMQMessage& message)
{
  zmq::message_t frame0;
  auto r0 = sub_in_->recv(frame0, zmq::recv_flags::dontwait);
  if (!r0)
    return false;

  std::string payload;
  // Two shapes on the wire: [topic][payload] from publishers that set an envelope prefix, and a bare
  // payload from those that do not. The topic then comes from the message itself.
  if (frame0.more()) {
    topic.assign(static_cast<const char*>(frame0.data()), frame0.size());
    zmq::message_t frame1;
    auto r1 = sub_in_->recv(frame1, zmq::recv_flags::dontwait);
    if (!r1) {
      LOGE("Inbound multipart missing payload for topic '%s'", topic.c_str());
      return false;
    }
    payload.assign(static_cast<const char*>(frame1.data()), frame1.size());
  } else {
    payload.assign(static_cast<const char*>(frame0.data()), frame0.size());
  }

  if (!message.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
    LOGE("Failed to deserialize inbound ZMQ payload (%zu bytes)", payload.size());
    return false;
  }

  if (topic.empty()) {
    topic = message.topic();
  } else if (message.topic().empty()) {
    message.set_topic(topic);
  }

  if (topic.empty()) {
    LOGE("Inbound ZMQ message has empty topic");
    return false;
  }

  LOGD("ZMQ inbound '%s' (%zu bytes)", topic.c_str(), payload.size());
  return true;
}

void ZmqBridge::publishPayload(const std::string& topic, const adas::proto::ZMQMessage& message)
{
  using Msg = adas::proto::ZMQMessage;
  switch (message.payload_case()) {
    case Msg::kCameraImage:
      publish(topic, message.camera_image());
      break;
    case Msg::kGpsLocation:
      publish(topic, message.gps_location());
      break;
    case Msg::kGpsData:
      publish(topic, message.gps_data());
      break;
    case Msg::kImuData:
      publish(topic, message.imu_data());
      break;
    case Msg::kCanData:
      publish(topic, message.can_data());
      break;
    case Msg::kPandaHealth:
      publish(topic, message.panda_health());
      break;
    case Msg::kCameraIntrinsics:
      publish(topic, message.camera_intrinsics());
      break;
    case Msg::kLaneLines:
      publish(topic, message.lane_lines());
      break;
    case Msg::kCarState:
      publish(topic, message.car_state());
      break;
    case Msg::kSteerCommand:
      publish(topic, message.steer_command());
      break;
    case Msg::kLaneKeep:
      publish(topic, message.lane_keep());
      break;
    case Msg::kLocalizationPose:
      publish(topic, message.localization_pose());
      break;
    case Msg::kCameraCalib:
      publish(topic, message.camera_calib());
      break;
    case Msg::kLaneUv:
      publish(topic, message.lane_uv());
      break;
    case Msg::kCameraOdometry:
      publish(topic, message.camera_odometry());
      break;
    case Msg::kMiddlewareStats:
      publish(topic, message.middleware_stats());
      break;
    case Msg::kLaneKeepDebug:
      publish(topic, message.lane_keep_debug());
      break;
    case Msg::kModelLongPlan:
      publish(topic, message.model_long_plan());
      break;
    case Msg::kLongPlan:
      publish(topic, message.long_plan());
      break;
    case Msg::kSafetyWarn:
      publish(topic, message.safety_warn());
      break;
    case Msg::kCameraCalibDebug:
      publish(topic, message.camera_calib_debug());
      break;
    case Msg::kTrafficDetections:
      publish(topic, message.traffic_detections());
      break;
    case Msg::kTrafficVision:
      publish(topic, message.traffic_vision());
      break;
    case Msg::kPhoneStats:
      publish(topic, message.phone_stats());
      break;
      break;
    case Msg::kLanePath:
      publish(topic, message.lane_path());
      break;
    case Msg::kLatPlan:
      publish(topic, message.lat_plan());
      break;
    default:
      LOGW("ZMQ inbound '%s' without payload — dropped", topic.c_str());
      break;
  }
}

void ZmqBridge::onInternalMessage(const std::string& topic_name, const adas::proto::ZMQMessage& msg)
{
  if (!pub_out_)
    return;

  std::string serialized;
  if (msg.topic().empty()) {
    adas::proto::ZMQMessage tmp;
    tmp.CopyFrom(msg);
    tmp.set_topic(topic_name);
    if (!tmp.SerializeToString(&serialized)) {
      LOGE("Failed to serialize outbound '%s'", topic_name.c_str());
      return;
    }
  } else if (!msg.SerializeToString(&serialized)) {
    LOGE("Failed to serialize outbound '%s'", topic_name.c_str());
    return;
  }

  zmq::message_t topic_frame(topic_name.data(), topic_name.size());
  zmq::message_t payload_frame(serialized.data(), serialized.size());
  const bool ok = pub_out_->send(topic_frame, zmq::send_flags::sndmore | zmq::send_flags::dontwait) &&
                  pub_out_->send(payload_frame, zmq::send_flags::dontwait);

  if (!ok) {
    LOGD("ZMQ outbound send dropped for '%s' (no peer / HWM)", topic_name.c_str());
  }
}

}  // namespace services
}  // namespace adas
