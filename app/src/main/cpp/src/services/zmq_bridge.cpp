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
    scheduleTimer(
        10, [this]() { zmqPollTimerCallback(); }, "poll");
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

  forwardOutbound<adas::proto::CANData>(topics::kCanRx,
                                        [](adas::proto::ZMQMessage& zmq, const adas::proto::CANData& payload) {
                                          zmq.set_timestamp(payload.timestamp());
                                          *zmq.mutable_can_data() = payload;
                                        });
  forwardOutbound<adas::proto::PandaHealth>(topics::kPandaHealth,
                                            [](adas::proto::ZMQMessage& zmq, const adas::proto::PandaHealth& payload) {
                                              zmq.set_timestamp(payload.timestamp());
                                              *zmq.mutable_panda_health() = payload;
                                            });
  forwardOutbound<adas::proto::CarState>(topics::kVehicleState,
                                         [](adas::proto::ZMQMessage& zmq, const adas::proto::CarState& payload) {
                                           zmq.set_timestamp(payload.timestamp());
                                           *zmq.mutable_car_state() = payload;
                                         });
  forwardOutbound<adas::proto::LaneKeepState>(
      topics::kLaneKeep, [](adas::proto::ZMQMessage& zmq, const adas::proto::LaneKeepState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_lane_keep() = payload;
      });
  forwardOutbound<adas::proto::LaneKeepDebug>(
      topics::kLaneKeepDebug, [](adas::proto::ZMQMessage& zmq, const adas::proto::LaneKeepDebug& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_lane_keep_debug() = payload;
      });
  forwardOutbound<adas::proto::LocalizationPose>(
      topics::kLocalizationPose, [](adas::proto::ZMQMessage& zmq, const adas::proto::LocalizationPose& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_localization_pose() = payload;
      });
  forwardOutbound<adas::proto::CameraCalibrationState>(
      topics::kCameraCalib, [](adas::proto::ZMQMessage& zmq, const adas::proto::CameraCalibrationState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_camera_calib() = payload;
      });
  forwardOutbound<adas::proto::CameraCalibDebug>(
      topics::kCameraCalibDebug, [](adas::proto::ZMQMessage& zmq, const adas::proto::CameraCalibDebug& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_camera_calib_debug() = payload;
      });
  forwardOutbound<adas::proto::SteerCommand>(
      topics::kSteerCommand, [](adas::proto::ZMQMessage& zmq, const adas::proto::SteerCommand& payload) {
        zmq.set_timestamp(payload.publish_ts_ms());
        *zmq.mutable_steer_command() = payload;
      });
  forwardOutbound<adas::proto::MiddlewareStats>(
      topics::kMiddlewareStats, [](adas::proto::ZMQMessage& zmq, const adas::proto::MiddlewareStats& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_middleware_stats() = payload;
      });
  forwardOutbound<adas::proto::LongPlanState>(
      topics::kLongPlan, [](adas::proto::ZMQMessage& zmq, const adas::proto::LongPlanState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_long_plan() = payload;
      });
  forwardOutbound<adas::proto::SafetyWarnState>(
      topics::kSafetyWarn, [](adas::proto::ZMQMessage& zmq, const adas::proto::SafetyWarnState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_safety_warn() = payload;
      });
  forwardOutbound<adas::proto::TrafficVisionState>(
      topics::kTrafficVision, [](adas::proto::ZMQMessage& zmq, const adas::proto::TrafficVisionState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_traffic_vision() = payload;
      });
  forwardOutbound<adas::proto::LanePath>(topics::kVisionPath,
                                         [](adas::proto::ZMQMessage& zmq, const adas::proto::LanePath& payload) {
                                           zmq.set_timestamp(payload.timestamp());
                                           *zmq.mutable_lane_path() = payload;
                                         });
  forwardOutbound<adas::proto::MapLocalState>(
      topics::kMapLocal, [](adas::proto::ZMQMessage& zmq, const adas::proto::MapLocalState& payload) {
        zmq.set_timestamp(payload.timestamp());
        *zmq.mutable_map_local() = payload;
      });
}

void ZmqBridge::zmqPollTimerCallback()
{
  if (poll_items_.empty())
    return;
  const auto n = zmq::poll(poll_items_.data(), poll_items_.size(), std::chrono::milliseconds(1));
  if (n > 0 && (poll_items_[0].revents & ZMQ_POLLIN)) {
    processInbound();
  }
}

void ZmqBridge::processInbound()
{
  zmq::message_t frame0;
  auto r0 = sub_in_->recv(frame0, zmq::recv_flags::dontwait);
  if (!r0)
    return;

  std::string topic;
  std::string payload;

  if (frame0.more()) {
    topic.assign(static_cast<const char*>(frame0.data()), frame0.size());
    zmq::message_t frame1;
    auto r1 = sub_in_->recv(frame1, zmq::recv_flags::dontwait);
    if (!r1) {
      LOGE("Inbound multipart missing payload for topic '%s'", topic.c_str());
      return;
    }
    payload.assign(static_cast<const char*>(frame1.data()), frame1.size());
  } else {
    payload.assign(static_cast<const char*>(frame0.data()), frame0.size());
  }

  adas::proto::ZMQMessage message;
  if (!message.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
    LOGE("Failed to deserialize inbound ZMQ payload (%zu bytes)", payload.size());
    return;
  }

  if (topic.empty()) {
    topic = message.topic();
  } else if (message.topic().empty()) {
    message.set_topic(topic);
  }

  if (topic.empty()) {
    LOGE("Inbound ZMQ message has empty topic");
    return;
  }

  LOGD("ZMQ inbound '%s' (%zu bytes)", topic.c_str(), payload.size());

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
    case Msg::kMapLocal:
      publish(topic, message.map_local());
      break;
    case Msg::kLanePath:
      publish(topic, message.lane_path());
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
