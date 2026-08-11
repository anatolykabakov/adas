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

  for (const auto& topic_name : kZmqOutboundTopics) {
    subscribe<adas::proto::ZMQMessage>(
        topic_name, [this, topic_name](const adas::proto::ZMQMessage& msg) { onInternalMessage(topic_name, msg); });
  }
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
  publish(topic, message);
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
