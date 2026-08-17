#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <zmq.hpp>

#include "adas/middleware/manager.hpp"
#include "adas/utils/proto_convert.h"
#include "messages.pb.h"

namespace adas {
inline constexpr const char* kZmqEndpointIn = "tcp://127.0.0.1:5555";

inline constexpr const char* kZmqEndpointOut = "tcp://127.0.0.1:5556";

inline const std::vector<std::string> kZmqOutboundTopics = {
    "can/rx",
    "panda/health",
    "vehicle/state",
    "control/lat_plan",
    "control/lane_keep",
    "control/lane_keep_debug",
    "localization/pose",
    "calibration/camera",
    "calibration/camera_debug",
    "controls/steer",
    "middleware/stats",
    "control/long_plan",
    "safety/warn",
    "traffic/state",
    "map/local",
    "vision/path",
};

namespace services {
/**
 * \brief The only place the internal bus meets the outside world.
 *
 * \details Inbound: ZMQ frames are parsed into schema messages and published on the internal bus.
 * Outbound: schema messages are wrapped into a `ZMQMessage` envelope and pushed out. The envelope exists
 * nowhere else — services exchange schema messages, which are the ones a recorded drive contains.
 *
 * This is what lets the phone application, the offline harness and a bag replay all speak to the same
 * services without any of them knowing about a socket.
 */
class ZmqBridge : public adas::middleware::Service {
public:
  struct Config {
    std::string endpoint_in = kZmqEndpointIn;    ///< Where inbound messages are received from.
    std::string endpoint_out = kZmqEndpointOut;  ///< Where outbound messages are published to.
  };

  ZmqBridge() : ZmqBridge(Config{}) {}
  explicit ZmqBridge(Config config);

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "zmq_bridge"; }
  const Config& config() const { return config_; }

protected:
  /** Timestamp the bridge stamps on the envelope.
   *
   *  Every schema message carries `timestamp`, except `SteerCommand`, which has four clocks and would
   *  make `timestamp` ambiguous — the envelope wants the moment it left us, so `publish_ts_ms`. Handled
   *  by overload rather than at the call site: an exception spelled out sixteen times is an exception
   *  that gets copied wrong.
   */
  template <typename Payload>
  static int64_t envelopeTimestamp(const Payload& payload)
  {
    return payload.timestamp();
  }
  static int64_t envelopeTimestamp(const adas::proto::SteerCommand& payload) { return payload.publish_ts_ms(); }

  /** Forward a schema message to the outside, wrapped in the envelope field it belongs to.
   *
   *  `field` is the generated `mutable_*` accessor, so the payload type and the oneof slot are named
   *  once each and cannot drift apart. */
  template <typename Payload>
  void forwardOutbound(const std::string& topic_name, Payload* (adas::proto::ZMQMessage::*field)())
  {
    forwardOutbound<Payload>(topic_name, [field](adas::proto::ZMQMessage& zmq, const Payload& payload) {
      zmq.set_timestamp(envelopeTimestamp(payload));
      *(zmq.*field)() = payload;
    });
  }

  template <typename Payload, typename Setter>
  void forwardOutbound(const std::string& topic_name, Setter setter)
  {
    subscribe<Payload>(topic_name, [this, topic_name, setter](const Payload& payload) {
      adas::proto::ZMQMessage zmq;
      zmq.set_topic(topic_name);
      setter(zmq, payload);
      onInternalMessage(topic_name, zmq);
    });
  }

private:
  void initSockets();
  void zmqPollTimerCallback();
  /// One inbound envelope: frames → parsed message with its topic resolved. False means nothing to do.
  bool recvEnvelope(std::string& topic, adas::proto::ZMQMessage& message);
  /// Envelope → the payload it carries, on the internal bus.
  void publishPayload(const std::string& topic, const adas::proto::ZMQMessage& message);
  void processInbound();
  void onInternalMessage(const std::string& topic_name, const adas::proto::ZMQMessage& msg);

  Config config_;
  std::string endpoint_in_ = kZmqEndpointIn;
  std::string endpoint_out_ = kZmqEndpointOut;

  std::unique_ptr<zmq::context_t> zmq_context_;
  std::unique_ptr<zmq::socket_t> sub_in_;
  std::unique_ptr<zmq::socket_t> pub_out_;
  std::vector<zmq::pollitem_t> poll_items_;
};

}  // namespace services

}  // namespace adas
