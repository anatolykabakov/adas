#pragma once

#include <memory>
#include <string>
#include <vector>
#include <zmq.hpp>

#include "middleware/middleware.hpp"
#include "messages.pb.h"

inline constexpr const char* kZmqEndpointIn = "tcp://127.0.0.1:5555";

inline constexpr const char* kZmqEndpointOut = "tcp://127.0.0.1:5556";

inline const std::vector<std::string> kZmqOutboundTopics = {
    "can/rx",
    "panda/health",
    "vehicle/state",
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
};

class ZmqBridgeService : public adas::Service {
public:
  struct Config {
    std::string endpoint_in = kZmqEndpointIn;
    std::string endpoint_out = kZmqEndpointOut;
  };

  ZmqBridgeService() : ZmqBridgeService(Config{}) {}
  explicit ZmqBridgeService(Config config);

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "zmq_bridge"; }
  const Config& config() const { return config_; }

private:
  void initSockets();
  void zmqPollTimerCallback();
  void processInbound();
  void onInternalMessage(const std::string& topic_name, const ai::flow::adas::ZMQMessage& msg);

  Config config_;
  std::string endpoint_in_ = kZmqEndpointIn;
  std::string endpoint_out_ = kZmqEndpointOut;

  std::unique_ptr<zmq::context_t> zmq_context_;
  std::unique_ptr<zmq::socket_t> sub_in_;
  std::unique_ptr<zmq::socket_t> pub_out_;
  std::vector<zmq::pollitem_t> poll_items_;
};
