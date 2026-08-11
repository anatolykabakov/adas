#pragma once

#include "adas/traffic/traffic_state.hpp"

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"

namespace adas {

namespace services {

class TrafficSign : public middleware::Service {
public:
  struct Config {
    double overspeed_margin_kmh = 5.0;
    int64_t speed_limit_hold_ms = 60000;
    int64_t tfl_hold_ms = 1500;
    double min_tfl_conf = 0.35;
    double min_sign_conf = 0.40;
  };

  TrafficSign() : TrafficSign(Config{}) {}
  explicit TrafficSign(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "traffic_sign"; }
  const Config& config() const { return config_; }

private:
  void onDets(const adas::proto::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void tick();

  Config config_;
  ChassisSample chassis_{};
  bool have_chassis_ = false;

  void publishTraffic(int64_t now_ms);

  traffic::State state_;
};

}  // namespace services

}  // namespace adas
