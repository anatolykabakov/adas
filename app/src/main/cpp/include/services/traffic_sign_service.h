#pragma once

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "utils/adas_topics.h"

namespace adas {

class TrafficSignService : public Service {
public:
  struct Config {
    double overspeed_margin_kmh = 5.0;
    int64_t speed_limit_hold_ms = 60000;
    int64_t tfl_hold_ms = 1500;
    double min_tfl_conf = 0.35;
    double min_sign_conf = 0.40;
  };

  TrafficSignService() : TrafficSignService(Config{}) {}
  explicit TrafficSignService(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "traffic_sign"; }
  const Config& config() const { return config_; }

private:
  void onDets(const ai::flow::adas::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void tick();
  void publishState(int64_t now_ms);

  Config config_;
  ChassisSample chassis_{};
  bool have_chassis_ = false;

  int speed_limit_kmh_ = 0;
  std::string speed_limit_label_;
  int64_t speed_limit_ts_ms_ = 0;

  ai::flow::adas::TrafficLightColor tfl_color_ = ai::flow::adas::TFL_UNKNOWN;
  float tfl_conf_ = 0.f;
  int64_t tfl_ts_ms_ = 0;
  int n_dets_ = 0;
  std::string status_ = "init";
};

}  // namespace adas
