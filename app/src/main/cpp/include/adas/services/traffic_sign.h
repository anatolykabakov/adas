#pragma once

#include "adas/traffic/traffic_state.hpp"

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
/** Assesses traffic-sign detections against the car's state. */
class TrafficSign : public middleware::Service {
public:
  struct Config {
    double overspeed_margin_kmh = 5.0;  ///< Tolerated excess over the posted limit before it counts as speeding [km/h].
    int64_t speed_limit_hold_ms = 60000;  ///< How long a seen limit stays in force without being seen again [ms].
    int64_t tfl_hold_ms = 1500;           ///< How long a traffic-light state is held after the last detection [ms].
    double min_tfl_conf = 0.35;           ///< Detection confidence needed for a traffic light.
    double min_sign_conf = 0.40;          ///< Detection confidence needed for a sign.
    double steer_ratio = 15.7;            ///< Steering ratio, used to tell which road a sign belongs to.
  };

  /// Constructs with the default config.
  TrafficSign() : TrafficSign(Config{}) {}
  /// \param[in] config Confidence gates and hold times.
  explicit TrafficSign(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "traffic_sign"; }
  /// \return The config in force.
  const Config& config() const { return config_; }

private:
  void onDets(const adas::proto::TrafficDetections& payload);
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
