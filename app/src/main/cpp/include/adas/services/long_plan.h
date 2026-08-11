#pragma once

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/long_planner.hpp"

namespace adas {

namespace services {

class LongPlan : public middleware::Service {
public:
  /** The rules live in `utils/long_planner.hpp` so they can be tested without a middleware
   *  instance; this service only collects inputs, calls them, and publishes. */
  using Config = longplan::Config;

  LongPlan() : LongPlan(Config{}) {}
  explicit LongPlan(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "long_plan"; }
  const Config& config() const { return config_; }

private:
  void onModelLong(const adas::proto::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void onPath(const LanePathMsg& msg);
  void tick();

  Config config_;
  ChassisSample chassis_{};
  bool have_chassis_ = false;
  adas::proto::ModelLongPlan model_{};
  bool have_model_ = false;
  std::vector<Vec2> path_{};
  bool have_path_ = false;
};

}  // namespace services

}  // namespace adas
