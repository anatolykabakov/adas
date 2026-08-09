#pragma once

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "utils/adas_topics.h"
#include "utils/long_planner.hpp"

namespace adas {

class LongPlanService : public Service {
public:
  /** The rules live in `utils/long_planner.hpp` so they can be tested without a middleware
   *  instance; this service only collects inputs, calls them, and publishes. */
  using Config = longplan::Config;

  LongPlanService() : LongPlanService(Config{}) {}
  explicit LongPlanService(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "long_plan"; }
  const Config& config() const { return config_; }

private:
  void onModelLong(const ai::flow::adas::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void onPath(const LanePathMsg& msg);
  void tick();

  Config config_;
  ChassisSample chassis_{};
  bool have_chassis_ = false;
  ai::flow::adas::ModelLongPlan model_{};
  bool have_model_ = false;
  std::vector<Vec2> path_{};
  bool have_path_ = false;
};

}  // namespace adas
