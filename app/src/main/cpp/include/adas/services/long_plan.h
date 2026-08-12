#pragma once

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/proto_convert.h"
#include "adas/utils/long_planner.hpp"

namespace adas {
namespace services {
class LongPlan : public middleware::Service {
public:
  using Config = longplan::Config;

  LongPlan() : LongPlan(Config{}) {}
  explicit LongPlan(Config config) : config_(std::move(config)) {}

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "long_plan"; }
  const Config& config() const { return config_; }

private:
  LaneFusionState lane_fusion_{};
  void onModelLong(const adas::proto::ModelLongPlan& payload);
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
