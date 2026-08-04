#pragma once

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "utils/adas_topics.h"
#include "utils/path_lateral_state.h"
#include "utils/safety_planner.hpp"

namespace adas {

class SafetyWarnService : public Service {
public:
  struct Config {
    safety::SafetyPlannerConfig planner{};
    int warn_set_frames = 3;
    int warn_hold_frames = 10;
  };

  SafetyWarnService() : SafetyWarnService(Config{}) {}
  explicit SafetyWarnService(Config config) : config_(std::move(config)) { rebuildLatches(); }

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "safety_warn"; }
  const Config& config() const { return config_; }

private:
  void onPath(const LanePathMsg& msg);
  void onModelLong(const ai::flow::adas::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void tick();
  void rebuildLatches();

  Config config_;
  ChassisSample chassis_{};
  bool have_chassis_ = false;
  PathLateralState lateral_{};
  bool have_lateral_ = false;
  double cte_rate_ms_ = 0.0;
  bool lane_anchored_ = false;
  double last_cte_m_ = 0.0;
  int64_t last_path_ts_us_ = 0;
  ai::flow::adas::ModelLongPlan model_{};
  bool have_model_ = false;

  safety::WarningLatch fcw_latch_{};
  safety::WarningLatch aeb_latch_{};
  safety::WarningLatch lldw_latch_{};
  safety::WarningLatch rldw_latch_{};
};

}  // namespace adas
