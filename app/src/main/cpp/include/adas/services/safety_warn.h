#pragma once

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/path_lateral_state.h"
#include "adas/utils/safety_planner.hpp"

namespace adas {

namespace services {

class SafetyWarn : public middleware::Service {
public:
  struct Config {
    safety::SafetyPlannerConfig planner{};
    int warn_set_frames = 3;
    int warn_hold_frames = 10;
  };

  SafetyWarn() : SafetyWarn(Config{}) {}
  explicit SafetyWarn(Config config) : config_(std::move(config)) { rebuildLatches(); }

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "safety_warn"; }
  const Config& config() const { return config_; }

private:
  void onPath(const LanePathMsg& msg);
  void onModelLong(const ai::flow::adas::ZMQMessage& msg);
  void onChassis(const ChassisSample& msg);
  void onSteer(const ai::flow::adas::ZMQMessage& msg);
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
  /** Last `controls/steer` with enabled = true; LDW is suppressed while it is fresh. */
  int64_t lat_active_ts_us_ = 0;

  safety::WarningLatch fcw_latch_{};
  safety::WarningLatch aeb_latch_{};
  safety::WarningLatch lldw_latch_{};
  safety::WarningLatch rldw_latch_{};
};

}  // namespace services

}  // namespace adas
