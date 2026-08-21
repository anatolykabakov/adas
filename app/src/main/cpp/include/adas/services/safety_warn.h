#pragma once

#include "adas/utils/lane_path.h"

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/proto_convert.h"
#include "adas/utils/path_lateral_state.h"
#include "adas/utils/safety_planner.h"

namespace adas {
namespace services {
/** Forward-collision and lane-departure warnings, latched for the HUD. */
class SafetyWarn : public middleware::Service {
public:
  struct Config {
    safety::SafetyPlannerConfig planner{};  ///< Thresholds the warnings are computed from.
    int warn_set_frames = 3;                ///< Frames a warning must hold before it is raised.
    int warn_hold_frames = 10;              ///< Frames it stays on after the cause is gone, so the driver can see it.

    LanePathConfig lane_path{};  ///< How lane lines become a path, for the lateral part of the warning.
    double steer_ratio = 15.7;   ///< Steering ratio, for turning the wheel angle into a road-wheel angle.
  };

  /// Constructs with the default config.
  SafetyWarn() : SafetyWarn(Config{}) {}
  /// \param[in] config Thresholds and latch times.
  explicit SafetyWarn(Config config) : config_(std::move(config)) { rebuildLatches(); }

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "safety_warn"; }
  /// \return The config in force.
  const Config& config() const { return config_; }

private:
  LaneFusionState lane_fusion_{};
  void onPath(const LanePathMsg& msg);
  void onModelLong(const adas::proto::ModelLongPlan& payload);
  void onChassis(const ChassisSample& msg);
  void onSteer(const adas::proto::SteerCommand& payload);
  void tick();
  /// Chassis, lateral state and driver intent as the planner's input. Everything the warning
  /// thresholds are evaluated against, and nothing about the warnings themselves.
  safety::PlannerInput buildPlannerInput() const;
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
  adas::proto::ModelLongPlan model_{};
  bool have_model_ = false;
  int64_t lat_active_ts_us_ = 0;

  safety::WarningLatch fcw_latch_{};
  safety::WarningLatch aeb_latch_{};
  safety::WarningLatch lldw_latch_{};
  safety::WarningLatch rldw_latch_{};
};

}  // namespace services

}  // namespace adas
