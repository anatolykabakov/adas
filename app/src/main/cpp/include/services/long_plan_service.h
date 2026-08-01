#pragma once

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "utils/adas_topics.h"
#include "utils/curvature_preview.h"

namespace adas {

class LongPlanService : public Service {
public:
  struct Config {
    double lead_prob_thresh = 0.5;
    double t_follow = 1.5;
    double min_gap_m = 4.0;
    double a_max = 1.2;
    double a_min = -2.5;
    double kp_gap = 0.35;
    double kp_v = 0.4;

    bool curv_enabled = true;
    double curv_a_lat_max = 1.8;
    double curv_preview_s = 4.0;
    double curv_min_speed_ms = 8.0;
    double curv_v_floor_ms = 8.0;
  };

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
