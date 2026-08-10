#pragma once

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/utils/gps_local_projector.h"
#include "adas/utils/topic_convert.h"

namespace adas {

namespace services {

class TopicConvert : public adas::middleware::Service {
public:
  struct Config {
    double steer_ratio = 15.7;

    double path_lane_blend_scale = 0.3;

    double lane_std_good_m = 0.2;
    double lane_std_bad_m = 0.8;
    /** Range the σ summary covers — see `LanePathConfig::lane_std_range_m` for the measurement. */
    double lane_std_range_m = 20.0;

    double path_camera_offset_m = 0.08;

    double lane_width_min_m = 2.6;
    double lane_width_max_m = 4.6;

    double cam_y_left_m = 0.0;

    double center_force_gain = 0.0;
    double center_force_max_m = 0.8;
    double center_force_turn_scale = 0.7;

    bool lane_mode_hysteresis = true;
    double lane_mode_off_prob = 0.3;
    double lane_mode_on_prob = 0.5;
  };

  TopicConvert() : TopicConvert(Config{}) {}
  explicit TopicConvert(Config config);

  void configure() override;
  void reset() override;
  const Config& config() const { return config_; }

  void setLaneBlendScale(double scale);

  void registerParameters();

private:
  void onVisionLanes(const ai::flow::adas::ZMQMessage& msg);
  void onVehicleState(const ai::flow::adas::ZMQMessage& msg);
  void onImu(const ai::flow::adas::ZMQMessage& msg);
  void onGps(const ai::flow::adas::ZMQMessage& msg);
  void onLaneUv(const ai::flow::adas::ZMQMessage& msg);
  void onCameraOdometry(const ai::flow::adas::ZMQMessage& msg);

  Config config_;
  LanePathConfig path_cfg_;
  LaneFusionState fusion_;
  double steer_ratio_ = 15.7;
  GpsLocalProjector gps_proj_;
};

}  // namespace services

}  // namespace adas
