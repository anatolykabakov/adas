#pragma once

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "utils/gps_local_projector.h"
#include "utils/topic_convert.h"

namespace adas {

class TopicConvertService : public adas::Service {
public:
  struct Config {
    double steer_ratio = 15.7;

    double path_lane_blend_scale = 0.3;

    double lane_std_good_m = 0.2;
    double lane_std_bad_m = 0.8;

    double path_camera_offset_m = 0.08;

    double lane_width_min_m = 2.6;
    double lane_width_max_m = 4.6;

    double cam_y_left_m = 0.0;

    double center_force_gain = 0.0;
    double center_force_max_m = 0.8;
    double center_force_turn_scale = 0.7;
  };

  TopicConvertService() : TopicConvertService(Config{}) {}
  explicit TopicConvertService(Config config);

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

}  // namespace adas
