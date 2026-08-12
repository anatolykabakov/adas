#pragma once

#include <cstdint>
#include <string>

#include "camera_calib.pb.h"
#include "adas/panda/can_frame.h"
#include "adas/panda/health.h"
#include <algorithm>
#include <vector>

#include "adas/mapmatch/road_route.h"
#include "adas/services/lane_keep.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"
#include "adas/utils/long_planner.hpp"
#include "adas/utils/safety_planner.hpp"
#include "messages.pb.h"
#include "lane_keep.pb.h"
#include "localization.pb.h"
#include "adas/traffic/traffic_state.hpp"
#include "adas/utils/adas_topics.h"

namespace utils {
adas::proto::CANData createCANMessage(const std::vector<can_frame>& frames, int64_t now_ms);
adas::proto::PandaHealth createHealthMessage(const health_t& health, int64_t now_ms);
adas::proto::CarState createCarStateMessage(const adas::proto::CarState& state);

}  // namespace utils

namespace adas {
/// Domain -> schema. The payload is returned, not the envelope: ZMQMessage exists only in the
/// bridge, so services exchange schema messages, which are the ones visible in a recorded drive.
adas::proto::LaneKeepState createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm);
adas::proto::LaneKeepDebug createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                               double frame_dt_s, const services::LaneKeep::Config& config);
adas::proto::SteerCommand createSteerCommand(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                             bool steer_output_enabled, bool have_desired);

adas::proto::LongPlanState createLongPlan(const longplan::Input& in, const longplan::Plan& plan, int64_t now_ms);

adas::proto::CameraCalibrationState createCameraCalibState(const CameraCalibrationState& state, int64_t timestamp_us);
adas::proto::CameraCalibDebug createCameraCalibDebug(const CameraCalibrationState& state, const PoseCalibrator& pose,
                                                     const VanishingPointCalibrator& vp, int64_t timestamp_us,
                                                     const char* source);

adas::proto::TrafficVisionState createTrafficVision(const traffic::State& state, const traffic::Assessment& a,
                                                    int64_t now_ms);

LaneKeepOutput laneKeepFromProto(const adas::proto::LaneKeepState& p, int64_t timestamp_us);
LocalizationPose localizationFromProto(const adas::proto::LocalizationPose& p, int64_t timestamp_us);
CameraCalibrationState cameraCalibFromProto(const adas::proto::CameraCalibrationState& p, int64_t timestamp_us);

adas::proto::MiddlewareStats createMiddlewareStats(const adas::middleware::MiddlewareSnapshot& snap);

adas::proto::LocalizationPose createLocalizationPose(const LocalizationPose& pose, int64_t timestamp_us);

/** What the warner decided this tick, gathered into a struct so the conversion function does not
 *  take nine loose arguments and mix them up. */
struct SafetyWarnFlags {
  double lead_d = 0.0;
  double lead_v = 0.0;
  double lead_prob = 0.0;
  bool has_lead = false;
  bool fcw = false;
  bool aeb = false;
  bool lldw = false;
  bool rldw = false;
  std::string status;
};

adas::proto::SafetyWarnState createSafetyWarn(const safety::PlannerInput& in, const safety::SafetyPlan& plan,
                                              const SafetyWarnFlags& w, int64_t ms);

/// Schema -> domain, at the input of a service.
ChassisSample carStateToChassis(const adas::proto::CarState& cs, double steer_ratio = 15.7);

RawImuSample imuToRaw(const adas::proto::IMUData& imu);

CameraOdometrySample cameraOdometryToSample(const adas::proto::CameraOdometry& odom);

template <typename Reg>
void registerLanePathParameters(LanePathConfig& cfg, Reg&& reg)
{
  reg(
      "path_lane_blend_scale", [&cfg](double v) { cfg.lane_blend_scale = std::clamp(v, 0.0, 1.0); },
      [&cfg] { return cfg.lane_blend_scale; });
  reg(
      "path_camera_offset_m", [&cfg](double v) { cfg.camera_offset_m = v; }, [&cfg] { return cfg.camera_offset_m; });
  reg(
      "center_force_gain", [&cfg](double v) { cfg.center_force_gain = std::max(0.0, v); },
      [&cfg] { return cfg.center_force_gain; });
  reg(
      "lane_std_good_m", [&cfg](double v) { cfg.lane_std_good_m = v; }, [&cfg] { return cfg.lane_std_good_m; });
  reg(
      "lane_std_bad_m", [&cfg](double v) { cfg.lane_std_bad_m = v; }, [&cfg] { return cfg.lane_std_bad_m; });
  reg(
      "lane_std_range_m", [&cfg](double v) { cfg.lane_std_range_m = v; }, [&cfg] { return cfg.lane_std_range_m; });
  // `cam_y_left_m` регистрирует LaneKeep: он держит вторую копию этого числа и обязан обновить обе,
  // а два владельца одного имени параметра — это дубль в реестре.
}

adas::proto::LanePath createLanePath(const LanePathMsg& path);
LanePathMsg lanePathFromProto(const adas::proto::LanePath& msg);
adas::proto::CarState carStateFromChassis(const ChassisSample& chassis);

/// Everything the map service knows when it publishes: exactly what the message is built from.
struct MapLocalInputs {
  int64_t timestamp_us = 0;
  double pose_gps_gap_m = 0.0;

  bool positioned = false;

  double lat = 0.0;
  double lon = 0.0;
  double map_x = 0.0;
  double map_y = 0.0;
  double yaw = 0.0;
  double speed_mps = 0.0;
  double build_ms = 0.0;
  double route_step_m = 0.0;

  const mapmatch::RouteAhead* route = nullptr;
};

adas::proto::MapLocalState createMapLocal(const MapLocalInputs& in);

}  // namespace adas
