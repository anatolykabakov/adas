#pragma once

#include <cstdint>
#include <string>

#include "camera_calib.pb.h"
#include "adas/panda/can_frame.h"
#include "adas/panda/health.h"
#include <algorithm>
#include <vector>

#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/road_route.h"
#include "adas/services/planner.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/lat_control_pid.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"
#include "adas/utils/long_planner.hpp"
#include "adas/utils/safety_planner.hpp"
#include "messages.pb.h"
#include "lane_keep.pb.h"
#include "steer.pb.h"
#include "localization.pb.h"
#include "adas/traffic/traffic_state.hpp"
#include "adas/utils/adas_topics.h"

namespace utils {
adas::proto::CANData createCANMessage(const std::vector<can_frame>& frames, int64_t now_ms);
/** \brief The panda's health, outbound.
 *
 *  \param[in] ignition Sticky and owned by the supervisor, not derivable from the raw packet.
 *  \param[in] lat_actuation_allowed Depends on TSK and EPS, which only the decoder knows.
 *
 *  Both are required arguments rather than defaulted fields: the controller gates on the second one, and
 *  forgetting it means driving a whole session at zero torque. That already happened on
 *  2026_08_13_09_10_19. */
adas::proto::PandaHealth createHealthMessage(const health_t& health, int64_t now_ms, bool ignition,
                                             bool lat_actuation_allowed);
adas::proto::CarState createCarStateMessage(const adas::proto::CarState& state);

}  // namespace utils

namespace adas {
/// Domain -> schema. The payload is returned, not the envelope: ZMQMessage exists only in the
/// bridge, so services exchange schema messages, which are the ones visible in a recorded drive.
adas::proto::LaneKeepState createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm);
adas::proto::LaneKeepDebug createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                               double frame_dt_s, const services::Planner::Config& config);

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
  // `cam_y_left_m` is registered by the Planner: it holds the second copy of that number and must update
  // both, and two owners of one parameter name would be a duplicate in the registry.
}

/** \brief Planner output to plan message. Curvature, not an angle: the angle is derived by the
 *  controller, on its own tick and at its own rate. */
/** Everything a steering command carries besides the controller's own numbers.
 *
 *  A struct rather than a parameter list because of the four timestamps: they come from four different
 *  clocks — frame capture, inference, the chassis frame the command was closed on, and publication —
 *  and the bag is the only place the latency chain can be reconstructed afterwards. As positional
 *  arguments they are four adjacent int64s that swap silently.
 */
struct SteerCommandInputs {
  int torque_cnm = 0;
  bool enabled = false;
  int64_t capture_ts_ms = 0;
  int64_t vision_ts_ms = 0;
  int64_t chassis_ts_ms = 0;
  int64_t publish_ts_ms = 0;
  bool slew_clipped = false;
  bool assist_allowed = false;
  bool assist_known = false;
  std::string status;
  int cruise_intent = 0;
  /// The HUD lane pictograms. Hardcoded true until the vision status is wired through; see task #40.
  bool hud_left_lane_visible = true;
  bool hud_right_lane_visible = true;
};

/// Steering command for the platform, plus the PID internals a drive is debugged with.
adas::proto::SteerCommand createSteerCommand(const SteerCommandInputs& in, const LatControlPid::Result& lat);

adas::proto::LatPlan createLatPlan(const LaneKeepOutput& out, double command_curvature, double frame_dt_s,
                                   const char* kappa_solver);

/** Path-side diagnostics and the vision timestamps from the lane path onto the lane-keep record.
 *
 *  `capture_ts_us` falls back to the message stamp: without it the latency chain in the bag starts at
 *  zero and every downstream age reads as the whole uptime. */
void applyLanePath(LaneKeepOutput& out, const LanePathMsg& msg);

/** What was actually commanded, read back from the last steering command.
 *
 *  The controller is a separate service, so the record would otherwise hold the request and never the
 *  actuation — and a drive is debugged on the difference between the two. */
void applySteerFeedback(LaneKeepOutput& out, const adas::proto::SteerCommand& cmd);

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

/// Road centrelines within `radius_m` of (x, y), appended to a built `MapLocalState`.
/// Separate from `createMapLocal` because only this part needs the map, and the service decides
/// whether the pose is trustworthy enough to ask for it at all.
void fillLocalMap(adas::proto::MapLocalState& out, const mapmatch::RoadMap& map, double x, double y, double radius_m);

}  // namespace adas
