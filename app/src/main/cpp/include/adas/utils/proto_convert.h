#pragma once

#include <cstdint>
#include <string>

#include "camera_calib.pb.h"
#include "adas/panda/can_frame.h"
#include "adas/panda/health.h"
#include <algorithm>
#include <vector>

#include "adas/services/planner.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/lat_control_pid.h"
#include "adas/utils/pose_calibrator.h"
#include "adas/utils/vanishing_point_calib.h"
#include "adas/utils/long_planner.h"
#include "adas/utils/safety_planner.h"
#include "messages.pb.h"
#include "lane_keep.pb.h"
#include "steer.pb.h"
#include "localization.pb.h"
#include "adas/traffic/traffic_state.hpp"
#include "adas/utils/adas_topics.h"

namespace utils {
/// \return A CANData message wrapping \p frames, stamped \p now_ms.
adas::proto::CANData createCANMessage(const std::vector<can_frame>& frames, int64_t now_ms);
/** \brief The panda's health, outbound. */
adas::proto::PandaHealth createHealthMessage(const health_t& health, int64_t now_ms, bool ignition,
                                             bool lat_actuation_allowed);
/// \return A copy of \p state re-stamped for publication.
adas::proto::CarState createCarStateMessage(const adas::proto::CarState& state);

}  // namespace utils

namespace adas {
/// Domain -> schema. The payload is returned, not the envelope: ZMQMessage exists only in the
/// bridge, so services exchange schema messages, which are the ones visible in a recorded drive.
adas::proto::LaneKeepState createLaneKeepState(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm);
/// \return The planner's full debug record for the bag.
adas::proto::LaneKeepDebug createLaneKeepDebug(const LaneKeepOutput& out, int64_t now_us, double max_torque_cnm,
                                               double frame_dt_s, const services::Planner::Config& config);

/// \return The longitudinal plan message: target speed, gap and its inputs.
adas::proto::LongPlanState createLongPlan(const longplan::Input& in, const longplan::Plan& plan, int64_t now_ms);

/// \return The calibration message published on calibration/camera.
adas::proto::CameraCalibrationState createCameraCalibState(const CameraCalibrationState& state, int64_t timestamp_us);
/// \return The calibration debug record: block progress and sample verdicts.
adas::proto::CameraCalibDebug createCameraCalibDebug(const CameraCalibrationState& state, const PoseCalibrator& pose,
                                                     const VanishingPointCalibrator& vp, int64_t timestamp_us,
                                                     const char* source);

/// \return The fused traffic state for the HUD.
adas::proto::TrafficVisionState createTrafficVision(const traffic::State& state, const traffic::Assessment& a,
                                                    int64_t now_ms);

/// \return The planner output reconstructed from its message, for replays.
LaneKeepOutput laneKeepFromProto(const adas::proto::LaneKeepState& p, int64_t timestamp_us);
/// \return The pose reconstructed from its message.
LocalizationPose localizationFromProto(const adas::proto::LocalizationPose& p, int64_t timestamp_us);
/// \return The calibration reconstructed from its message.
CameraCalibrationState cameraCalibFromProto(const adas::proto::CameraCalibrationState& p, int64_t timestamp_us);

/// \return The middleware/stats message from a snapshot.
adas::proto::MiddlewareStats createMiddlewareStats(const adas::middleware::MiddlewareSnapshot& snap);

/// \return The localization/pose message.
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

/// \return The safety/warn message: FCW/AEB/LDW flags and their inputs.
adas::proto::SafetyWarnState createSafetyWarn(const safety::PlannerInput& in, const safety::SafetyPlan& plan,
                                              const SafetyWarnFlags& w, int64_t ms);

/// Schema -> domain, at the input of a service.
ChassisSample carStateToChassis(const adas::proto::CarState& cs, double steer_ratio = 15.7);

/// \return The raw IMU sample the calibrator eats.
RawImuSample imuToRaw(const adas::proto::IMUData& imu);

/// \return The camera-odometry sample; valid only when both vectors are complete.
CameraOdometrySample cameraOdometryToSample(const adas::proto::CameraOdometry& odom);

template <typename Reg>
/// Register every LanePathConfig field as a runtime knob through \p reg.
void registerLanePathParameters(LanePathConfig& cfg, Reg&& reg)
{
  reg(
      "path_lane_blend_scale", [&cfg](double v) { cfg.lane_blend_scale = std::clamp(v, 0.0, 1.0); },
      [&cfg] { return cfg.lane_blend_scale; });
  reg("path_camera_offset_m", [&cfg](double v) { cfg.camera_offset_m = v; }, [&cfg] { return cfg.camera_offset_m; });
  reg(
      "center_force_gain", [&cfg](double v) { cfg.center_force_gain = std::max(0.0, v); },
      [&cfg] { return cfg.center_force_gain; });
  reg("lane_std_good_m", [&cfg](double v) { cfg.lane_std_good_m = v; }, [&cfg] { return cfg.lane_std_good_m; });
  reg("lane_std_bad_m", [&cfg](double v) { cfg.lane_std_bad_m = v; }, [&cfg] { return cfg.lane_std_bad_m; });
  reg("lane_std_range_m", [&cfg](double v) { cfg.lane_std_range_m = v; }, [&cfg] { return cfg.lane_std_range_m; });
  // `cam_y_left_m` is registered by the Planner: it holds the second copy of that number and must update
  // both, and two owners of one parameter name would be a duplicate in the registry.
}

/** \brief Planner output to plan message. Curvature, not an angle: the angle is derived by the
 *  controller, on its own tick and at its own rate. */
/** Everything a steering command carries besides the controller's own numbers. */
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

/// \return The control/lat_plan message: the curvature handed to Control.
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

/// \return The vision/path message from the fused path.
adas::proto::LanePath createLanePath(const LanePathMsg& path);
/// \return The fused path reconstructed from its message.
LanePathMsg lanePathFromProto(const adas::proto::LanePath& msg);
/// \return A CarState carrying the chassis sample, for offline feeds.
adas::proto::CarState carStateFromChassis(const ChassisSample& chassis);

/**
 * \brief Lead vehicle from the model's longitudinal output.
 * \param[in] plan The model's longitudinal message.
 * \return Probability, longitudinal and lateral gap [m], and the lead's speed [m/s].
 */
longplan::LeadState leadFromModel(const adas::proto::ModelLongPlan& plan);

}  // namespace adas
