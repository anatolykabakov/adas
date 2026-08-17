#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "adas/utils/proto_convert.h"
#include "adas/services/camera_calib.h"
#include "adas/services/internal_subscriber.h"
#include "adas/services/planner.h"
#include "adas/services/localization.h"
#include "adas/services/map_data.h"
#include "adas/services/safety_warn.h"
#include "adas/services/traffic_sign.h"
#include "adas/services/middleware_stats.h"
#include "adas/services/platform.h"
#include "adas/services/control.h"
#include "adas/services/car_config.h"
#include "adas/services/zmq_bridge.h"

/**
 * \brief The application: builds the middleware, starts the services, and is the only entry point.
 *
 * \details One class serves three callers — the phone application, the offline harness and the unit
 * tests — because they differ in one thing only: `Mode::RealTime` gives services threads and a wall
 * clock, `Mode::Simulated` advances a virtual clock on `step()`. Everything above that is identical, so a
 * bug found in a replay is a bug in the code that drives the car.
 *
 * The host talks to it through `publish*` and `popMessages`, never through the bus directly.
 */
class AdasApp {
public:
  enum class Mode { RealTime, Simulated };

  struct FeatureFlags {
    bool enable_panda = true;              ///< Start the panda driver. Off leaves the app without a car.
    bool enable_zmq_bridge = true;         ///< Start the bridge to the outside world.
    bool enable_lane_keep = true;          ///< Start the lateral planner.
    bool enable_localization = true;       ///< Start the pose estimator.
    bool enable_camera_calib = true;       ///< Start the camera-mount calibration.
    bool enable_vision_supercombo = true;  ///< Expect model output; off means no lane lines arrive.
    bool enable_long_plan = true;          ///< Start the longitudinal planner.
    bool enable_safety_warn = true;        ///< Start the warning service.
    bool enable_traffic_sign = true;       ///< Start sign and traffic-light assessment.
    bool enable_map_data = true;           ///< Start map matching; off when no map file is available.
  };

  struct Config {
    FeatureFlags feature_flags{};  ///< Which services to start at all.
    /// Car platform to build. An unknown name is an error: guessing the car guesses the CAN layout.
    std::string vehicle_name = "vw_golf_7_mqb";
    adas::services::CarConfig panda{};
    adas::services::ZmqBridge::Config zmq_bridge{};       ///< Bridge endpoints.
    adas::services::Planner::Config lane_keep{};          ///< Lateral planner settings.
    adas::services::Localization::Config localization{};  ///< Pose estimator settings.
    adas::services::CameraCalib::Config camera_calib{};   ///< Camera calibration settings.
    adas::services::SafetyWarn::Config safety_warn{};     ///< Warning service settings.
    adas::services::TrafficSign::Config traffic_sign{};   ///< Sign assessment settings.
    adas::services::MapData::Config map_data{};           ///< Map matching settings.

    static Config forSimulated(double wheelbase_m = 2.636, double pitch_deg = 0.0, double yaw_deg = 0.0,
                               double camera_height_m = 1.22);

    static Config loadFromFile(const std::string& path, bool* ok = nullptr);
  };

  AdasApp();
  explicit AdasApp(Config cfg);
  explicit AdasApp(int usb_fd);
  AdasApp(int usb_fd, std::string dbc_path);
  AdasApp(int usb_fd, std::string dbc_path, Config cfg);

  AdasApp(Mode mode, double wheelbase = 2.636, double pitch0_deg = 0.0, double yaw0_deg = 0.0,
          double camera_height = 1.22, int camera_calib_history_len = 50, double gps_noise_pos = 0.5,
          double gps_update_interval = 0.2, bool topic_convert = false);

  ~AdasApp();

  /// Starts every service. \return False when a service failed to come up; the app is then unusable.
  bool start();
  /// Stops the services and puts the panda back into a safe mode. Safe to call twice.
  void stop();

  void setConfig(const Config& cfg) { cfg_ = cfg; }
  const Config& config() const { return cfg_; }

  std::shared_ptr<adas::middleware::Manager> getMiddleware() { return middleware_; }
  Mode mode() const { return mode_; }

  /**
   * \brief Feed the car's state in. In the car this comes from decoded CAN; offline, from the harness.
   *
   * \param[in] chassis Speed, steering angle, yaw rate, blinkers, and whether the wheel is being held.
   * `steer_rad` must be filled: the filter gates its gyro on agreement with the bicycle-model yaw rate,
   * and a zero steering angle pins that estimate to zero and rejects every real turn.
   */
  void publishChassis(const adas::ChassisSample& chassis);
  /// \param[in] lanes Reference path in the ego frame plus the lane-line diagnostics behind it.
  void publishLanes(const adas::LanePathMsg& lanes);

  /// \param[in] path The same reference path in schema form, as a bag or a replay carries it.
  void publishLanePath(const adas::proto::LanePath& path);
  /// \param[in] lanes Raw model output in metres; the path is built from it inside.
  void publishLaneLines(const adas::proto::LaneLines& lanes);
  /**
   * \brief A GNSS fix already projected into the local plane.
   *
   * \param[in] gps Position in metres, course, velocity, and `accuracy_m` — without the accuracy the
   * filter cannot weight the fix and an offline run measures a different pipeline than the car drives.
   */
  void publishGps(const adas::GpsSample& gps);
  /// \param[in] imu Yaw rate already resolved into the car's frame [rad/s]; see \ref publishImuData for raw.
  void publishImu(const adas::ImuSample& imu);
  /**
   * \brief Raw three-axis IMU, as the phone reports it.
   *
   * \param[in] imu Accelerometer, gyro and magnetometer. This path reaches the mount calibrator, so it
   * also yields the road bank — \ref publishImu carries a finished yaw rate and cannot.
   */
  void publishImuData(const adas::proto::IMUData& imu);
  /// \param[in] gps A fix in latitude and longitude; the localization service projects it itself.
  void publishGpsLocation(const adas::proto::GPSLocation& gps);
  /// \param[in] uv Lane lines in image pixels, for the vanishing-point calibration.
  void publishLaneUv(const adas::LaneUvMsg& uv);

  /**
   * \brief Advance the virtual clock and run whatever is due. Simulated mode only.
   *
   * \param[in] timestamp_us New clock value [us]; must not go backwards.
   *
   * \note Without calling this, a simulated app publishes nothing at all — the services never run.
   */
  void step(uint64_t timestamp_us);

  /// \return Everything the services produced for the host since the last call, in arrival order.
  std::vector<adas::HostOutMsg> popMessages();

  /// Discards the learned camera mounting and starts the estimate over.
  void resetCameraCalib();
  /**
   * \brief Reseed the pose.
   *
   * \param[in] x East in the local plane [m].
   * \param[in] y North in the local plane [m].
   * \param[in] yaw Heading [rad], ENU convention: 0 is east, positive counter-clockwise.
   * \param[in] v Speed [m/s].
   * \param[in] yaw_rate Yaw rate [rad/s].
   */
  void resetLocalization(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0);
  /**
   * \brief Set a runtime knob by name, on a moving car.
   *
   * \param[in] name Knob registered by some service; unknown names are logged and ignored.
   * \param[in] value Parsed by the owning service according to the knob's type.
   * \return False when no service owns the name.
   */
  bool setParam(const std::string& name, const std::string& value);
  bool setParam(const std::string& name, double value);
  bool setParam(const std::string& name, bool value);
  std::string getParam(const std::string& name) const;
  /// \param[in] params Knobs to apply in one go. \return How many were accepted.
  size_t updateParams(const std::map<std::string, std::string>& params);

  /// Intrinsics [px] of the frames being delivered — see CameraCalib::setIntrinsics on frame size.
  void setCameraIntrinsics(double fx, double fy, double cx, double cy);
  void setCameraEstimate(double pitch_deg, double yaw_deg);
  void setCameraHeight(double height_m);
  /// Pure-pursuit geometry: lookahead per speed [s], its clamps [m], and a lateral offset [m].
  void setLaneKeepPp(double k_dd, double ld_min, double ld_max, double shift);
  void setLaneKeepPpLdCurvGain(double gain);
  void setLaneKeepMaxSteerDeg(double max_steer_deg);
  void setLaneKeepSteerRatio(double ratio);
  /// \param[in] sign +1 or -1: which way a positive command turns the wheel on this car.
  void setLaneKeepSteerSign(double sign);
  void setLaneKeepController(const std::string& controller);
  void setLaneKeepMpcKappaYawBlend(double alpha, double min_speed);
  void setLaneKeepMpcEmaAlphas(double kappa_alpha, double epsi_alpha, double cte_alpha);
  void setLaneKeepSteerSlewLimitDeg(double deg);
  void setLaneKeepTireStiffness(double tire_stiffness_factor);
  void setLaneKeepVehicleModel(bool use_vehicle_model, double tire_stiffness_factor);
  void setLaneKeepFpSteerDelayS(double seconds);
  void setLaneKeepFpSteeringRateWeight(double weight);
  void setLaneKeepCamYLeftM(double m);
  void setLaneKeepPidGains(double kp, double ki, double kf);
  /// \param[in] scale How much the lane centre pulls the path against the model plan; 0 disables it.
  void setLaneBlendScale(double scale);

  /// Direct access to the services, for tests and offline tools. Null when the mode did not start one;
  /// on a running app the returned service is being driven from its own thread.
  adas::services::Planner* laneKeep() { return lane_keep_service_.get(); }
  adas::services::Localization* localization() { return localization_service_.get(); }
  adas::services::MapData* mapData() { return map_data_service_.get(); }
  adas::services::CameraCalib* cameraCalib() { return camera_calib_service_.get(); }
  adas::services::InternalSubscriber& subscriber() { return *internal_subscriber_; }

private:
  void setupRealtimeServices();
  adas::services::Control::Config controlConfig() const;
  void setupSimulatedServices();

  Mode mode_ = Mode::RealTime;
  std::atomic<bool> running_{false};
  Config cfg_;

  std::shared_ptr<adas::middleware::Manager> middleware_;

  std::shared_ptr<adas::services::Platform> platform_service_;
  std::shared_ptr<adas::services::Control> control_service_;
  std::shared_ptr<adas::services::ZmqBridge> zmq_bridge_service_;
  adas::LanePathConfig inbound_path_cfg_{};
  adas::LaneFusionState inbound_fusion_{};
  bool sim_topic_convert_ = false;

  std::shared_ptr<adas::services::Planner> lane_keep_service_;
  std::shared_ptr<adas::services::SafetyWarn> safety_warn_service_;
  std::shared_ptr<adas::services::TrafficSign> traffic_sign_service_;
  std::shared_ptr<adas::services::Localization> localization_service_;
  std::shared_ptr<adas::services::MapData> map_data_service_;
  std::shared_ptr<adas::services::CameraCalib> camera_calib_service_;
  std::shared_ptr<adas::services::InternalSubscriber> internal_subscriber_;
};
