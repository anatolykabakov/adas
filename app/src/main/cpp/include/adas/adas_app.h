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
#include "adas/services/safety_warn.h"
#include "adas/services/traffic_sign.h"
#include "adas/services/middleware_stats.h"
#include "adas/services/platform.h"
#include "adas/services/control.h"
#include "adas/services/car_config.h"
#include "adas/services/zmq_bridge.h"

/** Builds the middleware and the services; the only entry point. */
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

    /// \return A config tuned for offline replay: no hardware, geometry from the arguments.
    static Config forSimulated(double wheelbase_m = 2.636, double pitch_deg = 0.0, double yaw_deg = 0.0,
                               double camera_height_m = 1.22);

    /// Load from JSON. \param[out] ok False when the file is missing or does not parse.
    static Config loadFromFile(const std::string& path, bool* ok = nullptr);
  };

  /// The real-time app: a panda descriptor, a CAN database, and the loaded config. This is what the
  /// phone constructs.
  AdasApp(int usb_fd, std::string dbc_path, Config cfg);

  /// The simulated app, constructed by the Python bindings for offline replay and the simulator.
  AdasApp(Mode mode, double wheelbase = 2.636, double pitch0_deg = 0.0, double yaw0_deg = 0.0,
          double camera_height = 1.22, int camera_calib_history_len = 50, double gps_noise_pos = 0.5,
          double gps_update_interval = 0.2, bool topic_convert = false);

  ~AdasApp();

  /// Starts every service. \return False when a service failed to come up; the app is then unusable.
  bool start();
  /// Stops the services and puts the panda back into a safe mode. Safe to call twice.
  void stop();

  /**
   * \brief Seat a re-opened panda descriptor without restarting; only the panda driver is touched.
   * \param[in] usb_fd Descriptor of an already opened panda.
   * \return False when there is no panda service — the caller should restart instead.
   */
  bool reseatPanda(int usb_fd);

  /// \return The config in force.
  const Config& config() const { return cfg_; }

  /// \return The bus, for tests and offline tools.
  std::shared_ptr<adas::middleware::Manager> getMiddleware() { return middleware_; }
  /// \return RealTime or Simulated.
  Mode mode() const { return mode_; }

  // Data feeds and knob setters used to live here as one wrapper per topic and per knob. The
  // Python bindings now publish typed messages straight onto the bus (getMiddleware()) and set knobs
  // through the parameter registry (setParam); only actions with no registry equivalent remain.

  /// Advance the virtual clock and run whatever is due (Simulated only); without it nothing publishes.
  void step(uint64_t timestamp_us);

  /// \return Everything the services produced for the host since the last call, in arrival order.
  std::vector<adas::HostOutMsg> popMessages();

  /// Discards the learned camera mounting and starts the estimate over.
  void resetCameraCalib();
  /// Reseed the pose: east/north [m], ENU heading [rad], speed [m/s], yaw rate [rad/s].
  void resetLocalization(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0);
  /// Set a runtime knob by name. \return False when no service owns the name.
  bool setParam(const std::string& name, const std::string& value);
  /// Numeric overload.
  bool setParam(const std::string& name, double value);
  /// Boolean overload.
  bool setParam(const std::string& name, bool value);
  /// \return The knob's current value, empty when unknown.
  std::string getParam(const std::string& name) const;
  /// \param[in] params Knobs to apply in one go. \return How many were accepted.
  size_t updateParams(const std::map<std::string, std::string>& params);

  /// Intrinsics [px] of the frames being delivered — see CameraCalib::setIntrinsics on frame size.
  void setCameraIntrinsics(double fx, double fy, double cx, double cy);
  /// Seed the mount estimate [deg].
  void setCameraEstimate(double pitch_deg, double yaw_deg);
  /// Set the camera height [m].
  void setCameraHeight(double height_m);

  /// Direct access to the services, for tests and offline tools. Null when the mode did not start one;
  /// on a running app the returned service is being driven from its own thread.
  adas::services::Planner* laneKeep() { return lane_keep_service_.get(); }
  /// \return The pose estimator, or null when not started.
  adas::services::Localization* localization() { return localization_service_.get(); }
  /// \return The camera calibration, or null when not started.
  adas::services::CameraCalib* cameraCalib() { return camera_calib_service_.get(); }
  /// \return The host-side sink (Simulated mode only).
  adas::services::InternalSubscriber& subscriber() { return *internal_subscriber_; }

private:
  /// Builds the service set for the current mode; each service says in which modes it exists.
  void setupServices();
  adas::services::Control::Config controlConfig() const;

  Mode mode_ = Mode::RealTime;
  std::atomic<bool> running_{false};
  Config cfg_;

  std::shared_ptr<adas::middleware::Manager> middleware_;

  std::shared_ptr<adas::services::Platform> platform_service_;
  std::shared_ptr<adas::services::Control> control_service_;
  std::shared_ptr<adas::services::ZmqBridge> zmq_bridge_service_;

  std::shared_ptr<adas::services::Planner> lane_keep_service_;
  std::shared_ptr<adas::services::SafetyWarn> safety_warn_service_;
  std::shared_ptr<adas::services::TrafficSign> traffic_sign_service_;
  std::shared_ptr<adas::services::Localization> localization_service_;
  std::shared_ptr<adas::services::CameraCalib> camera_calib_service_;
  std::shared_ptr<adas::services::InternalSubscriber> internal_subscriber_;
};
