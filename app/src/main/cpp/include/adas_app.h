#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "middleware/middleware.hpp"
#include "services/camera_calib_service.h"
#include "services/imu_calib_service.h"
#include "services/internal_subscriber.h"
#include "services/lane_keep_service.h"
#include "services/localization_service.h"
#include "services/map_data_service.h"
#include "services/long_plan_service.h"
#include "services/safety_warn_service.h"
#include "services/traffic_sign_service.h"
#include "services/middleware_stats_service.h"
#include "services/panda_service.h"
#include "services/topic_convert_service.h"
#include "services/zmq_bridge_service.h"

class AdasApp {
public:
  enum class Mode { RealTime, Simulated };

  struct FeatureFlags {
    bool enable_panda = true;
    bool enable_zmq_bridge = true;
    bool enable_lane_keep = true;
    bool enable_localization = true;
    bool enable_camera_calib = true;
    bool enable_imu_calib = true;
    bool enable_vision_supercombo = true;
    bool enable_long_plan = true;
    bool enable_safety_warn = true;
    bool enable_traffic_sign = true;
    /** On in the shipped `config.json`, off in this default. Not an oversight: this default is what runs when
     *  the config fails to load, and starting a service on a failed load is how you end up debugging a node
     *  nobody asked for. Nothing downstream consumes its output — see `services/map_data_service.h`. */
    bool enable_map_data = false;
  };

  struct Config {
    FeatureFlags feature_flags{};
    std::string vehicle_name = "vw_golf_7_mqb";
    PandaService::Config panda{};
    ZmqBridgeService::Config zmq_bridge{};
    adas::TopicConvertService::Config topic_convert{};
    adas::LaneKeepService::Config lane_keep{};
    adas::LocalizationService::Config localization{};
    adas::CameraCalibService::Config camera_calib{};
    adas::ImuCalibService::Config imu_calib{};
    adas::LongPlanService::Config long_plan{};
    adas::SafetyWarnService::Config safety_warn{};
    adas::TrafficSignService::Config traffic_sign{};
    adas::MapDataService::Config map_data{};

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

  bool start();
  void stop();

  void setConfig(const Config& cfg) { cfg_ = cfg; }
  const Config& config() const { return cfg_; }

  std::shared_ptr<adas::Middleware> getMiddleware() { return middleware_; }
  Mode mode() const { return mode_; }

  void publishChassis(const adas::ChassisSample& chassis);
  void publishLanes(const adas::LanePathMsg& lanes);

  void publishLaneLines(const ai::flow::adas::LaneLines& lanes);
  void publishGps(const adas::GpsSample& gps);
  void publishImu(const adas::ImuSample& imu);
  void publishLaneUv(const adas::LaneUvMsg& uv);

  void step(uint64_t timestamp_us);

  std::vector<adas::HostOutMsg> popMessages();

  void resetCameraCalib();
  void resetLocalization(double x = 0, double y = 0, double yaw = 0, double v = 0, double yaw_rate = 0);
  bool setParam(const std::string& name, const std::string& value);
  bool setParam(const std::string& name, double value);
  bool setParam(const std::string& name, bool value);
  std::string getParam(const std::string& name) const;
  size_t updateParams(const std::map<std::string, std::string>& params);

  void setCameraIntrinsics(double fx, double fy, double cx, double cy);
  void setCameraEstimate(double pitch_deg, double yaw_deg);
  void setCameraHeight(double height_m);
  void setLaneKeepPp(double k_dd, double ld_min, double ld_max, double shift);
  void setLaneKeepPpLdCurvGain(double gain);
  void setLaneKeepMaxSteerDeg(double max_steer_deg);
  void setLaneKeepSteerRatio(double ratio);
  void setLaneKeepSteerSign(double sign);
  void setLaneKeepController(const std::string& controller);
  void setLaneKeepMpcKappaYawBlend(double alpha, double min_speed);
  void setLaneKeepMpcEmaAlphas(double kappa_alpha, double epsi_alpha, double cte_alpha);
  void setLaneKeepSteerSlewLimitDeg(double deg);
  void setLaneKeepVehicleModel(bool on, double tire_stiffness_factor);
  void setLaneKeepFpSteerDelayS(double seconds);
  void setLaneKeepFpSteeringRateWeight(double weight);
  void setLaneKeepCamYLeftM(double m);
  /** Angle PID gains. Needed by `rlog_lat_diff.py`: an open-loop replay drives our integrator with an error
   *  our command cannot influence, so the only way to compare instantaneous controller response against
   *  upstream's logged output is to run ours with `ki = 0`. */
  void setLaneKeepPidGains(double kp, double ki, double kf);
  void setLaneKeepRecomputeSetpoint(bool on);
  void setLaneBlendScale(double scale);

  adas::LaneKeepService* laneKeep() { return lane_keep_service_.get(); }
  adas::LocalizationService* localization() { return localization_service_.get(); }
  adas::MapDataService* mapData() { return map_data_service_.get(); }
  adas::CameraCalibService* cameraCalib() { return camera_calib_service_.get(); }
  adas::ImuCalibService* imuCalib() { return imu_calib_service_.get(); }
  adas::InternalSubscriber& subscriber() { return *internal_subscriber_; }

private:
  void setupRealtimeServices();
  void setupSimulatedServices();

  Mode mode_ = Mode::RealTime;
  std::atomic<bool> running_{false};
  Config cfg_;

  std::shared_ptr<adas::Middleware> middleware_;

  std::shared_ptr<PandaService> panda_service_;
  std::shared_ptr<ZmqBridgeService> zmq_bridge_service_;
  std::shared_ptr<adas::TopicConvertService> topic_convert_service_;
  bool sim_topic_convert_ = false;

  std::shared_ptr<adas::LaneKeepService> lane_keep_service_;
  std::shared_ptr<adas::LongPlanService> long_plan_service_;
  std::shared_ptr<adas::SafetyWarnService> safety_warn_service_;
  std::shared_ptr<adas::TrafficSignService> traffic_sign_service_;
  std::shared_ptr<adas::LocalizationService> localization_service_;
  std::shared_ptr<adas::MapDataService> map_data_service_;
  std::shared_ptr<adas::CameraCalibService> camera_calib_service_;
  std::shared_ptr<adas::ImuCalibService> imu_calib_service_;
  std::shared_ptr<adas::InternalSubscriber> internal_subscriber_;
};
