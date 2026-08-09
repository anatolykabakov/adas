#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "adas/services/camera_calib.h"
#include "adas/services/imu_calib.h"
#include "adas/services/internal_subscriber.h"
#include "adas/services/lane_keep.h"
#include "adas/services/localization.h"
#include "adas/services/map_data.h"
#include "adas/services/long_plan.h"
#include "adas/services/safety_warn.h"
#include "adas/services/traffic_sign.h"
#include "adas/services/middleware_stats.h"
#include "adas/services/panda.h"
#include "adas/services/topic_convert.h"
#include "adas/services/zmq_bridge.h"

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
     *  nobody asked for. Nothing downstream consumes its output — see `services/map_data.h`. */
    bool enable_map_data = false;
  };

  struct Config {
    FeatureFlags feature_flags{};
    std::string vehicle_name = "vw_golf_7_mqb";
    adas::services::Panda::Config panda{};
    adas::services::ZmqBridge::Config zmq_bridge{};
    adas::services::TopicConvert::Config topic_convert{};
    adas::services::LaneKeep::Config lane_keep{};
    adas::services::Localization::Config localization{};
    adas::services::CameraCalib::Config camera_calib{};
    adas::services::ImuCalib::Config imu_calib{};
    adas::services::LongPlan::Config long_plan{};
    adas::services::SafetyWarn::Config safety_warn{};
    adas::services::TrafficSign::Config traffic_sign{};
    adas::services::MapData::Config map_data{};

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

  std::shared_ptr<adas::middleware::Manager> getMiddleware() { return middleware_; }
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

  adas::services::LaneKeep* laneKeep() { return lane_keep_service_.get(); }
  adas::services::Localization* localization() { return localization_service_.get(); }
  adas::services::MapData* mapData() { return map_data_service_.get(); }
  adas::services::CameraCalib* cameraCalib() { return camera_calib_service_.get(); }
  adas::services::ImuCalib* imuCalib() { return imu_calib_service_.get(); }
  adas::services::InternalSubscriber& subscriber() { return *internal_subscriber_; }

private:
  void setupRealtimeServices();
  void setupSimulatedServices();

  Mode mode_ = Mode::RealTime;
  std::atomic<bool> running_{false};
  Config cfg_;

  std::shared_ptr<adas::middleware::Manager> middleware_;

  std::shared_ptr<adas::services::Panda> panda_service_;
  std::shared_ptr<adas::services::ZmqBridge> zmq_bridge_service_;
  std::shared_ptr<adas::services::TopicConvert> topic_convert_service_;
  bool sim_topic_convert_ = false;

  std::shared_ptr<adas::services::LaneKeep> lane_keep_service_;
  std::shared_ptr<adas::services::LongPlan> long_plan_service_;
  std::shared_ptr<adas::services::SafetyWarn> safety_warn_service_;
  std::shared_ptr<adas::services::TrafficSign> traffic_sign_service_;
  std::shared_ptr<adas::services::Localization> localization_service_;
  std::shared_ptr<adas::services::MapData> map_data_service_;
  std::shared_ptr<adas::services::CameraCalib> camera_calib_service_;
  std::shared_ptr<adas::services::ImuCalib> imu_calib_service_;
  std::shared_ptr<adas::services::InternalSubscriber> internal_subscriber_;
};
