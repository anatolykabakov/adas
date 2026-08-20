#include "adas/adas_app.h"

#include "adas/utils/logger.h"

AdasApp::AdasApp(int usb_fd, std::string dbc_path, Config cfg)
  : mode_(Mode::RealTime), running_(false), cfg_(std::move(cfg))
{
  cfg_.panda.usb_fd = usb_fd;
  cfg_.panda.dbc_path = std::move(dbc_path);
}

AdasApp::AdasApp(Mode mode, double wheelbase, double pitch0_deg, double yaw0_deg, double camera_height,
                 int camera_calib_history_len, double gps_noise_pos, double gps_update_interval, bool topic_convert)
  : mode_(mode), running_(false), cfg_(Config::forSimulated(wheelbase, pitch0_deg, yaw0_deg, camera_height))
{
  cfg_.camera_calib.history_len = camera_calib_history_len;
  cfg_.localization.gps_noise_pos = gps_noise_pos;
  cfg_.localization.gps_update_interval = gps_update_interval;
  (void)topic_convert;  // kept in the signature for the Python callers; the convert path is gone
  if (mode_ != Mode::Simulated)
    return;
  setupServices();
}

AdasApp::~AdasApp() { stop(); }

bool AdasApp::start()
{
  if (running_) {
    LOGI("AdasApp already running");
    return true;
  }

  try {
    if (mode_ == Mode::RealTime) {
      setupServices();
      size_t started = middleware_->startAll();
      LOGI("Started %zu realtime services", started);
    } else {
      LOGI("AdasApp Simulated ready (%zu services)", middleware_->getServiceCount());
    }
    running_ = true;
  } catch (const std::exception& e) {
    LOGE("Exception in AdasApp::start(): %s", e.what());
  }
  return running_;
}

void AdasApp::stop()
{
  if (!running_)
    return;

  LOGI("Stopping AdasApp...");
  running_ = false;

  if (middleware_ && mode_ == Mode::RealTime) {
    size_t stopped = middleware_->stopAll();
    LOGI("Stopped %zu services", stopped);
    middleware_->printStats();
  }

  LOGI("AdasApp stopped");
}

void AdasApp::setupServices()
{
  // One list instead of two. The realtime and the simulated app used to build their service sets in
  // separate functions, and the sets drifted: the simulated one once ran without a SafetyWarn config
  // and nothing said so. Here every service states, on one line, in which modes it exists — a service
  // added for one mode is now a visible decision about the other, not an omission.
  const bool rt = (mode_ == Mode::RealTime);
  const auto& f = cfg_.feature_flags;

  middleware_ = std::make_shared<adas::middleware::Manager>(rt ? adas::middleware::Manager::Mode::RealTime :
                                                                 adas::middleware::Manager::Mode::Simulated);

  // The bus driver exists only where a bus does. The control law also runs in replay: the scripts
  // feed it state and read its commands, which is the whole point of an offline run.
  const bool have_car = rt && cfg_.panda.usb_fd != -1 && f.enable_panda;
  if (have_car)
    platform_service_ = middleware_->registerService<adas::services::Platform>(adas::services::Platform::Config{
        cfg_.panda.usb_fd, cfg_.panda.dbc_path, cfg_.vehicle_name, cfg_.panda.cruise_buttons_enabled,
        cfg_.panda.cruise_tip_cooldown_ms, cfg_.panda.speed_filter});
  if (have_car || !rt)
    control_service_ = middleware_->registerService<adas::services::Control>(controlConfig());

  if (rt && f.enable_zmq_bridge)
    zmq_bridge_service_ = middleware_->registerService<adas::services::ZmqBridge>(cfg_.zmq_bridge);

  // The simulated app ignores the lane-keep/localization/camera-calib flags on purpose: a replay
  // without them answers nothing, so turning them off there would only manufacture confusion.
  if (!rt || f.enable_lane_keep) {
    auto lk = cfg_.lane_keep;
    if (rt)  // long_plan is wired to its flag only on the car; a replay keeps what the config says.
      lk.long_plan_enabled = f.enable_long_plan;
    lane_keep_service_ = middleware_->registerService<adas::services::Planner>(lk);
    LOGI("adas::services::Planner controller=%s max_steer=%.1f° ratio=%.1f max_tq=%.0f pp=%.2f/[%.1f,%.1f] "
         "pid=%.2f/%.2f/%.5f",
         lk.controller.c_str(), lk.max_steer_deg, lk.steer_ratio, lk.max_torque_cnm, lk.pp_k_dd, lk.pp_ld_min,
         lk.pp_ld_max, lk.pid_kp, lk.pid_ki, lk.pid_kf);
  }

  if (f.enable_safety_warn)
    safety_warn_service_ = middleware_->registerService<adas::services::SafetyWarn>(cfg_.safety_warn);

  if (rt && f.enable_traffic_sign)
    traffic_sign_service_ = middleware_->registerService<adas::services::TrafficSign>(cfg_.traffic_sign);

  if (!rt || f.enable_localization)
    localization_service_ = middleware_->registerService<adas::services::Localization>(cfg_.localization);

  if (!rt || f.enable_camera_calib)
    camera_calib_service_ = middleware_->registerService<adas::services::CameraCalib>(cfg_.camera_calib);

  if (rt)
    middleware_->registerService<adas::services::MiddlewareStats>();

  if (!rt)
    internal_subscriber_ = middleware_->registerService<adas::services::InternalSubscriber>();

  LOGI("%s services ready (%zu)", rt ? "Realtime" : "Simulated", middleware_->getServiceCount());
}

adas::services::Control::Config AdasApp::controlConfig() const
{
  const auto& lk = cfg_.lane_keep;
  adas::services::Control::Config c;
  c.cruise_buttons_enabled = cfg_.panda.cruise_buttons_enabled;
  c.cruise_deadband_ms = cfg_.panda.cruise_deadband_ms;
  c.cruise_tip_step_ms = cfg_.panda.cruise_tip_step_ms;

  c.ctl = {lk.pid_kp,      lk.pid_ki,     lk.pid_kf,        lk.pid_ff_floor_mps,     100.0,
           lk.steer_ratio, lk.steer_sign, lk.max_steer_deg, lk.tire_stiffness_factor};
  c.slew = {lk.steer_slew_limit_deg, lk.mpc_max_lateral_jerk, lk.mpc_rate_min_speed,
            lk.mpc_Lf > 1e-3 ? lk.mpc_Lf : lk.wheelbase_m, 10};
  c.wheelbase_m = lk.wheelbase_m;
  c.lf_m = lk.mpc_Lf > 1e-3 ? lk.mpc_Lf : lk.wheelbase_m;
  c.max_torque_cnm = lk.max_torque_cnm;
  c.lane_max_age_s = lk.lane_max_age_s;
  c.lka_blinker_resume_delay_s = lk.lka_blinker_resume_delay_s;
  c.assist_max_age_s = lk.assist_max_age_s;
  return c;
}

void AdasApp::resetLocalization(double x, double y, double yaw, double v, double yaw_rate)
{
  if (localization_service_)
    localization_service_->resetPose(x, y, yaw, v, yaw_rate);
}

void AdasApp::setCameraIntrinsics(double fx, double fy, double cx, double cy)
{
  if (camera_calib_service_)
    camera_calib_service_->setIntrinsics(fx, fy, cx, cy);
}

void AdasApp::setCameraEstimate(double pitch_deg, double yaw_deg)
{
  if (camera_calib_service_)
    camera_calib_service_->setEstimate(pitch_deg, yaw_deg);
}

void AdasApp::setCameraHeight(double height_m)
{
  if (camera_calib_service_)
    camera_calib_service_->setHeight(height_m);
}

bool AdasApp::reseatPanda(int usb_fd)
{
  if (!platform_service_) {
    LOGW("reseatPanda(fd=%d): no panda service in this app — nothing to seat it in", usb_fd);
    return false;
  }
  platform_service_->reseatPanda(usb_fd);
  return true;
}

bool AdasApp::setParam(const std::string& name, const std::string& value)
{
  if (!middleware_)
    return false;
  const size_t applied = middleware_->setParameter(name, value);
  if (applied == 0)
    LOGW("setParam: unknown knob '%s' (value '%s')", name.c_str(), value.c_str());
  return applied > 0;
}

bool AdasApp::setParam(const std::string& name, double value) { return setParam(name, std::to_string(value)); }

bool AdasApp::setParam(const std::string& name, bool value)
{
  return setParam(name, std::string(value ? "true" : "false"));
}

std::string AdasApp::getParam(const std::string& name) const
{
  return middleware_ ? middleware_->getParameter(name) : std::string{};
}

size_t AdasApp::updateParams(const std::map<std::string, std::string>& params)
{
  size_t applied = 0;
  for (const auto& [name, value] : params) {
    if (setParam(name, value))
      ++applied;
  }
  if (applied != params.size())
    LOGW("updateParams: applied %zu of %zu", applied, params.size());
  return applied;
}

void AdasApp::step(uint64_t timestamp_us)
{
  if (!middleware_ || mode_ != Mode::Simulated)
    return;
  middleware_->setTime(timestamp_us);
  middleware_->step();
}

std::vector<adas::HostOutMsg> AdasApp::popMessages()
{
  if (!internal_subscriber_)
    return {};
  return internal_subscriber_->popMessages();
}

void AdasApp::resetCameraCalib()
{
  if (camera_calib_service_)
    camera_calib_service_->reset();
}
