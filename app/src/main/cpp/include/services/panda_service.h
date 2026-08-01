#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "middleware/middleware.hpp"
#include "messages.pb.h"
#include "panda/panda.h"
#include "utils/can_parser.h"
#include "volkswagen/carcontroller.h"
#include "volkswagen/mqb_car_state_decoder.h"
#include "volkswagen/panda_safety_supervisor.h"

class PandaService : public adas::Service {
public:
  struct Config {
    int usb_fd = -1;
    std::string dbc_path;
    bool cruise_buttons_enabled = false;
    double cruise_deadband_ms = 0.70;
    int cruise_tip_cooldown_ms = 200;
    double cruise_tip_step_ms = 1.0 / 3.6;
  };

  explicit PandaService(Config config);
  ~PandaService();

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "panda"; }
  const Config& config() const { return config_; }

private:
  void pandaRxCallback();
  void pandaStateCallback();
  void carControllerCallback();
  void steerCommandCallback(const ai::flow::adas::ZMQMessage& msg);
  void longPlanCallback(const ai::flow::adas::ZMQMessage& msg);
  void initializePanda();
  void publishCarState();
  volkswagen::CruiseButtonCmd computeCruiseButtons(const volkswagen::CarStateView& cs);

  Config config_;
  int usb_fd_;
  std::string dbc_path_;
  std::shared_ptr<Panda> panda_;
  std::unique_ptr<DBSParser> dbc_;

  volkswagen::MqbCarStateDecoder decoder_;
  volkswagen::PandaSafetySupervisor safety_;
  volkswagen::CarController car_controller_;

  int hca_cmd_steer_ = 0;
  bool hca_cmd_enabled_ = false;
  int64_t hca_cmd_ts_ms_ = 0;

  bool have_long_plan_ = false;
  double long_v_target_ = 0.0;
  int64_t long_plan_ts_ms_ = 0;
  bool cruise_was_engaged_ = false;
  double cruise_v_set_ = 0.0;
  int64_t cruise_cooldown_until_ms_ = 0;
  bool cruise_hold_tip_up_ = false;
  bool cruise_hold_tip_down_ = false;
  uint8_t cruise_gra_cnt_at_arm_ = 0xFF;
};
