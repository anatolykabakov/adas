#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/panda/panda.h"
#include "adas/utils/can_parser.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/utils/speed_filter.h"
#include "adas/platform/volkswagen/mqb_car_state_decoder.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"

namespace adas {
namespace services {
class Panda : public adas::middleware::Service {
public:
  struct Config {
    int usb_fd = -1;
    std::string dbc_path;
    bool cruise_buttons_enabled = false;
    double cruise_deadband_ms = 0.70;
    int cruise_tip_cooldown_ms = 200;
    double cruise_tip_step_ms = 1.0 / 3.6;
    bool lat_always_on = false;
    adas::SpeedFilter::Config speed_filter{};
  };

  explicit Panda(Config config);
  ~Panda();

  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "panda"; }
  const Config& config() const { return config_; }

private:
  void pandaRxCallback();
  void pandaStateCallback();
  void carControllerCallback();
  void steerCommandCallback(const adas::proto::SteerCommand& cmd);
  void longPlanCallback(const adas::proto::LongPlanState& lp);
  void initializePanda();
  void publishCarState(int64_t now_ms);
  volkswagen::CruiseButtonCmd computeCruiseButtons(const volkswagen::CarStateView& cs);
  bool assistAllowed(const volkswagen::CarStateView& cs) const;

  Config config_;
  int usb_fd_;
  std::string dbc_path_;
  std::shared_ptr<::Panda> panda_;
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
  double cruise_v_set_ceiling_ = 0.0;
  int64_t cruise_cooldown_until_ms_ = 0;
  bool cruise_hold_tip_up_ = false;
  bool cruise_hold_tip_down_ = false;
  uint8_t cruise_gra_cnt_at_arm_ = 0xFF;
};

}  // namespace services

}  // namespace adas
