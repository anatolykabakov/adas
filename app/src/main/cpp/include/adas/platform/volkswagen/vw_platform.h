#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "adas/platform/car_platform.h"
#include "adas/platform/volkswagen/car_iface.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"

namespace adas {
namespace platform {
namespace volkswagen {
/** Volkswagen MQB behind the brand-neutral interface. */
class VolkswagenMqb : public CarPlatform {
public:
  /**
   * \param[in] dbc_path DBC to parse for decoding.
   * \param[in] speed_filter Wheel-speed filter settings.
   * \param[in] cruise_buttons_enabled Whether the longitudinal axis is driven through the stock buttons.
   * \param[in] cruise_tip_cooldown_ms Minimum gap between button presses [ms].
   */
  VolkswagenMqb(std::string dbc_path, const adas::SpeedFilter::Config& speed_filter, bool cruise_buttons_enabled,
                int cruise_tip_cooldown_ms);

  const char* name() const override { return "vw_golf_7_mqb"; }
  const char* dbcAssetName() const override { return "vw_mqb_2010.dbc"; }
  void init() override;
  VehicleDefaults defaults() const override;

  bool isAllowedRxAddress(uint32_t address) const override;
  bool update(const adas::proto::CANData& msg, int64_t now_ms) override;
  const adas::proto::CarState& carState() const override { return ci_.carState(); }
  CarStateView stateView() const override;

  std::vector<can_frame> apply(const CarControl& cc) override;
  void setCruiseIntent(int intent, int64_t now_ms) override;
  SteerLimits steerLimits() const override;

  void configureSafety() override;
  void resetPandaState() override { safety_.resetBoardState(); }
  std::optional<health_t> safetyTick(::Panda& panda, health_t health, int64_t now_ms) override;
  bool ignition() const override { return safety_.lastIgnition(); }
  bool safetyModelOk() const override { return ci_.safetyModelOk(safety_.lastSafetyMode()); }
  bool lateralActuationAllowed() const override { return ci_.actuationAllowed(safety_.lastControlsAllowed()); }
  void enterSafeMode(::Panda& panda) override;

  void setLastCommand(int torque_cnm, bool lat_active) override;

private:
  /// GRA counter handshake: hold the button until the stock counter advances, then pause.
  ::volkswagen::CruiseButtonCmd cruiseButtons(const ::volkswagen::CarStateView& cs, int64_t now_ms);

  ::volkswagen::CarIface ci_;
  ::volkswagen::PandaSafetySupervisor safety_;

  bool cruise_buttons_enabled_ = false;
  int cruise_tip_cooldown_ms_ = 200;
  int cruise_intent_ = 0;
  int64_t cruise_intent_at_ms_ = 0;
  int64_t cruise_cooldown_until_ms_ = 0;
  bool cruise_hold_tip_up_ = false;
  bool cruise_hold_tip_down_ = false;
  uint8_t cruise_gra_cnt_at_arm_ = 0xFF;

  int log_torque_cnm_ = 0;
  bool log_lat_active_ = false;
};

}  // namespace volkswagen
}  // namespace platform
}  // namespace adas
