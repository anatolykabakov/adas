#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "adas/platform/car_platform.h"
#include "adas/platform/volkswagen/car_iface.h"
#include "adas/platform/volkswagen/mqb_variants.h"
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
   * \param[in] long_control_enabled Send ACC frames and ask the panda for its longitudinal flag.
   * \param[in] variant Which MQB car: geometry and mass differ, the bus does not.
   */
  VolkswagenMqb(std::string dbc_path, const adas::SpeedFilter::Config& speed_filter, bool long_control_enabled,
                const ::volkswagen::MqbVariant& variant);

  const char* name() const override { return variant_.name; }
  const char* dbcAssetName() const override { return "vw_mqb_2010.dbc"; }
  void init() override;
  VehicleDefaults defaults() const override;

  bool isAllowedRxAddress(uint32_t address) const override;
  bool update(const adas::proto::CANData& msg, int64_t now_ms) override;
  const adas::proto::CarState& carState() const override { return ci_.carState(); }
  CarStateView stateView() const override;

  std::vector<can_frame> apply(const CarControl& cc) override;
  SteerLimits steerLimits() const override;
  LongLimits longLimits() const override;
  bool longitudinalActuationAllowed() const override { return ci_.longActuationAllowed(safety_.lastControlsAllowed()); }

  void configureSafety() override;
  void resetPandaState() override { safety_.resetBoardState(); }
  std::optional<health_t> safetyTick(::Panda& panda, health_t health, int64_t now_ms) override;
  bool ignition() const override { return safety_.lastIgnition(); }
  bool safetyModelOk() const override { return ci_.safetyModelOk(safety_.lastSafetyMode()); }
  bool lateralActuationAllowed() const override { return ci_.actuationAllowed(safety_.lastControlsAllowed()); }
  void enterSafeMode(::Panda& panda) override;

  void setLastCommand(int torque_cnm, bool lat_active) override;

private:
  ::volkswagen::CarIface ci_;
  ::volkswagen::PandaSafetySupervisor safety_;
  const ::volkswagen::MqbVariant& variant_;
  bool long_control_enabled_ = false;

  int log_torque_cnm_ = 0;
  bool log_lat_active_ = false;
};

}  // namespace volkswagen
}  // namespace platform
}  // namespace adas
