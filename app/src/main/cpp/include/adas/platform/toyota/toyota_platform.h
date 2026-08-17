#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "adas/platform/car_platform.h"
#include "adas/platform/toyota/values.h"
#include "adas/utils/can_parser.h"
#include "adas/utils/speed_filter.h"

namespace adas {
namespace platform {
namespace toyota {

/**
 * \brief Toyota TSS2 behind the brand-neutral interface.
 *
 * \details A second make exists to prove the seam holds, and this one does: it shares no code with
 * Volkswagen, no file outside `platform/toyota/` changed to add it, and the two differ in every detail
 * that matters — a torque ceiling of 1500 against 300, a steering message every frame against every
 * second one, a checksum over the address rather than a CRC table, cruise state from `PCM_CRUISE`
 * rather than a TSK status byte.
 *
 * \warning **Decoding is implemented; sending has never been tried on a car.** The frame layout and
 * checksum follow `opendbc` and `selfdrive/car/toyota/toyotacan.py`, but nothing here has been
 * validated against real hardware, and a steering message the EPS rejects is the good outcome — the
 * bad one is a message it accepts and misreads. Drive it as a passenger and check the decoded state
 * against the cluster before letting it write anything; `sendingAllowed()` is false until then.
 */
class ToyotaTss2 : public CarPlatform {
public:
  /**
   * \param[in] dbc_path DBC to parse — `toyota_nodsu_pt_generated.dbc` from opendbc.
   * \param[in] speed_filter Wheel-speed filter settings.
   */
  ToyotaTss2(std::string dbc_path, const adas::SpeedFilter::Config& speed_filter);

  const char* name() const override { return "toyota_tss2"; }
  const char* dbcAssetName() const override { return "toyota_nodsu_pt_generated.dbc"; }
  void init() override;
  VehicleDefaults defaults() const override;

  bool isAllowedRxAddress(uint32_t address) const override;
  bool update(const adas::proto::CANData& msg, int64_t now_ms) override;
  const adas::proto::CarState& carState() const override { return state_; }
  CarStateView stateView() const override;

  std::vector<can_frame> apply(const CarControl& cc) override;
  void setCruiseIntent(int intent, int64_t now_ms) override;
  SteerLimits steerLimits() const override;
  void setLastCommand(int torque_cnm, bool lat_active) override;

  void configureSafety() override;
  std::optional<health_t> safetyTick(::Panda& panda, health_t health, int64_t now_ms) override;
  bool ignition() const override { return ignition_; }
  bool safetyModelOk() const override { return safety_mode_ == SafetyConstants::kToyota; }
  bool lateralActuationAllowed() const override;
  void enterSafeMode(::Panda& panda) override;

  /**
   * \brief Whether this port may put frames on the bus at all.
   *
   * \details False, and deliberately hardcoded: the send path has never been checked against a real
   * EPS. Reading the bus is safe and useful on its own — it is how the decode gets validated — so the
   * port is usable today for everything except actuation.
   */
  static constexpr bool sendingAllowed() { return false; }

private:
  bool decodeFrame(const can_frame& frame);
  /// `(length + address bytes + payload bytes) & 0xFF`, as `opendbc/can/common.cc::toyota_checksum`.
  static uint8_t checksum(uint32_t address, const uint8_t* data, size_t len);

  std::string dbc_path_;
  std::unique_ptr<DBSParser> dbc_;
  adas::SpeedFilter speed_filter_;
  adas::proto::CarState state_;
  bool dirty_ = false;

  bool ignition_ = false;
  uint16_t safety_mode_ = SafetyConstants::kNoOutput;
  bool controls_allowed_ = false;
  bool safety_configured_ = false;

  double eps_torque_ = 0.0;  ///< Driver torque from `EPS_STATUS`, in the EPS's own units.
  int last_steer_ = 0;       ///< Last torque sent, for the rate limit.
  int log_torque_cnm_ = 0;
  bool log_lat_active_ = false;
};

}  // namespace toyota
}  // namespace platform
}  // namespace adas
