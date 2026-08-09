#pragma once

#include <cstdint>
#include <optional>

#include "panda/health.h"
#include "panda/panda.h"

namespace volkswagen {

struct MqbSafetyConstants {
  static constexpr uint16_t kVolkswagen = 15;
  static constexpr uint16_t kNoOutput = 19;
  static constexpr uint16_t kParamStock = 0;
  static constexpr uint16_t kAltExpDisableDisengageOnGas = 1;
  /** Always-on lateral. Bit 16 is `ALKA` in dragonpilot's panda and `ALT_EXP_ALLOW_AEB` in the flowpilot
   *  board source we have locally — not a contradiction but two panda generations, health packet 11 against
   *  16. On the firmware this car runs it is ALKA, proven from dragonpilot's own recordings rather than from
   *  sources it does not ship: they send `alternative_experience = 17`, `safety_tx_blocked` never
   *  incremented once across every route, and `latActive` was true with `controls_allowed` false in 64.3 %
   *  of frames with real torque applied in 96.3 % of those (median 53 cNm). */
  static constexpr uint16_t kAltExpAlka = 16;
  static constexpr uint32_t kIgnVoltageOnMv = 11500;
  static constexpr uint32_t kIgnVoltageOffMv = 10500;
  static constexpr int64_t kIgnOffDebounceMs = 3000;
  static constexpr int64_t kSafetyRetryBaseMs = 1000;
  static constexpr int64_t kSafetyRetryMaxMs = 10000;
};

struct SafetyLogContext {
  int last_tsk_status = -1;
  bool cruise_engaged = false;
  bool brake_pressed = false;
  bool gas_pressed = false;
  uint8_t eps_hca_status = 0;
  int apply_steer_last = 0;
  int hca_cmd_steer = 0;
  bool lat_cmd_active = false;
  bool ldw_valid = false;
};

class PandaSafetySupervisor {
public:
  using C = MqbSafetyConstants;

  /** What to send the panda. Must agree with the `latActive` gate: the bit without the gate changes
   *  nothing, and the gate without the bit makes the panda drop our frames. One switch owns both. */
  void setAlternativeExperience(uint16_t alt_exp) { alt_exp_ = alt_exp; }
  uint16_t alternativeExperience() const { return alt_exp_; }

  bool updateIgnitionSticky(bool ignition_hw, uint32_t voltage_mv, int64_t now_ms);

  std::optional<health_t> tick(Panda& panda, health_t health, int64_t now_ms, const SafetyLogContext& log);

  uint16_t lastSafetyMode() const { return last_safety_mode_; }
  bool lastControlsAllowed() const { return last_controls_allowed_; }
  bool lastIgnition() const { return last_ignition_; }
  bool safetyConfigured() const { return safety_configured_; }

private:
  void scheduleSafetyRetry(int64_t now_ms);
  void clearSafetyRetry();

  bool initialized_ = false;
  bool safety_configured_ = false;
  bool ignition_sticky_ = false;
  int64_t ignition_low_since_ms_ = 0;
  bool alt_exp_configured_ = false;
  uint16_t alt_exp_ = C::kAltExpDisableDisengageOnGas;

  int64_t next_safety_attempt_ms_ = 0;
  int safety_fail_streak_ = 0;

  uint16_t last_safety_mode_ = C::kNoOutput;
  bool last_controls_allowed_ = false;
  bool last_ignition_ = false;
};

}  // namespace volkswagen
