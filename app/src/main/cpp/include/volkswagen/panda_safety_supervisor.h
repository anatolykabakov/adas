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

  int64_t next_safety_attempt_ms_ = 0;
  int safety_fail_streak_ = 0;

  uint16_t last_safety_mode_ = C::kNoOutput;
  bool last_controls_allowed_ = false;
  bool last_ignition_ = false;
};

}  // namespace volkswagen
