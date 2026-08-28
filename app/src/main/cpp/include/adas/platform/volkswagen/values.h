#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace volkswagen {
struct CanBus {
  static constexpr int pt = 0;
  static constexpr int cam = 2;
};

struct CarControllerParams {
  static constexpr int STEER_STEP = 2;
  static constexpr int LDW_STEP = 10;
  /// ACC_06/ACC_07 go at 50 Hz like HCA, the ACC_02 cluster message at ~16.7 Hz — the radar's own cadence.
  static constexpr int ACC_CONTROL_STEP = 2;
  static constexpr int ACC_HUD_STEP = 6;

  /// The panda's longitudinal envelope for this platform [m/s²]: 2000 / −3500 in its milli-units, and
  /// 3.01 is the value the bus reads as "no request".
  static constexpr float ACCEL_MAX = 2.0f;
  static constexpr float ACCEL_MIN = -3.5f;
  static constexpr float ACCEL_INACTIVE = 3.01f;
  /// Acceleration gradient the ACC_06 frame advertises when active [m/s³]; upstream's constant.
  static constexpr float ACC_JERK_LIMIT = 4.0f;

  static constexpr int STEER_MAX = 300;
  static constexpr int STEER_DRIVER_MULTIPLIER = 3;
  static constexpr int STEER_DRIVER_FACTOR = 1;
  static constexpr int STEER_DRIVER_ALLOWANCE = 80;

  static constexpr int STEER_DELTA_UP = 4;
  static constexpr int STEER_DELTA_DOWN = 10;

  static constexpr int LDW_MSG_NONE = 0;
  static constexpr int LDW_MSG_TAKE_OVER = 8;
};

enum class EpsHcaStatus : uint8_t {
  Disabled = 0,
  Initializing = 1,
  Fault = 2,
  Ready = 3,
  Rejected = 4,
  Active = 5,
};

/// \return EPS HCA status as text for logs.
inline const char* epsHcaStatusName(uint8_t s)
{
  switch (s) {
    case 0:
      return "DISABLED";
    case 1:
      return "INITIALIZING";
    case 2:
      return "FAULT";
    case 3:
      return "READY";
    case 4:
      return "REJECTED";
    case 5:
      return "ACTIVE";
    default:
      return "UNKNOWN";
  }
}

/** `ACC_Status_ACC` / `ACC_Status_Anzeige`: what the ACC reports about itself on the bus. */
enum class AccStatus : uint8_t {
  Off = 0,
  MainOn = 2,
  Active = 3,
  Faulted = 6,
};

/// \return The status byte the ACC frames carry for this combination — upstream's acc_control_value.
inline uint8_t accControlValue(bool main_switch_on, bool acc_faulted, bool long_active)
{
  if (acc_faulted)
    return static_cast<uint8_t>(AccStatus::Faulted);
  if (long_active)
    return static_cast<uint8_t>(AccStatus::Active);
  if (main_switch_on)
    return static_cast<uint8_t>(AccStatus::MainOn);
  return static_cast<uint8_t>(AccStatus::Off);
}

/// \return True when the TSK status means the stock cruise/ACC module reports a fault (upstream accFaulted).
inline bool accFaultedFromTsk(int tsk_status) { return tsk_status == 6 || tsk_status == 7; }

/// \return True when the EPS reports a state that accepts torque.
inline bool epsHcaAllowsSteer(uint8_t s)
{
  return s == static_cast<uint8_t>(EpsHcaStatus::Ready) || s == static_cast<uint8_t>(EpsHcaStatus::Active);
}

}  // namespace volkswagen
