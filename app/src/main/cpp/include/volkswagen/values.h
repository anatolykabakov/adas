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

inline bool epsHcaAllowsSteer(uint8_t s)
{
  return s == static_cast<uint8_t>(EpsHcaStatus::Ready) || s == static_cast<uint8_t>(EpsHcaStatus::Active);
}

}  // namespace volkswagen
