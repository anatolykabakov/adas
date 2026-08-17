#pragma once

#include <cstdint>

namespace adas {
namespace platform {
namespace toyota {

/**
 * \brief Actuation limits for Toyota TSS2, as the panda enforces them.
 *
 * \details Ported from `selfdrive/car/toyota/values.py::CarControllerParams`. Toyota steers by torque
 * like MQB does, but the numbers are not comparable: the units are the EPS's own, the ceiling is 1500
 * against MQB's 300, and the message goes out every frame rather than every second one. Anything that
 * treats "torque" as one scale across makes will be wrong by a factor of five here.
 */
struct CarControllerParams {
  static constexpr int STEER_MAX = 1500;       ///< Torque at full command, in the EPS's units.
  static constexpr int STEER_STEP = 1;         ///< STEERING_LKA goes out at 100 Hz, every frame.
  static constexpr int STEER_DELTA_UP = 10;    ///< Rise per step; 1.5 s to peak torque.
  static constexpr int STEER_DELTA_DOWN = 25;  ///< Fall per step. Above 45 the RAV4 faults.
  static constexpr int STEER_ERROR_MAX = 350;  ///< Largest gap between command and motor before a fault.

  /// Driver torque tolerated before the panda takes the rack back. Toyota's safety model uses the
  /// EPS torque signal directly, so this is in the same units as `STEER_MAX`.
  static constexpr int STEER_DRIVER_ALLOWANCE = 100;
};

/** Panda safety model ids, as `panda/python/__init__.py` numbers them. */
struct SafetyConstants {
  static constexpr uint16_t kToyota = 2;
  static constexpr uint16_t kNoOutput = 19;
  /// Torque-sensor scale, the parameter the Toyota safety model takes. 73 is the common TSS2 value.
  static constexpr uint16_t kEpsScale = 73;
};

/** Addresses we decode. Everything else is dropped before it reaches the parser. */
struct Addresses {
  static constexpr uint32_t kSteerAngleSensor = 0x25;  ///< 37
  static constexpr uint32_t kWheelSpeeds = 0xAA;       ///< 170
  static constexpr uint32_t kBrakeModule = 0x226;      ///< 550
  static constexpr uint32_t kEpsStatus = 0x262;        ///< 610
  static constexpr uint32_t kPcmCruise = 0x1D2;        ///< 466
  static constexpr uint32_t kPcmCruise2 = 0x1D3;       ///< 467
  static constexpr uint32_t kGasPedal = 0x2C1;         ///< 705
  static constexpr uint32_t kGearPacket = 0x3BC;       ///< 956
  static constexpr uint32_t kSteeringLka = 0x2E4;      ///< 740, the one we send
};

}  // namespace toyota
}  // namespace platform
}  // namespace adas
