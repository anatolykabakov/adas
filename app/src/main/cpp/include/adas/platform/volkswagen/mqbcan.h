#pragma once

#include <cstdint>

#include "adas/panda/can_frame.h"
#include "adas/platform/volkswagen/values.h"

namespace volkswagen {
struct HudControl {
  bool leftLaneVisible = false;
  bool rightLaneVisible = false;
  bool leftLaneDepart = false;
  bool rightLaneDepart = false;
};

struct LdwStockValues {
  uint8_t data[8]{};
  bool valid = false;
};

/** The acceleration request as ACC_06/ACC_07 carry it. */
struct AccControl {
  bool enabled = false;      ///< Longitudinal control active: an acceleration is being requested.
  float accel_ms2 = 0.f;     ///< Request [m/s²]; ignored when not enabled (the frames say "inactive").
  uint8_t acc_type = 0;      ///< `ACC_Typ` mirrored from the stock radar frame, so the cluster agrees.
  uint8_t acc_status = 0;    ///< `ACC_Status_ACC`, see accControlValue().
  bool stopping = false;     ///< The law is in its stopping state: hold request, short stop distance.
  bool starting = false;     ///< The law is in its starting state: hold release.
  bool esp_hold = false;     ///< The ESP confirms a standstill hold (ESP_21), so ACC_07 asks for standby.
};

/** The cluster's ACC picture (ACC_02). */
struct AccHud {
  uint8_t acc_status = 0;      ///< `ACC_Status_Anzeige`, same encoding as the control status.
  float set_speed_kph = 0.f;   ///< `ACC_Wunschgeschw_02`; above 250 the cluster draws nothing.
  int lead_distance = 0;       ///< `ACC_Abstandsindex`: 0 none, 8 (or 512 on newer clusters) a lead is drawn.
  int time_gap = 3;            ///< `ACC_Gesetzte_Zeitluecke` bars, 1..5.
};

/// Build HCA_01. \param[in,out] counter Rolling counter, incremented.
can_frame create_steering_control(int bus, int apply_steer, bool lkas_enabled, uint8_t* counter);

/// Build LDW_02 mirroring the stock payload with our lane-visible bits.
can_frame create_lka_hud_control(int bus, const LdwStockValues& ldw_stock, bool enabled, bool steering_pressed,
                                 int hud_alert, const HudControl& hud);

/// Build ACC_06 (0x122): the acceleration request the motor and brake controllers read.
/// \param[in,out] counter Rolling counter, incremented.
can_frame create_acc_06(int bus, const AccControl& acc, uint8_t* counter);

/// Build ACC_07 (0x12E): the same acceleration for the ESP, plus stop distance and hold requests.
/// \param[in,out] counter Rolling counter, incremented.
can_frame create_acc_07(int bus, const AccControl& acc, uint8_t* counter);

/// Build ACC_02 (0x30C): what the cluster shows. \param[in,out] counter Rolling counter, incremented.
can_frame create_acc_02(int bus, const AccHud& hud, uint8_t* counter);

/// MQB CRC-8 (poly 0x2F) with the per-message, per-counter secret — exposed for the tests.
uint8_t volkswagen_mqb_checksum(long address, const uint8_t* data, size_t len);

}  // namespace volkswagen
