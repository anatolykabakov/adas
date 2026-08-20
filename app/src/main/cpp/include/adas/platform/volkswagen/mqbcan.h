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

struct GraStockValues {
  uint8_t data[8]{};
  bool valid = false;
  /// \return The 4-bit rolling counter of the frame.
  uint8_t counter() const { return data[1] & 0x0F; }
};

struct CruiseButtonCmd {
  bool cancel = false;
  bool resume = false;
  bool set = false;
  bool tip_up = false;
  bool tip_down = false;
  bool send = false;
};

/// Build HCA_01. \param[in,out] counter Rolling counter, incremented.
can_frame create_steering_control(int bus, int apply_steer, bool lkas_enabled, uint8_t* counter);

/// Build LDW_02 mirroring the stock payload with our lane-visible bits.
can_frame create_lka_hud_control(int bus, const LdwStockValues& ldw_stock, bool enabled, bool steering_pressed,
                                 int hud_alert, const HudControl& hud);

/// Build GRA_ACC_01 with the requested button press on top of the stock payload.
can_frame create_acc_buttons_control(int bus, const GraStockValues& gra_stock, const CruiseButtonCmd& cmd);

}  // namespace volkswagen
