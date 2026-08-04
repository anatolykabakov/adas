#pragma once

#include <cstdint>

#include "panda/can_frame.h"
#include "volkswagen/values.h"

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

can_frame create_steering_control(int bus, int apply_steer, bool lkas_enabled, uint8_t* counter);

can_frame create_lka_hud_control(int bus, const LdwStockValues& ldw_stock, bool enabled, bool steering_pressed,
                                 int hud_alert, const HudControl& hud);

can_frame create_acc_buttons_control(int bus, const GraStockValues& gra_stock, const CruiseButtonCmd& cmd);

}  // namespace volkswagen
