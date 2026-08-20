#pragma once

#include <string>

#include "adas/utils/speed_filter.h"

namespace adas {
namespace services {
/** Car-side settings: the bus, the brand, and the cruise buttons. */
struct CarConfig {
  int usb_fd = -1;                        ///< File descriptor of the panda, opened by the host; -1 means no hardware.
  std::string dbc_path;                   ///< CAN database the decoder parses.
  bool cruise_buttons_enabled = false;    ///< Drive the longitudinal axis through the stock cruise buttons.
  double cruise_deadband_ms = 0.70;       ///< Speed error below which no button is pressed [m/s].
  int cruise_tip_cooldown_ms = 200;       ///< Minimum gap between button presses [ms].
  double cruise_tip_step_ms = 1.0 / 3.6;  ///< Setpoint change per press [m/s] — one stalk press is 1 km/h.
  adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
};

}  // namespace services
}  // namespace adas
