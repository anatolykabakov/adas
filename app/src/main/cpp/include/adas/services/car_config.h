#pragma once

#include <string>

#include "adas/utils/speed_filter.h"

namespace adas {
namespace services {
/**
 * \brief Car-side settings: the bus, the brand, and the cruise buttons.
 *
 * \details These lived in the config of the monolithic `Panda` service. After the split the shared
 * config stayed single: `Platform` reads the bus and the brand, `Control` reads the cruise decision.
 * A separate struct keeps either service from owning the other's settings.
 */
struct CarConfig {
  int usb_fd = -1;  ///< File descriptor of the panda, opened by the host; -1 means no hardware.
  std::string dbc_path;
  bool cruise_buttons_enabled = false;       ///< Drive the longitudinal axis through the stock cruise buttons.
  double cruise_deadband_ms = 0.70;          ///< Speed error below which no button is pressed [m/s].
  int cruise_tip_cooldown_ms = 200;          ///< Minimum gap between button presses [ms].
  double cruise_tip_step_ms = 1.0 / 3.6;     ///< Setpoint change per press [m/s] — one stalk press is 1 km/h.
  adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
};

}  // namespace services
}  // namespace adas
