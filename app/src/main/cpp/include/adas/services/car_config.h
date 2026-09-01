#pragma once

#include <string>

#include "adas/utils/speed_filter.h"

namespace adas {
namespace services {
/** Car-side settings: the bus, the brand, and whether we take the longitudinal axis. */
struct CarConfig {
  int usb_fd = -1;       ///< File descriptor of the panda, opened by the host; -1 means no hardware.
  std::string dbc_path;  ///< CAN database the decoder parses.
  /// Take over speed: the platform sends its ACC frames and asks the panda for its longitudinal flag,
  /// and the control law produces an acceleration. Off is lateral-only with the stock ACC untouched.
  bool long_control_enabled = false;
  adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
};

}  // namespace services
}  // namespace adas
