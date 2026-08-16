#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "adas/middleware/manager.hpp"
#include "adas/panda/panda.h"
#include "adas/platform/car_platform.h"
#include "messages.pb.h"

namespace adas {
namespace services {
/**
 * \brief The panda driver and nothing more: bytes both ways plus the safety supervisor.
 *
 * \details The boundary follows CAN frames, as upstream draws it: `boardd` carries `can` and `sendcan`
 * while decoding lives above it, in the car's own implementation. What stays here is only what needs
 * the hardware — receive, send, poll the health, and the supervisor tick that feeds the panda its
 * heartbeat and safety mode.
 *
 * Which car is behind the bus is decided once, by `vehicle.name`, and reached only through
 * `platform::CarPlatform`. This service names no brand: adding a car does not change a line of it.
 */
class Platform : public adas::middleware::Service {
public:
  struct Config {
    int usb_fd = -1;  ///< Panda file descriptor, opened by the host; -1 means no hardware.
    std::string dbc_path;
    std::string car_name = "vw_golf_7_mqb";    ///< Which car is on the bus; `vehicle.name` in the config.
    bool cruise_buttons_enabled = false;       ///< Send cruise-button frames at all.
    int cruise_tip_cooldown_ms = 200;          ///< Minimum gap between button presses [ms].
    adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  };

  explicit Platform(Config config);
  ~Platform();

  void configure() override;
  void reset() override {}
  std::string_view getName() const override { return "platform"; }
  const Config& config() const { return config_; }

private:
  void rxCallback();
  void stateCallback();
  void txCallback();

  Config config_;
  std::shared_ptr<::Panda> panda_;

  /// The car behind the bus. Null when `vehicle.name` names one we do not have — the service then
  /// receives and publishes nothing rather than guessing a layout.
  std::unique_ptr<adas::platform::CarPlatform> car_;

  /// The controller's last command — the intent this service turns into frames.
  adas::proto::SteerCommand cmd_{};
};

}  // namespace services
}  // namespace adas
