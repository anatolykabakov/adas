#pragma once

#include <atomic>
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
/** The panda driver and nothing more: bytes both ways plus the safety supervisor. */
class Platform : public adas::middleware::Service {
public:
  struct Config {
    int usb_fd = -1;                           ///< Panda file descriptor, opened by the host; -1 means no hardware.
    std::string dbc_path;                      ///< CAN database the decoder parses.
    std::string car_name = "vw_golf_7_mqb";    ///< Which car is on the bus; `vehicle.name` in the config.
    bool long_control_enabled = false;         ///< Send ACC frames; see CarConfig::long_control_enabled.
    adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  };

  /// \param[in] config Panda descriptor, DBC path and car name.
  explicit Platform(Config config);
  ~Platform();

  void configure() override;
  void reset() override {}
  std::string_view getName() const override { return "platform"; }
  /// \return The config in force.
  const Config& config() const { return config_; }

  /**
   * \brief Hand the service a freshly opened panda descriptor.
   * \param[in] usb_fd Descriptor of an already opened panda; negative is ignored.
   */
  void reseatPanda(int usb_fd);

  /// How many times a descriptor was actually swapped in. Diagnostics only.
  int pandaReseats() const { return reseats_.load(std::memory_order_relaxed); }

private:
  void rxCallback();
  void stateCallback();
  void txCallback();
  /// Swap in a queued descriptor. Runs on this service's thread only.
  void applyPendingPanda();

  Config config_;
  std::shared_ptr<::Panda> panda_;

  /// The car behind the bus. Null when `vehicle.name` names one we do not have — the service then
  /// receives and publishes nothing rather than guessing a layout.
  std::unique_ptr<adas::platform::CarPlatform> car_;

  /// The controller's last command — the intent this service turns into frames.
  adas::proto::SteerCommand cmd_{};

  /// A descriptor waiting to be seated, or -1. Written by any thread, consumed by ours.
  std::atomic<int> pending_fd_{-1};
  std::atomic<int> reseats_{0};
};

}  // namespace services
}  // namespace adas
