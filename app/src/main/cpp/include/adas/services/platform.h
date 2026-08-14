#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <string_view>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"
#include "adas/panda/panda.h"
#include "adas/platform/volkswagen/car_iface.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"

namespace adas {
namespace services {
/**
 * \brief The panda driver and nothing more: bytes both ways plus the safety supervisor.
 *
 * \details The boundary follows CAN frames, as upstream draws it: `boardd` carries `can` and `sendcan`
 * while decoding lives above it, in the controller's `CarIface`. What stays here is only what needs the
 * hardware — receive, send, poll the health, and the supervisor tick that feeds the panda its heartbeat
 * and safety mode.
 *
 * `CarState` decoding is deliberately elsewhere: it is knowledge about a car brand, not about USB. This
 * service knows no brand at all, so adding a car never touches it.
 */
class Platform : public adas::middleware::Service {
public:
  struct Config {
    int usb_fd = -1;  ///< Panda file descriptor, opened by the host; -1 means no hardware.
    std::string dbc_path;
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
  /// GRA counter handshake: hold the button until the counter advances, then pause.
  volkswagen::CruiseButtonCmd cruiseButtons(const volkswagen::CarStateView& cs);

  Config config_;
  std::shared_ptr<::Panda> panda_;
  volkswagen::PandaSafetySupervisor safety_;
  volkswagen::CarIface ci_;

  /// The controller's last command — the intent this service turns into frames.
  adas::proto::SteerCommand cmd_{};

  int64_t cruise_cooldown_until_ms_ = 0;
  bool cruise_hold_tip_up_ = false;
  bool cruise_hold_tip_down_ = false;
  uint8_t cruise_gra_cnt_at_arm_ = 0xFF;
};

}  // namespace services
}  // namespace adas
