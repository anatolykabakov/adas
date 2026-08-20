#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "adas/panda/can_frame.h"
#include "adas/panda/health.h"
#include "adas/panda/panda.h"
#include "adas/platform/car_types.h"
#include "adas/utils/speed_filter.h"
#include "messages.pb.h"

namespace adas {
namespace platform {
/** One car behind a brand-neutral interface: read the bus, write to it, know what the panda allows. */
class CarPlatform {
public:
  virtual ~CarPlatform() = default;

  /// Brand and variant, as logged and written to the bag.
  virtual const char* name() const = 0;

  /// Opens whatever the brand needs (DBC, decoder) before the first frame.
  virtual void init() = 0;

  /// CAN database asset name; empty when none is needed.
  virtual const char* dbcAssetName() const = 0;

  /// Geometry and steering facts, used as config defaults for this car.
  virtual VehicleDefaults defaults() const = 0;

  // --- Reading the bus ------------------------------------------------------

  /// Addresses this car may receive; anything else is dropped before decoding.
  virtual bool isAllowedRxAddress(uint32_t address) const = 0;

  /**
   * \brief Decode a batch of frames.
   * \param[in] msg Frames received this tick.
   * \param[in] now_ms Arrival time [ms].
   * \return True when a decoded value changed; the service publishes on this.
   */
  virtual bool update(const adas::proto::CANData& msg, int64_t now_ms) = 0;

  /// Full decoded state for the bag.
  virtual const adas::proto::CarState& carState() const = 0;

  /// Brand-neutral view, the only state the controllers see.
  virtual CarStateView stateView() const = 0;

  // --- Writing to the bus ---------------------------------------------------

  /// Frames to send this tick. Rate limits, counters and checksums are the brand's business.
  virtual std::vector<can_frame> apply(const CarControl& cc) = 0;

  /**
   * \brief Stock-cruise intent, for cars driven through the buttons.
   * \param[in] intent 0 none, 1 setpoint up, 2 setpoint down.
   * \param[in] now_ms Now [ms].
   */
  virtual void setCruiseIntent(int intent, int64_t now_ms) = 0;

  /// Steering envelope the panda enforces, so the controller clamps first.
  virtual SteerLimits steerLimits() const = 0;

  // --- What the panda will allow -------------------------------------------

  /**
   * \brief The command just issued, recorded next to the health packet.
   * \param[in] torque_cnm Requested torque [cNm].
   * \param[in] lat_active Whether lateral actuation was requested.
   */
  virtual void setLastCommand(int torque_cnm, bool lat_active) = 0;

  /// Tells the supervisor which safety model and alternative-experience bits this car needs.
  virtual void configureSafety() = 0;

  /**
   * \brief Forget board state, keep car state. Called when a re-opened panda descriptor is seated:
   * safety model and alt-experience live on the board and must be set again, while the decoded car
   * state and the cruise latch survive.
   */
  virtual void resetPandaState() = 0;

  /**
   * \brief One supervisor tick: heartbeat, safety mode, ignition debounce.
   * \return The health to publish, or nothing.
   */
  virtual std::optional<health_t> safetyTick(::Panda& panda, health_t health, int64_t now_ms) = 0;

  /// True while the car is on, after the brand's debounce.
  virtual bool ignition() const = 0;

  /// True when the panda is in the mode this car needs; mismatch forbids sending.
  virtual bool safetyModelOk() const = 0;

  /// Whether lateral torque may reach the rack this tick.
  virtual bool lateralActuationAllowed() const = 0;

  /// Put the panda into a non-actuating mode. Called on shutdown; must not throw.
  virtual void enterSafeMode(::Panda& panda) = 0;
};

/// Construction options, so the factory signature does not grow per car.
struct CarPlatformOptions {
  std::string dbc_path;                      ///< DBC to parse, when the brand decodes through one.
  adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  bool cruise_buttons_enabled = false;       ///< Drive the longitudinal axis through the stock buttons.
  int cruise_tip_cooldown_ms = 200;          ///< Minimum gap between button presses [ms].
};

/**
 * \brief Builds the platform named in `vehicle.name`.
 * \return The platform, or nullptr for an unknown name — no default: a guessed car is a guessed CAN layout.
 */
std::unique_ptr<CarPlatform> makeCarPlatform(const std::string& name, const CarPlatformOptions& opts);

/// Names makeCarPlatform() accepts, for error messages and tests.
std::vector<std::string> knownCarPlatforms();

}  // namespace platform
}  // namespace adas
