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
/**
 * \brief Everything the rest of the system needs from a car, with nothing brand-specific in the signatures.
 *
 * \details The shape follows comma's split of `CarState` / `CarController` / safety: a brand supplies
 * three things and nothing else — how to read the bus, how to write to it, and what the panda will
 * allow. The `Platform` service owns one of these and never names a brand.
 *
 * The implementation keeps its own decoded state, so the service never handles a brand-typed view: it
 * hands over a request and receives frames. Anything a particular make needs to build those frames — a
 * rolling counter to match, a stock LDW payload to mirror, an EPS status byte — stays inside that
 * make's implementation.
 *
 * Adding a car means implementing this interface in `platform/<brand>/` and registering it in
 * `makeCarPlatform`. Nothing outside that directory should need to change.
 */
class CarPlatform {
public:
  virtual ~CarPlatform() = default;

  /// Brand and variant, as it goes into the log and the bag.
  virtual const char* name() const = 0;

  /// Opens whatever the brand needs — a DBC, a decoder — before the first frame arrives.
  virtual void init() = 0;

  /// The CAN database this car is decoded with, as an asset file name. Empty when it needs none.
  virtual const char* dbcAssetName() const = 0;

  /**
   * \brief The car's own geometry and steering facts.
   *
   * \details Read before the config is applied, so a config that says nothing about the vehicle still
   * describes the right one. Selecting a car and getting another car's wheelbase is how a controller
   * ends up confidently wrong about curvature.
   */
  virtual VehicleDefaults defaults() const = 0;

  // --- Reading the bus ------------------------------------------------------

  /// Addresses this car is allowed to receive; anything else is dropped before decoding.
  virtual bool isAllowedRxAddress(uint32_t address) const = 0;

  /**
   * \brief Decode a batch of frames.
   * \param[in] msg Frames received this tick.
   * \param[in] now_ms Arrival time [ms].
   * \return True when a decoded value changed — the service publishes on this, not on every batch.
   */
  virtual bool update(const adas::proto::CANData& msg, int64_t now_ms) = 0;

  /// The full decoded state for the bag, as the shared `CarState` message.
  virtual const adas::proto::CarState& carState() const = 0;

  /// Brand-neutral view of the car, the only state the controllers see.
  virtual CarStateView stateView() const = 0;

  // --- Writing to the bus ---------------------------------------------------

  /**
   * \brief Frames to send this tick for the requested actuation.
   *
   * \details Rate limiting, counters and checksums are the brand's business: the caller only says what
   * it wants. The car state used is the one decoded by `update`, so the caller never holds a
   * brand-typed view.
   */
  virtual std::vector<can_frame> apply(const CarControl& cc) = 0;

  /**
   * \brief Intent for the stock cruise, for cars driven through the buttons rather than by torque.
   * \param[in] intent 0 none, 1 nudge the setpoint up, 2 nudge it down.
   * \param[in] now_ms Now [ms]; the brand owns whatever handshake and cooldown its bus needs.
   */
  virtual void setCruiseIntent(int intent, int64_t now_ms) = 0;

  /// Steering envelope the panda enforces, so the controller clamps before the panda has to.
  virtual SteerLimits steerLimits() const = 0;

  // --- What the panda will allow -------------------------------------------

  /**
   * \brief The command just issued, for whatever the brand records next to the health packet.
   *
   * \details Kept separate from `apply` because the health tick runs on its own timer: without this the
   * log would show the command from whichever tick happened to be last, which is the one moment nobody
   * is asking about.
   *
   * \param[in] torque_cnm Requested torque [cNm].
   * \param[in] lat_active Whether lateral actuation was requested at all.
   */
  virtual void setLastCommand(int torque_cnm, bool lat_active) = 0;

  /// Tells the supervisor which safety model and alternative-experience bits this car needs.
  virtual void configureSafety() = 0;

  /**
   * \brief One supervisor tick: heartbeat, safety mode, ignition debounce.
   * \param[in] panda Open device.
   * \param[in] health Health packet just read from it.
   * \param[in] now_ms Now [ms].
   * \return The health to publish, or nothing when this tick produced none.
   */
  virtual std::optional<health_t> safetyTick(::Panda& panda, health_t health, int64_t now_ms) = 0;

  /// True while the car is on, after the brand's own debounce.
  virtual bool ignition() const = 0;

  /// True when the panda is in the mode this car needs; a mismatch means no frames may be sent.
  virtual bool safetyModelOk() const = 0;

  /// Whether lateral torque may reach the rack this tick, given what the panda reports.
  virtual bool lateralActuationAllowed() const = 0;

  /// Put the panda into a mode that cannot actuate. Called on shutdown, and it must not throw.
  virtual void enterSafeMode(::Panda& panda) = 0;
};

/// Everything a brand may need at construction, so the factory signature does not grow per car.
struct CarPlatformOptions {
  std::string dbc_path;                      ///< DBC to parse, for brands that decode through one.
  adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  bool cruise_buttons_enabled = false;       ///< Drive the longitudinal axis through the stock buttons.
  int cruise_tip_cooldown_ms = 200;          ///< Minimum gap between button presses [ms].
};

/**
 * \brief Builds the platform named in `vehicle.name`.
 *
 * \details Unknown names are an error, not a silent default: guessing the car wrong is guessing the CAN
 * layout wrong, and a wrong layout puts torque on the rack from a frame that meant something else.
 *
 * \param[in] name Vehicle name from the config.
 * \param[in] opts What the brand needs to construct itself.
 * \return The platform, or nullptr when the name is not known.
 */
std::unique_ptr<CarPlatform> makeCarPlatform(const std::string& name, const CarPlatformOptions& opts);

/// Names `makeCarPlatform` accepts, for the error message and for tests.
std::vector<std::string> knownCarPlatforms();

}  // namespace platform
}  // namespace adas
