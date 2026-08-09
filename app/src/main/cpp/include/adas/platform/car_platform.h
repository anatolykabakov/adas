#pragma once

#include <memory>
#include <string>
#include <vector>

#include "adas/panda/can_frame.h"
#include "adas/platform/car_types.h"

namespace adas {
namespace platform {

/** Everything the rest of the system needs from a car, with nothing brand-specific in the signatures.
 *
 *  The shape follows comma's split of `CarState` / `CarController` / safety: a brand supplies three
 *  things and nothing else — how to read the bus, how to write to it, and what the panda will allow.
 *  `Panda` owns one of these through a pointer and never names a brand.
 *
 *  Adding a car means implementing this interface in `platform/<brand>/` and registering it in
 *  `makeCarPlatform`. Nothing outside that directory should need to change.
 */
class CarPlatform {
public:
  virtual ~CarPlatform() = default;

  /** Brand and variant, as it goes into the log and the bag. */
  virtual const char* name() const = 0;

  // --- reading the bus -------------------------------------------------------------------------

  /** Addresses this car is allowed to receive; anything else is dropped before decoding. */
  virtual bool isAllowedRxAddress(uint32_t address) const = 0;

  virtual void updateFromFrame(const can_frame& frame) = 0;

  /** True once since the last call if any decoded value changed — the service publishes on this. */
  virtual bool consumeDirty() = 0;

  /** Brand-neutral view of the car, the only state the controllers see. */
  virtual CarStateView stateView() const = 0;

  /** The full decoded state for the bag, as the shared `CarState` message. */
  virtual const ai::flow::adas::CarState& carState() const = 0;

  // --- writing to the bus ----------------------------------------------------------------------

  /** Frames to send this tick for the requested actuation. Rate limiting, counters and checksums are
   *  the platform's business: the caller only says what it wants. */
  virtual std::vector<can_frame> update(const CarControl& cc, const CarStateView& cs) = 0;

  /** Stock-cruise button presses, if this car is driven through them. Brands that actuate the
   *  longitudinal axis directly return nothing. */
  virtual void requestCruise(const CruiseRequest& req) = 0;

  // --- what the panda will allow ---------------------------------------------------------------

  /** Steering limits the panda enforces, so the controller can clamp before the panda has to. */
  virtual SteerLimits steerLimits() const = 0;

  /** Whether lateral torque may reach the rack this tick, given what the panda reports. */
  virtual bool lateralActuationAllowed(bool controls_allowed, bool always_on, const CarStateView& cs) const = 0;
};

/** Builds the platform named in `vehicle.name`. Unknown names are an error, not a silent default:
 *  guessing the car wrong is guessing the CAN layout wrong. */
std::unique_ptr<CarPlatform> makeCarPlatform(const std::string& name);

}  // namespace platform
}  // namespace adas
