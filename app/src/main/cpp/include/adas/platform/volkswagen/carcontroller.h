#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "adas/panda/can_frame.h"
#include "adas/platform/volkswagen/mqbcan.h"
#include "adas/platform/volkswagen/values.h"

namespace volkswagen {

struct Actuators {
  float steer = 0.f;

  std::optional<int> steerTorqueCNm;
};

int applyDriverSteerTorqueLimits(int apply_torque, float driver_torque, int apply_steer_last);

struct CarControl {
  bool latActive = false;
  Actuators actuators;
  HudControl hud;
  int visualAlert = 0;
  CruiseButtonCmd cruise{};
};

struct CarStateView {
  float vEgo = 0.f;
  bool standstill = false;
  float steeringTorque = 0.f;
  bool steeringPressed = false;
  uint8_t epsHcaStatus = 0;
  LdwStockValues ldwStock;
  GraStockValues graStock;
  bool cruiseEngaged = false;
  /** Cruise main switch on — `TSK_Status != 0/1`. This, not `cruiseEngaged`, is what always-on lateral
   *  keys on: the stock VW cruise disengages below ~30 km/h and on brake, while the main switch stays on. */
  bool cruiseAvailable = false;
  bool gearReverse = false;
  /// Getriebe_11 has been seen at least once; before that the gear is unknown.
  bool gearKnown = false;
  bool gasPressed = false;
  bool brakePressed = false;
};

/** Whether lateral torque may go to the rack this frame.
 *
 *  Pure on purpose: this is the decision that puts torque on the wheel, and as a service method it needed a
 *  USB handle to reach, so nothing tested it directly. Upstream's other two conditions — standstill
 *  (`|vEgo| < 0.3`, their `MIN_LATERAL_CONTROL_SPEED`) and the EPS fault check — are enforced inside
 *  `CarController::update` on every frame and deliberately not repeated here; duplicating them would create
 *  two places to keep in step. */
inline bool lateralActuationAllowed(bool controls_allowed, bool always_on, const CarStateView& cs)
{
  if (controls_allowed)
    return true;
  if (!always_on)
    return false;
  // gearKnown is required: before the first Getriebe_11 frame the gear reads as 0, and "not reverse"
  // would be formally true and substantially false — the car may be in reverse. Unknown means no.
  return cs.cruiseAvailable && cs.gearKnown && !cs.gearReverse;
}

class CarController {
public:
  CarController() = default;

  std::vector<can_frame> update(const CarControl& CC, const CarStateView& CS);

  int applySteerLast() const { return apply_steer_last_; }
  int frame() const { return frame_; }

private:
  int apply_steer_last_ = 0;
  int frame_ = 0;
  int hca_same_torque_count_ = 0;
  int hca_enabled_frame_count_ = 0;
  uint8_t hca_counter_ = 0;
  uint8_t gra_acc_counter_last_ = 0xFF;
};

}  // namespace volkswagen
