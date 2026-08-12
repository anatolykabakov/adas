#pragma once

#include <cstdint>
#include <optional>

#include "car_state.pb.h"

namespace adas {
namespace platform {
/** What the lateral controller asks for. Torque in centinewton-metres when the platform actuates
 *  torque directly; `steer` is the same request normalised to [-1, 1]. */
struct Actuators {
  float steer = 0.f;
  std::optional<int> steerTorqueCNm;
};

/** Driver-facing state the car's own cluster shows. Kept minimal on purpose: anything a particular
 *  brand draws differently belongs in that brand's implementation, not here. */
struct HudState {
  bool leftLaneVisible = false;
  bool rightLaneVisible = false;
  bool leftLaneDepart = false;
  bool rightLaneDepart = false;
  int visualAlert = 0;
};

/** Stock-cruise button request for cars driven through the cruise stalk rather than by torque or
 *  acceleration directly. */
struct CruiseRequest {
  bool resume = false;
  bool set = false;
  bool cancel = false;
};

struct CarControl {
  bool latActive = false;
  Actuators actuators;
  HudState hud;
  CruiseRequest cruise;
};

/** The car as the controllers see it, with nothing brand-specific in it.
 *
 *  A brand that needs more — a stock LDW payload to mirror back, an EPS status byte, a rolling
 *  counter to match — keeps that inside its own implementation. If a field here is only meaningful
 *  for one make, it is in the wrong place. */
struct CarStateView {
  float vEgo = 0.f;
  bool standstill = false;
  float steeringTorque = 0.f;
  bool steeringPressed = false;
  bool cruiseEngaged = false;
  bool cruiseAvailable = false;
  bool gearReverse = false;
  bool gearKnown = false;
  bool gasPressed = false;
  bool brakePressed = false;
};

/** The steering envelope the panda enforces. The controller clamps to these so the panda never has to
 *  cut the command — a cut is a fault, not a limit. */
struct SteerLimits {
  int maxTorqueCNm = 0;
  int stepFrames = 0;
  int deltaUpPerStep = 0;
  int deltaDownPerStep = 0;
  int driverAllowanceCNm = 0;
};

}  // namespace platform
}  // namespace adas
