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

/**
 * \brief What a car is, as facts about the vehicle rather than settings.
 *
 * \details These belong to the make and model, not to the driver: nobody tunes the wheelbase of their
 * Golf. Upstream keeps them in `selfdrive/car/<brand>/interface.py` for exactly that reason, and we
 * follow — the platform supplies them, `config.json` may override one when a particular car differs
 * (a trim with a different rack, say), and an untouched config means the car's own numbers are used.
 */
struct VehicleDefaults {
  double wheelbase_m = 2.636;          ///< Wheelbase [m].
  double steer_ratio = 15.7;           ///< Steering-wheel to road-wheel ratio.
  double mass_kg = 1533.0;             ///< Kerb mass plus a driver [kg].
  double center_to_front_frac = 0.45;  ///< Distance to the front axle as a fraction of the wheelbase.
  double steer_sign = -1.0;            ///< Which way a positive command turns the wheel: +1 or −1.
  double tire_stiffness_factor = 1.0;  ///< Starting scale on the reference stiffness; the learner refines it.
  double max_steer_deg = 20.0;         ///< Ceiling on the commanded road-wheel angle [deg].
};

/** The steering envelope the panda enforces. The controller clamps to these so the panda never has to
 *  cut the command — a cut is a fault, not a limit. */
struct SteerLimits {
  int maxTorqueCNm = 0;        ///< Torque at full command [cNm], as the panda will allow it.
  int stepFrames = 0;          ///< Frames between steering messages; the bus expects a fixed cadence.
  int deltaUpPerStep = 0;      ///< Maximum increase of torque per step [cNm].
  int deltaDownPerStep = 0;    ///< Maximum decrease per step [cNm]; larger, since releasing is always safe.
  int driverAllowanceCNm = 0;  ///< Driver torque tolerated before the panda takes the rack back [cNm].
};

}  // namespace platform
}  // namespace adas
