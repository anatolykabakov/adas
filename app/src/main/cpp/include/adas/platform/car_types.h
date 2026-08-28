#pragma once

#include <cstdint>
#include <optional>

#include "car_state.pb.h"

namespace adas {
namespace platform {
/** What the controllers ask for. Torque in centinewton-metres when the platform actuates torque
 *  directly; `steer` is the same request normalised to [-1, 1]. The longitudinal request is an
 *  acceleration [m/s²] — the standard actuator of every ACC bus, and the same shape the lateral one
 *  has: a physical quantity the platform lays into its own frames. */
struct Actuators {
  float steer = 0.f;
  std::optional<int> steerTorqueCNm;
  std::optional<float> accelMs2;
};

/** Driver-facing state the car's own cluster shows. Kept minimal on purpose: anything a particular
 *  brand draws differently belongs in that brand's implementation, not here. */
struct HudState {
  bool leftLaneVisible = false;
  bool rightLaneVisible = false;
  bool leftLaneDepart = false;
  bool rightLaneDepart = false;
  int visualAlert = 0;
  /// ACC cluster: the set speed and whether a lead is followed. Zero set speed draws nothing.
  float setSpeedMps = 0.f;
  bool leadVisible = false;
};

/** Where the longitudinal law is in its own state machine — the bus encodes stopping/starting
 *  differently from a plain acceleration, so the platform has to know. Mirrors LongCtrlState. */
enum class LongState : int {
  Off = 0,
  Pid = 1,
  Stopping = 2,
  Starting = 3,
};

struct CarControl {
  bool latActive = false;
  bool longActive = false;
  LongState longState = LongState::Off;
  Actuators actuators;
  HudState hud;
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

/** What a car is, as facts about the vehicle rather than settings. */
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

/** The acceleration envelope the panda enforces, same contract as the steering one: the control law
 *  clamps to it first, so a frame is never cut on the board. */
struct LongLimits {
  float accelMaxMs2 = 0.f;   ///< Largest acceleration the bus accepts [m/s²].
  float accelMinMs2 = 0.f;   ///< Largest deceleration [m/s²], negative.
  int stepFrames = 0;        ///< Frames between acceleration messages.
  bool supported = false;    ///< False when this platform has no longitudinal actuator at all.
};

}  // namespace platform
}  // namespace adas
