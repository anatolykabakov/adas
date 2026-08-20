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

/// panda's driver-torque limit. \return The torque after clamping against the driver's input.
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
  bool cruiseAvailable = false;
  bool gearReverse = false;
  bool gearKnown = false;
  bool gasPressed = false;
  bool brakePressed = false;
};

/** Whether lateral torque may go to the rack this frame. */
inline bool lateralActuationAllowed(bool controls_allowed, bool always_on, const CarStateView& cs)
{
  if (controls_allowed)
    return true;
  if (!always_on)
    return false;
  return cs.cruiseAvailable && cs.gearKnown && !cs.gearReverse;
}

/** Turns an actuation request into MQB CAN frames. */
class CarController {
public:
  CarController() = default;

  /// One 100 Hz tick. \return HCA (and HUD/button) frames to send this tick.
  std::vector<can_frame> update(const CarControl& CC, const CarStateView& CS);

  /// \return Torque last commanded [unit].
  int applySteerLast() const { return apply_steer_last_; }
  /// \return Tick counter, drives the 2-frame HCA step.
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
