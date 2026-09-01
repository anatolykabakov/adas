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
  /// Longitudinal request [m/s²]; absent means the acceleration frames say "inactive".
  std::optional<float> accelMs2;
};

/// panda's driver-torque limit. \return The torque after clamping against the driver's input.
int applyDriverSteerTorqueLimits(int apply_torque, float driver_torque, int apply_steer_last);

/// Where the longitudinal law is; the bus encodes stopping and starting as their own bits.
enum class LongCtrlState : int { Off = 0, Pid = 1, Stopping = 2, Starting = 3 };

struct CarControl {
  bool latActive = false;
  bool longActive = false;
  LongCtrlState longState = LongCtrlState::Off;
  Actuators actuators;
  HudControl hud;
  int visualAlert = 0;
  float setSpeedMps = 0.f;  ///< For the cluster; 0 draws no set speed.
  bool leadVisible = false;
};

struct CarStateView {
  float vEgo = 0.f;
  bool standstill = false;
  float steeringTorque = 0.f;
  bool steeringPressed = false;
  uint8_t epsHcaStatus = 0;
  LdwStockValues ldwStock;
  bool cruiseEngaged = false;
  bool cruiseAvailable = false;
  bool gearReverse = false;
  bool gearKnown = false;
  bool gasPressed = false;
  bool brakePressed = false;
  uint8_t accType = 0;  ///< Stock radar's ACC_Typ, mirrored into our ACC_06.
  bool espHold = false;  ///< ESP confirms it is holding the car.
  int tskStatus = -1;    ///< TSK_Status as last seen; 6/7 is a fault.
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

/** Whether an acceleration request may go out this frame. Unlike torque there is no always-on
 *  variant: the panda has to have engaged (controls_allowed), the ACC main switch has to be on, and
 *  the driver's brake overrides everything. */
inline bool longitudinalActuationAllowed(bool controls_allowed, const CarStateView& cs)
{
  return controls_allowed && cs.cruiseAvailable && !cs.brakePressed && cs.gearKnown && !cs.gearReverse;
}

/** Turns an actuation request into MQB CAN frames. */
class CarController {
public:
  CarController() = default;

  /// Whether to emit the ACC frames at all. Off leaves the stock ACC in charge of speed.
  void setLongControlEnabled(bool on) { long_control_enabled_ = on; }
  /// \return See setLongControlEnabled.
  bool longControlEnabled() const { return long_control_enabled_; }

  /// One 100 Hz tick. \return HCA, ACC and HUD frames to send this tick.
  std::vector<can_frame> update(const CarControl& CC, const CarStateView& CS);

  /// \return Torque last commanded [unit].
  int applySteerLast() const { return apply_steer_last_; }
  /// \return Acceleration last put on the bus [m/s²], or the inactive value when nothing was asked.
  float applyAccelLast() const { return apply_accel_last_; }
  /// \return Tick counter, drives the 2-frame HCA step.
  int frame() const { return frame_; }

private:
  int apply_steer_last_ = 0;
  float apply_accel_last_ = CarControllerParams::ACCEL_INACTIVE;
  int frame_ = 0;
  int hca_same_torque_count_ = 0;
  int hca_enabled_frame_count_ = 0;
  uint8_t hca_counter_ = 0;
  uint8_t acc06_counter_ = 0;
  uint8_t acc07_counter_ = 0;
  uint8_t acc02_counter_ = 0;
  bool long_control_enabled_ = false;
};

}  // namespace volkswagen
