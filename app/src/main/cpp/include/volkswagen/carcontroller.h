#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "panda/can_frame.h"
#include "volkswagen/mqbcan.h"
#include "volkswagen/values.h"

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
  bool gasPressed = false;
  bool brakePressed = false;
};

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
