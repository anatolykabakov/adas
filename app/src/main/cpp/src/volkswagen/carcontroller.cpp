#include "volkswagen/carcontroller.h"

#include <algorithm>
#include <cmath>

namespace volkswagen {

int applyDriverSteerTorqueLimits(int apply_torque, float driver_torque, int apply_steer_last)
{
  using P = CarControllerParams;
  const float driver_max_torque =
      P::STEER_MAX + (P::STEER_DRIVER_ALLOWANCE + driver_torque * P::STEER_DRIVER_FACTOR) * P::STEER_DRIVER_MULTIPLIER;
  const float driver_min_torque =
      -P::STEER_MAX +
      (-P::STEER_DRIVER_ALLOWANCE + driver_torque * P::STEER_DRIVER_FACTOR) * P::STEER_DRIVER_MULTIPLIER;
  const float max_steer_allowed = std::max(std::min(static_cast<float>(P::STEER_MAX), driver_max_torque), 0.f);
  const float min_steer_allowed = std::min(std::max(static_cast<float>(-P::STEER_MAX), driver_min_torque), 0.f);
  apply_torque =
      static_cast<int>(std::round(std::clamp(static_cast<float>(apply_torque), min_steer_allowed, max_steer_allowed)));

  if (apply_steer_last > 0) {
    const int lo = std::max(apply_steer_last - P::STEER_DELTA_DOWN, -P::STEER_DELTA_UP);
    const int hi = apply_steer_last + P::STEER_DELTA_UP;
    apply_torque = std::clamp(apply_torque, lo, hi);
  } else {
    const int lo = apply_steer_last - P::STEER_DELTA_UP;
    const int hi = std::min(apply_steer_last + P::STEER_DELTA_DOWN, P::STEER_DELTA_UP);
    apply_torque = std::clamp(apply_torque, lo, hi);
  }
  return apply_torque;
}

std::vector<can_frame> CarController::update(const CarControl& CC, const CarStateView& CS)
{
  std::vector<can_frame> can_sends;
  using P = CarControllerParams;

  if (frame_ % P::STEER_STEP == 0) {
    int apply_steer = 0;
    bool hca_enabled = false;

    const bool eps_ok = epsHcaAllowsSteer(CS.epsHcaStatus);
    const bool standstill = CS.standstill || std::abs(CS.vEgo) < 0.3f;
    const bool lat_active = CC.latActive && eps_ok && !standstill;

    if (lat_active) {
      int new_steer = 0;
      if (CC.actuators.steerTorqueCNm.has_value()) {
        new_steer = *CC.actuators.steerTorqueCNm;
      } else {
        new_steer = static_cast<int>(std::round(CC.actuators.steer * P::STEER_MAX));
      }
      apply_steer = applyDriverSteerTorqueLimits(new_steer, CS.steeringTorque, apply_steer_last_);

      if (apply_steer == 0) {
        hca_enabled = false;
        hca_enabled_frame_count_ = 0;
      } else {
        hca_enabled_frame_count_ += 1;

        if (hca_enabled_frame_count_ >= 118 * (100 / P::STEER_STEP)) {
          hca_enabled = false;
          hca_enabled_frame_count_ = 0;
        } else {
          hca_enabled = true;
          if (apply_steer_last_ == apply_steer) {
            hca_same_torque_count_ += 1;
            if (hca_same_torque_count_ > static_cast<int>(1.9 * (100 / P::STEER_STEP))) {
              apply_steer -= (apply_steer < 0) ? -1 : 1;
              hca_same_torque_count_ = 0;
            }
          } else {
            hca_same_torque_count_ = 0;
          }
        }
      }
    } else {
      hca_enabled = false;
      apply_steer = 0;
      hca_enabled_frame_count_ = 0;
    }

    apply_steer_last_ = apply_steer;
    can_sends.push_back(create_steering_control(CanBus::pt, apply_steer, hca_enabled, &hca_counter_));
  }

  if (frame_ % P::LDW_STEP == 0) {
    int hud_alert = 0;
    if (CC.visualAlert == P::LDW_MSG_TAKE_OVER) {
      hud_alert = P::LDW_MSG_TAKE_OVER;
    }
    HudControl hud = CC.hud;
    if (CC.latActive) {
      if (!hud.leftLaneVisible && !hud.rightLaneVisible) {
        hud.leftLaneVisible = true;
        hud.rightLaneVisible = true;
      }
    }
    can_sends.push_back(
        create_lka_hud_control(CanBus::pt, CS.ldwStock, CC.latActive, CS.steeringPressed, hud_alert, hud));
  }

  if (CC.cruise.send && CS.graStock.valid) {
    const uint8_t cnt = CS.graStock.counter();
    if (cnt != gra_acc_counter_last_) {
      const int bus = CanBus::pt;
      can_sends.push_back(create_acc_buttons_control(bus, CS.graStock, CC.cruise));
    }
    gra_acc_counter_last_ = cnt;
  } else if (CS.graStock.valid) {
    gra_acc_counter_last_ = CS.graStock.counter();
  }

  frame_ += 1;
  return can_sends;
}

}  // namespace volkswagen
