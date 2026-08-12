#include "adas/platform/volkswagen/mqb_car_state_decoder.h"

#include <cmath>
#include <cstring>
#include <set>

#include "adas/utils/logger.h"

namespace volkswagen {
namespace {
const std::set<uint32_t> kAllowedCanFrames = {
    0x0B2, 0x086, 0x09F, 0x0AD, 0x0FD, 0x101, 0x106, 0x120, 0x121,
    0x122, 0x126, 0x12B, 0x12E, 0x30C, 0x397, 0x3BE, 0x3DB,
};

constexpr double kKphToMs = 1.0 / 3.6;
constexpr double kDegToRad = 0.017453292519943295;

double signedSignal(double mag, double sign_bit) { return mag * (sign_bit > 0.5 ? -1.0 : 1.0); }

}  // namespace

bool isAllowedMqbRxAddress(uint32_t address) { return kAllowedCanFrames.count(address) != 0; }

CarStateView MqbCarStateDecoder::toCarStateView() const
{
  CarStateView cs;
  cs.vEgo = state_.v_ego();
  cs.standstill = state_.standstill() || (std::abs(state_.v_ego()) < 0.3f);
  cs.steeringTorque = state_.steering_torque();
  cs.steeringPressed = state_.steering_pressed();
  cs.epsHcaStatus = eps_hca_status_;
  cs.ldwStock = ldw_stock_;
  cs.graStock = gra_stock_;
  cs.cruiseEngaged = state_.cruise_engaged();
  cs.cruiseAvailable = state_.cruise_available();
  cs.gearReverse = gearIsReverse(state_.gear());
  cs.gearKnown = state_.gear() != 0;
  cs.gasPressed = state_.gas_pressed();
  cs.brakePressed = state_.brake_pressed();
  return cs;
}

void MqbCarStateDecoder::updateFromFrame(const can_frame& frame, int64_t now_ms)
{
  if (!dbc_)
    return;

  switch (frame.address) {
    case 0x3DB: {
      auto li = dbc_->extractSignal(frame, "BH_Blinker_li");
      auto re = dbc_->extractSignal(frame, "BH_Blinker_re");
      if (li && re) {
        state_.set_left_blinker(*li > 0.5);
        state_.set_right_blinker(*re > 0.5);
        dirty_ = true;
      }
      break;
    }
    case 0x0B2: {
      auto fl = dbc_->extractSignal(frame, "ESP_VL_Radgeschw_02");
      auto fr = dbc_->extractSignal(frame, "ESP_VR_Radgeschw_02");
      auto rl = dbc_->extractSignal(frame, "ESP_HL_Radgeschw_02");
      auto rr = dbc_->extractSignal(frame, "ESP_HR_Radgeschw_02");
      if (fl && fr && rl && rr) {
        const float fl_ms = static_cast<float>(*fl * kKphToMs);
        const float fr_ms = static_cast<float>(*fr * kKphToMs);
        const float rl_ms = static_cast<float>(*rl * kKphToMs);
        const float rr_ms = static_cast<float>(*rr * kKphToMs);
        auto* ws = state_.mutable_wheel_speeds();
        ws->set_fl(fl_ms);
        ws->set_fr(fr_ms);
        ws->set_rl(rl_ms);
        ws->set_rr(rr_ms);

        const double v_raw = (fl_ms + fr_ms + rl_ms + rr_ms) * 0.25;
        const int64_t now = now_ms;
        const double dt_s = prev_v_ts_ms_ > 0 && now > prev_v_ts_ms_ ? (now - prev_v_ts_ms_) * 1e-3 : 0.0;
        prev_v_ts_ms_ = now;
        speed_filter_.update(v_raw, dt_s);

        state_.set_v_ego_raw(static_cast<float>(v_raw));
        state_.set_v_ego(static_cast<float>(speed_filter_.speed()));
        state_.set_a_ego(static_cast<float>(speed_filter_.accel()));
        state_.set_standstill(v_raw < 0.05);
        dirty_ = true;
      }
      break;
    }
    case 0x086: {
      auto angle = dbc_->extractSignal(frame, "LWI_Lenkradwinkel");
      auto asign = dbc_->extractSignal(frame, "LWI_VZ_Lenkradwinkel");
      auto rate = dbc_->extractSignal(frame, "LWI_Lenkradw_Geschw");
      auto rsign = dbc_->extractSignal(frame, "LWI_VZ_Lenkradw_Geschw");
      if (angle && asign) {
        state_.set_steering_angle_deg(static_cast<float>(signedSignal(*angle, *asign)));
        dirty_ = true;
      }
      if (rate && rsign) {
        state_.set_steering_rate_deg(static_cast<float>(signedSignal(*rate, *rsign)));
        dirty_ = true;
      }
      break;
    }
    case 0x09F: {
      auto torq = dbc_->extractSignal(frame, "EPS_Lenkmoment");
      auto tsign = dbc_->extractSignal(frame, "EPS_VZ_Lenkmoment");
      if (torq && tsign) {
        const double t = signedSignal(*torq, *tsign);
        state_.set_steering_torque(static_cast<float>(t));
        state_.set_steering_pressed(std::abs(t) > CarControllerParams::STEER_DRIVER_ALLOWANCE);
        dirty_ = true;
      }
      if (auto hca_st = dbc_->extractSignal(frame, "EPS_HCA_Status")) {
        eps_hca_status_ = static_cast<uint8_t>(*hca_st);
      }
      break;
    }
    case 0x101: {
      auto yaw = dbc_->extractSignal(frame, "ESP_Gierrate");
      auto ysign = dbc_->extractSignal(frame, "ESP_VZ_Gierrate");
      if (yaw && ysign) {
        state_.set_yaw_rate(static_cast<float>(signedSignal(*yaw, *ysign) * kDegToRad));
        dirty_ = true;
      }
      break;
    }
    case 0x121: {
      if (auto gas = dbc_->extractSignal(frame, "MO_Fahrpedalrohwert_01")) {
        const float g = static_cast<float>(*gas / 100.0);
        state_.set_gas(g);
        state_.set_gas_pressed(g > 0.f);
        dirty_ = true;
      }
      break;
    }
    case 0x106: {
      if (auto pressure = dbc_->extractSignal(frame, "ESP_Bremsdruck")) {
        state_.set_brake(static_cast<float>(*pressure / 250.0));
        dirty_ = true;
      }
      if (auto driver_brake = dbc_->extractSignal(frame, "ESP_Fahrer_bremst")) {
        brake_esp_ = *driver_brake > 0.5;
        state_.set_brake_pressed(brake_esp_ || brake_motor_);
        dirty_ = true;
      }
      break;
    }
    case 0x3BE: {
      if (auto pedal = dbc_->extractSignal(frame, "MO_Fahrer_bremst")) {
        brake_motor_ = *pedal > 0.5;
        state_.set_brake_pressed(brake_esp_ || brake_motor_);
        dirty_ = true;
      }
      break;
    }
    case 0x0AD: {
      if (auto gear = dbc_->extractSignal(frame, "GE_Fahrstufe")) {
        state_.set_gear(static_cast<int32_t>(*gear));
        dirty_ = true;
      }
      break;
    }
    case 0x12B: {
      if (frame.dat.size() >= 8) {
        std::memcpy(gra_stock_.data, frame.dat.data(), 8);
        gra_stock_.valid = true;
      }
      auto set_btn = [&](const char* name, void (adas::proto::CarState::*setter)(bool)) {
        if (auto v = dbc_->extractSignal(frame, name)) {
          (state_.*setter)(*v > 0.5);
          dirty_ = true;
        }
      };
      set_btn("GRA_Hauptschalter", &adas::proto::CarState::set_cruise_main_switch);
      set_btn("GRA_Tip_Setzen", &adas::proto::CarState::set_cruise_set);
      set_btn("GRA_Tip_Wiederaufnahme", &adas::proto::CarState::set_cruise_resume);
      set_btn("GRA_Abbrechen", &adas::proto::CarState::set_cruise_cancel);
      set_btn("GRA_Tip_Hoch", &adas::proto::CarState::set_cruise_accel);
      set_btn("GRA_Tip_Runter", &adas::proto::CarState::set_cruise_decel);
      if (auto gap = dbc_->extractSignal(frame, "GRA_Verstellung_Zeitluecke")) {
        state_.set_cruise_gap_adjust(static_cast<int32_t>(*gap));
        dirty_ = true;
      }
      break;
    }
    case 0x397: {
      if (frame.dat.size() >= 8 && (frame.src & 0x80) == 0) {
        std::memcpy(ldw_stock_.data, frame.dat.data(), 8);
        ldw_stock_.valid = true;
      }
      break;
    }
    case 0x120: {
      if (auto st = dbc_->extractSignal(frame, "TSK_Status")) {
        const int status = static_cast<int>(*st);
        if (status != last_tsk_status_) {
          LOGW("TSK_Status %d → %d (pcm cruise %s; panda allowed needs rising edge 3/4/5)", last_tsk_status_, status,
               cruiseEngagedFromTsk(status) ? "engaged" : "NOT engaged");
          last_tsk_status_ = status;
        }
        state_.set_acc_status(status);
        const bool engaged = cruiseEngagedFromTsk(status);
        state_.set_cruise_engaged(engaged);
        state_.set_cruise_available(cruiseAvailableFromTsk(status));
        dirty_ = true;
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace volkswagen
