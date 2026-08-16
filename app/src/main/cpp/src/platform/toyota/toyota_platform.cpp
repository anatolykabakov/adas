#include "adas/platform/toyota/toyota_platform.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "adas/utils/logger.h"

namespace adas {
namespace platform {
namespace toyota {
namespace {
constexpr double kKphToMs = 1.0 / 3.6;

/// Driver torque above which the wheel counts as held, in the EPS's units.
constexpr double kSteeringPressedThreshold = 100.0;
}  // namespace

ToyotaTss2::ToyotaTss2(std::string dbc_path, const adas::SpeedFilter::Config& speed_filter)
  : dbc_path_(std::move(dbc_path)), speed_filter_(speed_filter)
{
}

void ToyotaTss2::init()
{
  if (dbc_path_.empty()) {
    LOGW("ToyotaTss2: no DBC path — the bus cannot be decoded");
    return;
  }
  try {
    dbc_ = std::make_unique<DBSParser>(dbc_path_);
    LOGI("ToyotaTss2: DBC %s (%zu messages)", dbc_path_.c_str(), dbc_->getAllMessages().size());
  } catch (const std::exception& e) {
    LOGE("ToyotaTss2: DBC load failed: %s", e.what());
  }
}

VehicleDefaults ToyotaTss2::defaults() const
{
  // Corolla TSS2, from `selfdrive/car/toyota/interface.py`. A different Toyota is a different entry
  // there and would be a different platform name here, not a config edit.
  VehicleDefaults v;
  v.wheelbase_m = 2.70;
  v.steer_ratio = 13.9;
  v.mass_kg = 1370.0;
  v.center_to_front_frac = 0.45;
  // Toyota's STEER_TORQUE_CMD is positive to the left, like our command; MQB is the other way round.
  v.steer_sign = 1.0;
  v.tire_stiffness_factor = 0.7933;
  v.max_steer_deg = 20.0;
  return v;
}

bool ToyotaTss2::isAllowedRxAddress(uint32_t address) const
{
  switch (address) {
    case Addresses::kSteerAngleSensor:
    case Addresses::kWheelSpeeds:
    case Addresses::kBrakeModule:
    case Addresses::kEpsStatus:
    case Addresses::kPcmCruise:
    case Addresses::kPcmCruise2:
    case Addresses::kGasPedal:
    case Addresses::kGearPacket:
      return true;
    default:
      return false;
  }
}

bool ToyotaTss2::update(const adas::proto::CANData& msg, int64_t now_ms)
{
  if (!dbc_)
    return false;

  dirty_ = false;
  for (const auto& f : msg.frames()) {
    can_frame frame;
    frame.address = static_cast<long>(f.address());
    frame.dat = f.data();
    frame.busTime = static_cast<long>(f.bus_time());
    frame.src = static_cast<long>(f.src());
    decodeFrame(frame);
  }
  if (dirty_)
    state_.set_timestamp(now_ms);
  return dirty_;
}

bool ToyotaTss2::decodeFrame(const can_frame& frame)
{
  const auto address = static_cast<uint32_t>(frame.address);
  switch (address) {
    case Addresses::kWheelSpeeds: {
      // The four wheels are reported in km/h with a −67.67 offset already applied by the DBC.
      const auto fl = dbc_->extractSignal(frame, "WHEEL_SPEED_FL");
      const auto fr = dbc_->extractSignal(frame, "WHEEL_SPEED_FR");
      const auto rl = dbc_->extractSignal(frame, "WHEEL_SPEED_RL");
      const auto rr = dbc_->extractSignal(frame, "WHEEL_SPEED_RR");
      if (!fl || !fr || !rl || !rr)
        return false;
      const double v_raw = (*fl + *fr + *rl + *rr) * 0.25 * kKphToMs;
      speed_filter_.update(v_raw, 0.01);
      state_.set_v_ego_raw(static_cast<float>(v_raw));
      state_.set_v_ego(static_cast<float>(speed_filter_.speed()));
      state_.set_standstill(v_raw < 0.1);
      dirty_ = true;
      return true;
    }
    case Addresses::kSteerAngleSensor: {
      const auto angle = dbc_->extractSignal(frame, "STEER_ANGLE");
      const auto fraction = dbc_->extractSignal(frame, "STEER_FRACTION");
      const auto rate = dbc_->extractSignal(frame, "STEER_RATE");
      if (angle) {
        state_.set_steering_angle_deg(static_cast<float>(*angle + (fraction ? *fraction : 0.0)));
        dirty_ = true;
      }
      if (rate)
        state_.set_steering_rate_deg(static_cast<float>(*rate));
      return true;
    }
    case Addresses::kEpsStatus: {
      // The driver's own torque, which is also what the panda's safety model watches.
      const auto torque = dbc_->extractSignal(frame, "STEER_TORQUE_DRIVER");
      if (torque) {
        eps_torque_ = *torque;
        state_.set_steering_torque(static_cast<float>(*torque));
        state_.set_steering_pressed(std::abs(*torque) > kSteeringPressedThreshold);
        dirty_ = true;
      }
      return true;
    }
    case Addresses::kPcmCruise: {
      const auto active = dbc_->extractSignal(frame, "CRUISE_ACTIVE");
      const auto standstill = dbc_->extractSignal(frame, "STANDSTILL_ON");
      if (active) {
        state_.set_cruise_engaged(*active > 0.5);
        dirty_ = true;
      }
      if (standstill)
        state_.set_standstill(*standstill > 0.5);
      return true;
    }
    case Addresses::kPcmCruise2: {
      const auto available = dbc_->extractSignal(frame, "MAIN_ON");
      if (available) {
        state_.set_cruise_available(*available > 0.5);
        dirty_ = true;
      }
      return true;
    }
    case Addresses::kGasPedal: {
      const auto gas = dbc_->extractSignal(frame, "GAS_PEDAL");
      if (gas) {
        state_.set_gas(static_cast<float>(*gas));
        state_.set_gas_pressed(*gas > 1e-3);
        dirty_ = true;
      }
      return true;
    }
    case Addresses::kBrakeModule: {
      const auto pressed = dbc_->extractSignal(frame, "BRAKE_PRESSED");
      if (pressed) {
        state_.set_brake_pressed(*pressed > 0.5);
        dirty_ = true;
      }
      return true;
    }
    case Addresses::kGearPacket: {
      const auto gear = dbc_->extractSignal(frame, "GEAR");
      if (gear) {
        // Toyota's enum: 0 park, 1 reverse, 2 neutral, 3 drive, 4 sport.
        const int g = static_cast<int>(*gear);
        state_.set_gear(g);
        dirty_ = true;
      }
      return true;
    }
    default:
      return false;
  }
}

CarStateView ToyotaTss2::stateView() const
{
  CarStateView v;
  v.vEgo = state_.v_ego();
  v.standstill = state_.standstill();
  v.steeringTorque = state_.steering_torque();
  v.steeringPressed = state_.steering_pressed();
  v.cruiseEngaged = state_.cruise_engaged();
  v.cruiseAvailable = state_.cruise_available();
  v.gearReverse = state_.gear() == 1;
  v.gearKnown = state_.gear() != 0 || state_.v_ego() > 0.1f;
  v.gasPressed = state_.gas_pressed();
  v.brakePressed = state_.brake_pressed();
  return v;
}

SteerLimits ToyotaTss2::steerLimits() const
{
  SteerLimits lim;
  lim.maxTorqueCNm = CarControllerParams::STEER_MAX;
  lim.stepFrames = CarControllerParams::STEER_STEP;
  lim.deltaUpPerStep = CarControllerParams::STEER_DELTA_UP;
  lim.deltaDownPerStep = CarControllerParams::STEER_DELTA_DOWN;
  lim.driverAllowanceCNm = CarControllerParams::STEER_DRIVER_ALLOWANCE;
  return lim;
}

void ToyotaTss2::setCruiseIntent(int /*intent*/, int64_t /*now_ms*/)
{
  // Toyota is driven by acceleration through ACC_CONTROL, not by pressing the stalk. Until the
  // longitudinal side is ported there is no button to press, and pretending otherwise would put an
  // MQB idea on a bus that has no place for it.
}

void ToyotaTss2::setLastCommand(int torque_cnm, bool lat_active)
{
  log_torque_cnm_ = torque_cnm;
  log_lat_active_ = lat_active;
}

uint8_t ToyotaTss2::checksum(uint32_t address, const uint8_t* data, size_t len)
{
  unsigned int sum = static_cast<unsigned int>(len);
  uint32_t addr = address;
  while (addr != 0) {
    sum += addr & 0xFF;
    addr >>= 8;
  }
  for (size_t i = 0; i + 1 < len; i++)
    sum += data[i];
  return static_cast<uint8_t>(sum & 0xFF);
}

std::vector<can_frame> ToyotaTss2::apply(const CarControl& cc)
{
  std::vector<can_frame> out;
  if (!sendingAllowed()) {
    // The frame below is believed correct and has never met an EPS. Building it and dropping it keeps
    // the code exercised and honest at the same time; flipping the switch is a decision to be made
    // with the car in front of you, not a default.
    return out;
  }

  int steer = 0;
  if (cc.latActive && cc.actuators.steerTorqueCNm) {
    steer = std::clamp(*cc.actuators.steerTorqueCNm, -CarControllerParams::STEER_MAX, CarControllerParams::STEER_MAX);
    const int up = CarControllerParams::STEER_DELTA_UP;
    const int down = CarControllerParams::STEER_DELTA_DOWN;
    const int delta = steer - last_steer_;
    const int rise = std::abs(steer) > std::abs(last_steer_) ? up : down;
    steer = last_steer_ + std::clamp(delta, -rise, rise);
  }
  last_steer_ = steer;

  // STEERING_LKA, five bytes: [0] counter and SET_ME_1, [1..2] torque big-endian, [3] LKA_STATE,
  // [4] checksum. Layout from `opendbc/toyota_nodsu_pt_generated.dbc`.
  uint8_t data[5] = {0, 0, 0, 0, 0};
  static uint8_t counter = 0;
  counter = static_cast<uint8_t>((counter + 1) & 0x3F);
  data[0] = static_cast<uint8_t>(0x80 | counter | (cc.latActive ? 0x01 : 0x00));
  const auto torque = static_cast<int16_t>(steer);
  data[1] = static_cast<uint8_t>((torque >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>(torque & 0xFF);
  data[3] = 0;
  data[4] = checksum(Addresses::kSteeringLka, data, sizeof(data));

  can_frame frame;
  frame.address = Addresses::kSteeringLka;
  frame.dat.assign(reinterpret_cast<const char*>(data), sizeof(data));
  frame.busTime = 0;
  frame.src = 0;
  out.push_back(frame);
  return out;
}

bool ToyotaTss2::lateralActuationAllowed() const
{
  if (!sendingAllowed())
    return false;
  return controls_allowed_;
}

void ToyotaTss2::configureSafety() { safety_configured_ = false; }

std::optional<health_t> ToyotaTss2::safetyTick(::Panda& panda, health_t health, int64_t /*now_ms*/)
{
  ignition_ = health.ignition_line_pkt != 0 || health.ignition_can_pkt != 0;
  controls_allowed_ = health.controls_allowed_pkt != 0;
  safety_mode_ = health.safety_mode_pkt;

  if (!safety_configured_ && ignition_) {
    try {
      // The EPS scale is a parameter of Toyota's safety model, not a flag: the panda compares driver
      // torque against it, so the wrong value moves the threshold at which it takes the rack back.
      panda.set_safety_model(SafetyConstants::kToyota, SafetyConstants::kEpsScale);
      safety_configured_ = true;
      LOGI("ToyotaTss2: safety model %u, eps scale %u", SafetyConstants::kToyota, SafetyConstants::kEpsScale);
    } catch (const std::exception& e) {
      LOGE("ToyotaTss2: set_safety_model failed: %s", e.what());
    }
  }
  return health;
}

void ToyotaTss2::enterSafeMode(::Panda& panda)
{
  try {
    panda.set_safety_model(SafetyConstants::kNoOutput, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } catch (...) {
    LOGE("ToyotaTss2: failed to put the panda into safe mode on shutdown");
  }
}

}  // namespace toyota
}  // namespace platform
}  // namespace adas
