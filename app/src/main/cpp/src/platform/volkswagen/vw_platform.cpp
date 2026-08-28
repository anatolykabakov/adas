#include "adas/platform/volkswagen/vw_platform.h"

#include <chrono>
#include <thread>
#include <utility>

#include "adas/platform/volkswagen/mqb_car_state_decoder.h"
#include "adas/platform/volkswagen/values.h"
#include "adas/utils/logger.h"

namespace adas {
namespace platform {
namespace volkswagen {
namespace {
using C = ::volkswagen::MqbSafetyConstants;
}  // namespace

VolkswagenMqb::VolkswagenMqb(std::string dbc_path, const adas::SpeedFilter::Config& speed_filter,
                             bool long_control_enabled)
  : ci_({std::move(dbc_path), speed_filter}), long_control_enabled_(long_control_enabled)
{
  ci_.setLongControlEnabled(long_control_enabled_);
}

void VolkswagenMqb::init() { ci_.init(); }

VehicleDefaults VolkswagenMqb::defaults() const
{
  // Golf 7 Highline, measured on our own car and refined from bags; the stiffness factor is where the
  // learner starts, not where it stays.
  VehicleDefaults v;
  v.wheelbase_m = 2.636;
  v.steer_ratio = 15.6;  // Fitted from bags; the factory figure is 15.7.
  v.mass_kg = 1533.0;
  v.center_to_front_frac = 0.45;
  v.steer_sign = -1.0;
  v.tire_stiffness_factor = 1.0;
  v.max_steer_deg = 20.0;
  return v;
}

bool VolkswagenMqb::isAllowedRxAddress(uint32_t address) const { return ::volkswagen::isAllowedMqbRxAddress(address); }

bool VolkswagenMqb::update(const adas::proto::CANData& msg, int64_t now_ms) { return ci_.update(msg, now_ms); }

CarStateView VolkswagenMqb::stateView() const
{
  const ::volkswagen::CarStateView cs = ci_.carStateView();
  CarStateView out;
  out.vEgo = cs.vEgo;
  out.standstill = cs.standstill;
  out.steeringTorque = cs.steeringTorque;
  out.steeringPressed = cs.steeringPressed;
  out.cruiseEngaged = cs.cruiseEngaged;
  out.cruiseAvailable = cs.cruiseAvailable;
  out.gearReverse = cs.gearReverse;
  out.gearKnown = cs.gearKnown;
  out.gasPressed = cs.gasPressed;
  out.brakePressed = cs.brakePressed;
  return out;
}

SteerLimits VolkswagenMqb::steerLimits() const
{
  using P = ::volkswagen::CarControllerParams;
  SteerLimits lim;
  lim.maxTorqueCNm = P::STEER_MAX;
  lim.stepFrames = P::STEER_STEP;
  lim.deltaUpPerStep = P::STEER_DELTA_UP;
  lim.deltaDownPerStep = P::STEER_DELTA_DOWN;
  lim.driverAllowanceCNm = P::STEER_DRIVER_ALLOWANCE;
  return lim;
}

LongLimits VolkswagenMqb::longLimits() const
{
  using P = ::volkswagen::CarControllerParams;
  LongLimits lim;
  lim.accelMaxMs2 = P::ACCEL_MAX;
  lim.accelMinMs2 = P::ACCEL_MIN;
  lim.stepFrames = P::ACC_CONTROL_STEP;
  lim.supported = long_control_enabled_;
  return lim;
}

void VolkswagenMqb::setLastCommand(int torque_cnm, bool lat_active)
{
  log_torque_cnm_ = torque_cnm;
  log_lat_active_ = lat_active;
}

std::vector<can_frame> VolkswagenMqb::apply(const CarControl& cc)
{
  const ::volkswagen::CarStateView cs = ci_.carStateView();

  ::volkswagen::CarControl out;
  out.latActive = cc.latActive;
  out.longActive = cc.longActive;
  out.longState = static_cast<::volkswagen::LongCtrlState>(static_cast<int>(cc.longState));
  out.actuators.steerTorqueCNm = cc.actuators.steerTorqueCNm;
  out.actuators.accelMs2 = cc.actuators.accelMs2;
  out.hud.leftLaneVisible = cc.hud.leftLaneVisible;
  out.hud.rightLaneVisible = cc.hud.rightLaneVisible;
  out.setSpeedMps = cc.hud.setSpeedMps;
  out.leadVisible = cc.hud.leadVisible;

  return ci_.apply(out, cs);
}

void VolkswagenMqb::configureSafety()
{
  safety_.setAlternativeExperience(C::kAltExpDisableDisengageOnGas | C::kAltExpAlka);
  // The longitudinal flag is the takeover itself: with it the panda stops forwarding the radar's
  // ACC frames. Without our own frames behind it the car would lose its stock ACC, so the two are one switch.
  safety_.setSafetyParam(long_control_enabled_ ? C::kParamLongControl : C::kParamStock);
}

std::optional<health_t> VolkswagenMqb::safetyTick(::Panda& panda, health_t health, int64_t now_ms)
{
  ::volkswagen::SafetyLogContext log;
  log.last_tsk_status = ci_.lastTskStatus();
  log.eps_hca_status = ci_.epsHcaStatus();
  log.apply_steer_last = ci_.applySteerLast();
  log.cruise_engaged = ci_.carState().cruise_engaged();
  log.brake_pressed = ci_.carState().brake_pressed();
  log.gas_pressed = ci_.carState().gas_pressed();
  log.hca_cmd_steer = log_torque_cnm_;
  log.lat_cmd_active = log_lat_active_;
  return safety_.tick(panda, health, now_ms, log);
}

void VolkswagenMqb::enterSafeMode(::Panda& panda)
{
  try {
    panda.set_safety_model(C::kNoOutput, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } catch (...) {
    LOGE("VolkswagenMqb: failed to put the panda into safe mode on shutdown");
  }
}

}  // namespace volkswagen
}  // namespace platform
}  // namespace adas
