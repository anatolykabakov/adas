#include "adas/services/control.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/adas_topics.h"
#include "adas/utils/logger.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
Control::Control(Config config) : config_(std::move(config)), ctl_(config_.ctl), long_ctl_(config_.long_ctl) {}

void Control::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& cs) {
    chassis_ = carStateToChassis(cs, ctl_.steerRatio());
    car_state_ = cs;
  });
  subscribe<adas::proto::PandaHealth>(topics::kPandaHealth, [this](const adas::proto::PandaHealth& h) { onHealth(h); });
  subscribe<adas::proto::LatPlan>(topics::kLatPlan, [this](const adas::proto::LatPlan& lp) {
    plan_ = lp;
    have_plan_ = true;
  });
  subscribe<adas::proto::LocalizationPose>(topics::kLocalizationPose, [this](const adas::proto::LocalizationPose& p) {
    {
      ctl_.setLearnedParams(p.learned_params_valid(), p.learned_stiffness_factor(), p.learned_steer_ratio(),
                            p.learned_angle_offset_deg());
    }
    road_roll_valid_ = p.road_roll_valid();
    road_roll_rad_ = road_roll_valid_ ? p.road_roll_deg() * M_PI / 180.0 : 0.0;
  });
  subscribe<adas::proto::LongPlanState>(topics::kLongPlan, [this](const adas::proto::LongPlanState& lp) {
    long_plan_ = lp;
    have_long_plan_ = true;
    long_plan_rx_us_ = static_cast<int64_t>(now());
  });

  ctl_.setSlewConfig(config_.slew);
  registerParameters();

  scheduleTimer(10, [this] { latTick(); }, "control");

  LOGI("Control: %s + %s + %s → %s (longitudinal %s)", topics::kVehicleState, topics::kLatPlan, topics::kLongPlan,
       topics::kSteerCommand, config_.long_control_enabled ? "on" : "off");
}

void Control::onHealth(const adas::proto::PandaHealth& h)
{
  assist_gate_.onReport(h.lat_actuation_allowed(), static_cast<int64_t>(now()));
  controls_gate_.onReport(h.controls_allowed(), static_cast<int64_t>(now()));
}

lateral::VehicleParams Control::vehicleParams() const
{
  lateral::VehicleParams v;
  v.wheelbase_m = config_.wheelbase_m;
  v.steer_ratio = ctl_.effectiveSteerRatio();
  v.tire_stiffness_factor = ctl_.effectiveStiffnessFactor();
  v.use_vehicle_model = config_.lat_use_vehicle_model;
  v.steer_sign = config_.ctl.steer_sign < 0.0 ? -1.0 : 1.0;
  v.road_roll_rad = road_roll_rad_;
  v.road_roll_valid = road_roll_valid_;
  return v;
}

void Control::registerParameters()
{
  registerParameter<double>(
      "steer_ratio", [this](const double& v) { ctl_.setSteerRatio(v); }, [this] { return ctl_.steerRatio(); });
  registerParameter<double>(
      "max_steer_deg", [this](const double& v) { ctl_.setMaxSteerDeg(v); },
      [this] { return ctl_.maxSteerRad() * 180.0 / M_PI; });
  registerParameter<double>(
      "tire_stiffness_factor", [this](const double& v) { ctl_.setTireStiffnessFactor(v); },
      [this] { return ctl_.effectiveStiffnessFactor(); });
  registerParameter<bool>("lat_use_vehicle_model", config_.lat_use_vehicle_model);
  registerParameter<double>(
      "steer_slew_limit_deg",
      [this](const double& v) {
        config_.slew.limit_deg = v;
        ctl_.setSlewConfig(config_.slew);
      },
      [this] { return config_.slew.limit_deg; });
  registerParameter<double>(
      "pid_kp",
      [this](const double& v) {
        config_.ctl.pid_kp = v;
        ctl_.setPidGains(config_.ctl.pid_kp, config_.ctl.pid_ki, config_.ctl.pid_kf);
      },
      [this] { return config_.ctl.pid_kp; });
  registerParameter<double>(
      "pid_ki",
      [this](const double& v) {
        config_.ctl.pid_ki = v;
        ctl_.setPidGains(config_.ctl.pid_kp, config_.ctl.pid_ki, config_.ctl.pid_kf);
      },
      [this] { return config_.ctl.pid_ki; });
  registerParameter<double>(
      "pid_kf",
      [this](const double& v) {
        config_.ctl.pid_kf = v;
        ctl_.setPidGains(config_.ctl.pid_kp, config_.ctl.pid_ki, config_.ctl.pid_kf);
      },
      [this] { return config_.ctl.pid_kf; });
  registerParameter<double>("lane_max_age_s", config_.lane_max_age_s);
  registerParameter<double>("lka_blinker_resume_delay_s", config_.lka_blinker_resume_delay_s);
  registerParameter<double>("assist_max_age_s", config_.assist_max_age_s);
  registerParameter<bool>("long_control_enabled", config_.long_control_enabled);
  registerParameter<double>("long_plan_max_age_s", config_.long_plan_max_age_s);
}

std::string Control::longGate(int64_t now_us)
{
  if (!config_.long_control_enabled)
    return "off";
  if (!have_long_plan_ || long_plan_.speeds_size() != longitudinal::kControlN)
    return long_plan_.status() == "no_cruise" ? "no_cruise" : "no_plan";
  if ((now_us - long_plan_rx_us_) * 1e-6 > config_.long_plan_max_age_s)
    return "stale";
  const auto c = controls_gate_.update(now_us, config_.assist_max_age_s);
  if (!(c.known && c.allowed))
    return "no_controls";
  if (car_state_.brake_pressed())
    return "brake";
  if (car_state_.gas_pressed())
    return "gas";
  return "ok";
}

Control::LatGates Control::gates(const ChassisSample& ch, int64_t now_us)
{
  LatGates g;
  const auto a = assist_gate_.update(now_us, config_.assist_max_age_s);
  g.assist_allowed = a.known && a.allowed;
  g.assist_known = a.known;
  if (!have_plan_)
    return g;

  std::string status = plan_.valid() ? "ok" : plan_.status();

  double age_s = 0.0;
  if (status == "ok" && !stale_gate_.update(now_us, plan_.capture_ts_ms() * 1000, config_.lane_max_age_s, age_s)) {
    status = "stale";
    if (stale_gate_.justChanged())
      LOGW("Control: plan is stale (%.0f ms > %.0f) — command withdrawn", age_s * 1e3, config_.lane_max_age_s * 1e3);
  }

  const auto b = blinker_gate_.update(now_us, ch.left_blinker, ch.right_blinker, config_.lka_blinker_resume_delay_s);
  if (b.suppressed && status == "ok")
    status = "blinker";
  if (b.changed)
    LOGI("Control: blinker %s", b.suppressed ? "on — steering handed to the driver" : "off — steering resumed");

  if (!a.allowed && status == "ok") {
    status = "no_assist";
    if (a.changed)
      LOGW("Control: panda blocks torque (allowed=%d known=%d) — command withdrawn",
           static_cast<int>(assist_gate_.lastReportAllowed()), static_cast<int>(a.known));
  }
  g.status = status;
  return g;
}

void Control::latTick()
{
  const int64_t now_us = static_cast<int64_t>(now());
  const auto& ch = chassis_;
  const auto g = gates(ch, now_us);
  const bool active = g.status == "ok";

  const auto vehicle = vehicleParams();
  const double kappa = lateral::curvatureWithRoll(plan_.desired_curvature(), ch.speed_mps, vehicle);
  double steer_rad = -steerFromCurvature(kappa, ch.speed_mps, config_.lf_m, lateral::slipFactorOrZero(vehicle));

  const bool slew_clipped = ctl_.applySlew(steer_rad, ch.speed_mps, frame_dt_s_, active);
  if (active)
    ctl_.setSetpointFromSteer(steer_rad);
  else
    ctl_.clearSetpoint();

  const auto lat = ctl_.update(active, ch);

  const int torque = active ? static_cast<int>(std::lround(lat.steer_norm * config_.max_torque_cnm)) : 0;

  // The longitudinal law on the same tick: the plan's speed trajectory, read at the actuator delay,
  // into an acceleration — or nothing, when a gate says so.
  const std::string long_status = longGate(now_us);
  longitudinal::LongControlInput li;
  li.active = long_status == "ok";
  li.v_ego = ch.speed_mps;
  li.brake_pressed = car_state_.brake_pressed();
  // upstream's `cruiseState.standstill` is the stock ACC's hold flag and exists only under pcmCruise;
  // with our own longitudinal there is none, so a stopped car is held by the plan (stopping state),
  // not by this flag — feeding the plain speed here made Starting and Stopping trade places every tick.
  li.standstill = false;
  li.t_since_plan_s = have_long_plan_ ? std::max(0.0, (now_us - long_plan_rx_us_) * 1e-6) : 0.0;
  if (li.active) {
    for (int i = 0; i < longitudinal::kControlN; ++i) {
      li.speeds[i] = long_plan_.speeds(i);
      li.accels[i] = long_plan_.accels(i);
    }
  }
  const auto lo = long_ctl_.update(li);

  SteerCommandInputs si;
  si.long_active = li.active && lo.state != longitudinal::LongCtrlState::Off;
  si.accel_ms2 = si.long_active ? lo.accel : 0.0;
  si.long_state = static_cast<int>(lo.state);
  si.v_target_mps = lo.v_target;
  si.v_cruise_mps = have_long_plan_ ? long_plan_.v_cruise() : 0.0;
  si.long_status = long_status;
  si.torque_cnm = torque;
  si.enabled = active;
  si.capture_ts_ms = plan_.capture_ts_ms();
  si.vision_ts_ms = plan_.infer_ts_ms();
  si.chassis_ts_ms = static_cast<int64_t>(ch.timestamp_us / 1000);
  si.publish_ts_ms = now_us / 1000;
  si.slew_clipped = slew_clipped;
  si.assist_allowed = g.assist_allowed;
  si.assist_known = g.assist_known;
  si.status = g.status;
  const auto cmd = createSteerCommand(si, lat);
  publish(topics::kSteerCommand, cmd);
}

}  // namespace services
}  // namespace adas
