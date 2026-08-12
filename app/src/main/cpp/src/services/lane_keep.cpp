#include "adas/services/lane_keep.h"

#include "adas/lateral/convert.hpp"
#include "adas/lateral/limits.hpp"
#include "adas/utils/proto_convert.h"

#include <cmath>

#include "messages.pb.h"
#include "adas/utils/logger.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace services {
namespace {
constexpr double kMinDtS = 0.02;
constexpr double kMaxDtS = 0.5;
constexpr double kMaxGapS = 0.5;
constexpr double kDtAlpha = 0.3;

/** Темп интегратора углового PID. Константа, а не измеренный интервал: у апстрима
 *  `PIDController(..., rate=100)` задаётся при создании и не меняется никогда, а `controlsd` крутится
 *  ровно на `DT_CTRL = 0.01`. Измеряемый темп добавлял зависимость команды от самого первого тика и
 *  от совпавших меток времени — расхождение с апстримом, которого у него нет. На отгружаемой
 *  конфигурации разница между двумя вариантами измерена и составляет 1 cNm ск.кв.: цена нулевая,
 *  поэтому берём их форму. */
constexpr double kPidRateHz = 100.0;

}  // namespace

LaneKeep::LaneKeep(Config p)
  : config_(std::move(p))
  , lat_(config_.pid_kp, config_.pid_ki, config_.pid_kf, kPidRateHz, config_.pid_ff_floor_mps)
  , max_steer_rad_(config_.max_steer_deg * M_PI / 180.0)
  , max_torque_cnm_(config_.max_torque_cnm)
  , steer_ratio_(std::max(config_.steer_ratio, 1e-3))
  , steer_sign_(config_.steer_sign < 0.0 ? -1.0 : 1.0)
  , steer_output_enabled_(config_.steer_output_enabled)
{
  if (config_.controller != "mpc" && config_.controller != "fp")
    config_.controller = "pp";

  frame_dt_ = IntervalFilter({config_.vision_nominal_dt_s, kMinDtS, kMaxDtS, kMaxGapS, kDtAlpha});
  speed_gate_.setThresholds(config_.min_control_speed_mps,
                            config_.min_control_speed_mps + std::max(0.0, config_.min_control_speed_hyst_mps));
}

void LaneKeep::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, steer_ratio_));
  });
  subscribe<adas::proto::LaneLines>(topics::kVisionLanes, [this](const adas::proto::LaneLines& payload) {
    const LanePathMsg path = laneLinesToPath(payload, config_.lane_path, &lane_fusion_);
    publish(topics::kVisionPath, createLanePath(path));
    onLanes(path);
  });
  // Готовая линия со стенда. Переопубликовывается в `kVisionPath` без изменений, чтобы продольный
  // планер и предупреждатель видели ровно то, по чему ехал поперечный контур.
  subscribe<adas::proto::LanePath>(topics::kVisionPathIn, [this](const adas::proto::LanePath& payload) {
    publish(topics::kVisionPath, payload);
    onLanes(lanePathFromProto(payload));
  });
  makeSolver();
  subscribe<adas::proto::LocalizationPose>(topics::kLocalizationPose, [this](const adas::proto::LocalizationPose& p) {
    if (config_.use_learned_params) {
      setLearnedParams(p.learned_params_valid(), p.learned_stiffness_factor(), p.learned_steer_ratio(),
                       p.learned_angle_offset_deg());
    }
    road_roll_valid_ = p.road_roll_valid();
    road_roll_rad_ = road_roll_valid_ ? p.road_roll_deg() * M_PI / 180.0 : 0.0;
  });
  subscribe<adas::proto::PandaHealth>(topics::kPandaHealth, [this](const adas::proto::PandaHealth& m) {
    assist_gate_.onReport(m.lat_actuation_allowed(), static_cast<int64_t>(now()));
  });
  registerParameters();
  LOGI("LaneKeep: controller=%s  ratio=%.1f  cam_y_left=%.3f  → %s / %s", config_.controller.c_str(), steer_ratio_,
       config_.cam_y_left_m, topics::kLaneKeep, topics::kSteerCommand);
}

void LaneKeep::registerParameters()
{
  registerParameter<double>(
      "steer_ratio", [this](const double& v) { setSteerRatio(v); }, [this] { return steer_ratio_; });
  registerParameter<double>(
      "max_steer_deg", [this](const double& v) { setMaxSteerDeg(v); }, [this] { return config_.max_steer_deg; });
  registerParameter<double>(
      "cam_y_left_m", [this](const double& v) { setCamYLeftM(v); }, [this] { return config_.cam_y_left_m; });
  registerParameter<std::string>(
      "lane_keep_controller", [this](const std::string& v) { setController(v); },
      [this] { return config_.controller; });
  registerParameter<double>(
      "pp_k_dd", [this](const double& v) { setPurePursuit(v, config_.pp_ld_min, config_.pp_ld_max, config_.pp_shift); },
      [this] { return config_.pp_k_dd; });
  registerParameter<double>(
      "pp_ld_min", [this](const double& v) { setPurePursuit(config_.pp_k_dd, v, config_.pp_ld_max, config_.pp_shift); },
      [this] { return config_.pp_ld_min; });
  registerParameter<double>(
      "pp_ld_max", [this](const double& v) { setPurePursuit(config_.pp_k_dd, config_.pp_ld_min, v, config_.pp_shift); },
      [this] { return config_.pp_ld_max; });
  registerParameter<double>(
      "pp_shift", [this](const double& v) { setPurePursuit(config_.pp_k_dd, config_.pp_ld_min, config_.pp_ld_max, v); },
      [this] { return config_.pp_shift; });
  registerParameter<double>(
      "pp_ld_curv_gain", [this](const double& v) { setPpLdCurvGain(v); }, [this] { return config_.pp_ld_curv_gain; });
  registerParameter<double>(
      "steer_slew_limit_deg", [this](const double& v) { setSteerSlewLimitDeg(v); },
      [this] { return config_.steer_slew_limit_deg; });
  registerParameter<double>(
      "tire_stiffness_factor", [this](const double& v) { setVehicleModel(config_.lat_use_vehicle_model, v); },
      [this] { return config_.tire_stiffness_factor; });
  registerParameter<bool>(
      "lat_use_vehicle_model", [this](const bool& v) { setVehicleModel(v, config_.tire_stiffness_factor); },
      [this] { return config_.lat_use_vehicle_model; });
  registerParameter<double>(
      "fp_steer_delay_s", [this](const double& v) { setFpSteerDelayS(v); },
      [this] { return config_.fp_steer_delay_s; });
  registerParameter<double>(
      "fp_steering_rate_weight", [this](const double& v) { setFpSteeringRateWeight(v); },
      [this] { return config_.fp_steering_rate_weight; });
  // Настройки разбора разметки. Раньше их держал TopicConvert; с его расформированием они остались
  // без владельца, и `path_lane_blend_scale` с телефона перестал доходить до чего бы то ни было.
  registerLanePathParameters(config_.lane_path, [this](const char* name, auto setter, auto getter) {
    registerParameter<double>(
        name, [setter](const double& v) { setter(v); }, [getter] { return getter(); });
  });

  // Веса и затравка vp-решателя. Были глобалами с сеттерами, наружу торчали как `set_mpc_*` в
  // pyadas; теперь они принадлежат экземпляру, поэтому меняются только пересбором планера.
  const auto reg_vp = [this](const char* name, double& field) {
    registerParameter<double>(
        name,
        [this, &field](const double& v) {
          field = std::max(0.0, v);
          solver_.reset();
        },
        [&field] { return field; });
  };
  reg_vp("mpc_epsi_gain", config_.mpc_epsi_gain);
  reg_vp("mpc_ff_scale", config_.mpc_ff_scale);
  reg_vp("mpc_cte_weight_base", config_.mpc_cte_weight_base);
  reg_vp("mpc_cte_quartic_scale", config_.mpc_cte_quartic_scale);
  reg_vp("mpc_cte_gain_base", config_.mpc_cte_gain_base);
  reg_vp("mpc_cte_gain_floor", config_.mpc_cte_gain_floor);

  registerParameter<bool>("use_learned_params", config_.use_learned_params);
  registerParameter<double>("lane_max_age_s", config_.lane_max_age_s);
  registerParameter<bool>("lka_suppress_on_blinker", config_.lka_suppress_on_blinker);
  registerParameter<double>("lka_blinker_resume_delay_s", config_.lka_blinker_resume_delay_s);
  registerParameter<bool>("lat_recompute_setpoint", config_.lat_recompute_setpoint);
  registerParameter<bool>("lat_require_assist", config_.lat_require_assist);
  registerParameter<double>("assist_max_age_s", config_.assist_max_age_s);
}

void LaneKeep::reset()
{
  if (solver_)
    solver_->reset();
  last_ = LaneKeepOutput{};
  have_chassis_ = false;
  have_desired_ = false;
  desired_swa_deg_ = 0.0;
  lat_.reset();
  slew_.reset();
  frame_dt_.reset();
  speed_gate_.reset();
  stale_gate_ = StaleGate{};
  blinker_gate_ = BlinkerGate{};
  assist_gate_ = AssistGate{};
}

bool LaneKeep::recomputeSetpoint(LaneKeepOutput& out)
{
  if (!useFlowpilot() || !have_chassis_ || !have_desired_)
    return false;
  auto* fp = dynamic_cast<lateral::FpPlanner*>(solver_.get());
  if (!fp)
    return false;
  const auto re = fp->recomputeSteer(chassis_.speed_mps, frame_dt_.value(), vehicleParams());
  if (!re)
    return false;
  const double steer = re->steer_rad;

  setDesiredFromSteer(steer);
  out.steer_rad = steer;
  out.curvature = re->curvature;
  out.dbg.mpc_kappa_used = re->curvature;
  out.dbg.mpc_max_steer_rad = re->max_steer_rad;
  out.dbg.mpc_delta_clamped_rad = steer;
  return true;
}

void LaneKeep::setController(std::string controller)
{
  if (controller == "mpc")
    config_.controller = "mpc";
  else if (controller == "fp_acados") {
    config_.controller = "fp";
    config_.fp_solver = "acados";
  } else if (controller == "fp" || controller == "flowpilot")
    config_.controller = "fp";
  else
    config_.controller = "pp";
  makeSolver();
  lat_.reset();
  slew_.reset();
  frame_dt_.reset();
  speed_gate_.reset();
  LOGI("LaneKeep: controller → %s", solverName());
}

void LaneKeep::setPurePursuit(double k_dd, double ld_min, double ld_max, double shift)
{
  config_.pp_k_dd = k_dd;
  config_.pp_ld_min = ld_min;
  config_.pp_ld_max = ld_max;
  config_.pp_shift = shift;
  solver_.reset();
}

lateral::PpPlanner::Config LaneKeep::ppPlannerConfig() const
{
  lateral::PpPlanner::Config c;
  c.k_dd = config_.pp_k_dd;
  c.waypoint_shift = config_.pp_shift;
  c.ld_min = config_.pp_ld_min;
  c.ld_max = config_.pp_ld_max;
  c.ld_curv_gain = config_.pp_ld_curv_gain;
  c.vehicle = vehicleParams();
  c.max_steer_rad = max_steer_rad_;
  return c;
}

lateral::VpPlanner::Config LaneKeep::vpPlannerConfig() const
{
  lateral::VpPlanner::Config c;
  c.Lf = config_.mpc_Lf;
  c.cte_ema_alpha = config_.mpc_cte_ema_alpha;
  c.epsi_ema_alpha = config_.mpc_epsi_ema_alpha;
  c.kappa_ema_alpha = config_.mpc_kappa_ema_alpha;
  c.vision_nominal_dt_s = config_.vision_nominal_dt_s;
  c.kappa_yaw_blend = config_.mpc_kappa_yaw_blend;
  c.kappa_yaw_min_speed = config_.mpc_kappa_yaw_min_speed;
  c.rate_limit_deg = config_.mpc_rate_limit_deg;
  c.rate_min_speed = config_.mpc_rate_min_speed;
  c.max_lateral_jerk = config_.mpc_max_lateral_jerk;

  c.solver.epsi_gain = std::max(config_.mpc_epsi_gain, 0.0);
  c.solver.ff_scale = std::max(config_.mpc_ff_scale, 0.0);
  c.solver.cte_weight_base = std::max(config_.mpc_cte_weight_base, 0.0);
  c.solver.cte_quartic_scale = std::max(config_.mpc_cte_quartic_scale, 0.0);
  c.solver.cte_gain_base = std::max(config_.mpc_cte_gain_base, 0.0);
  c.solver.cte_gain_floor = std::max(config_.mpc_cte_gain_floor, 0.0);

  c.vehicle = vehicleParams();
  c.limits = {config_.mpc_max_steer_deg, config_.mpc_low_speed_steer_deg, config_.mpc_steer_deg_per_mps};
  return c;
}

void LaneKeep::setMaxSteerDeg(double max_steer_deg)
{
  config_.max_steer_deg = max_steer_deg;
  max_steer_rad_ = max_steer_deg * M_PI / 180.0;
  solver_.reset();
}

void LaneKeep::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
  if (have_desired_)
    updateTorqueFromAngle();
}

void LaneKeep::onLanes(const LanePathMsg& msg)
{
  const double speed = have_chassis_ ? chassis_.speed_mps : 0.0;
  auto out = step(speed, msg);
  out.dbg.lane_anchored = msg.lane_anchored;
  out.dbg.lanelines_active = msg.lanelines_active;
  out.dbg.road_roll_deg = road_roll_valid_ ? road_roll_rad_ * 180.0 / M_PI : 0.0;
  out.dbg.lane_width_m = msg.lane_width_m;
  out.dbg.lane_offset_m = msg.lane_offset_m;
  out.dbg.center_force_m = msg.center_force_m;
  out.dbg.p_lane_blend_scale = msg.p_lane_blend_scale;
  out.dbg.p_camera_offset_m = msg.p_camera_offset_m;
  out.dbg.p_center_force_gain = msg.p_center_force_gain;
  out.capture_ts_us = msg.capture_ts_us > 0 ? msg.capture_ts_us : (msg.timestamp_us > 0 ? msg.timestamp_us : 0);
  out.vision_ts_us = msg.infer_ts_us > 0 ? msg.infer_ts_us : 0;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : 0;

  const bool commanded = out.has_target && out.status == "ok";
  if (slew_.apply(out.steer_rad, speed, frame_dt_.value(), commanded)) {
    out.steer_norm = out.max_steer_rad > 1e-6 ? out.steer_rad / out.max_steer_rad : 0.0;
    out.dbg.slew_clipped = true;
  }

  if (commanded) {
    setDesiredFromSteer(out.steer_rad);
    have_desired_ = true;
  } else {
    have_desired_ = false;
    desired_swa_deg_ = 0.0;
    desired_swa_no_offset_deg_ = 0.0;
    lat_.reset();
  }

  out.desired_swa_deg = desired_swa_deg_;
  if (have_chassis_) {
    out.actual_swa_deg = chassis_.steering_angle_deg;
    out.angle_error_deg = desired_swa_deg_ - chassis_.steering_angle_deg;
  }
  out.dbg.steer_output_enabled = steer_output_enabled_;
  last_ = out;
  publish(topics::kLaneKeep, createLaneKeepState(last_, static_cast<int64_t>(now()), max_torque_cnm_));

  updateTorqueFromAngle();
  publish(topics::kLaneKeepDebug,
          createLaneKeepDebug(last_, static_cast<int64_t>(now()), max_torque_cnm_, frame_dt_.value(), config_));
}

void LaneKeep::setDesiredFromSteer(double steer_rad)
{
  desired_swa_no_offset_deg_ = steer_sign_ * (steer_rad * 180.0 / M_PI) * effectiveSteerRatio();
  desired_swa_deg_ = desired_swa_no_offset_deg_ + effectiveAngleOffsetDeg();
}

void LaneKeep::applyGates(LaneKeepOutput& out, int64_t now_us, bool& assist_ok)
{
  if (out.status == "ok") {
    double age_s = 0.0;
    if (!stale_gate_.update(now_us, out.capture_ts_us, config_.lane_max_age_s, age_s)) {
      out.status = "stale";
      if (stale_gate_.justChanged()) {
        LOGW("LaneKeep: path stale (%.0f ms > %.0f) — command cleared", age_s * 1e3, config_.lane_max_age_s * 1e3);
      }
    } else if (stale_gate_.justChanged()) {
      LOGI("LaneKeep: path fresh again (%.0f ms)", age_s * 1e3);
    }
  }

  if (config_.lka_suppress_on_blinker && have_chassis_) {
    const auto b =
        blinker_gate_.update(now_us, chassis_.left_blinker, chassis_.right_blinker, config_.lka_blinker_resume_delay_s);
    if (b.suppressed) {
      if (out.status == "ok")
        out.status = "blinker";
      if (b.changed) {
        LOGI("LaneKeep: turn signal (%s) — wheel handed back", b.left ? "left" : "right");
      }
    } else {
      if (out.status == "blinker")
        out.status = "ok";
      if (b.changed) {
        LOGI("LaneKeep: turn signal off for %.1f s — steering again", b.since_off_s);
      }
    }
  }

  const auto a = assist_gate_.update(now_us, config_.assist_max_age_s);
  assist_ok = a.allowed;
  out.dbg.assist_allowed = a.known && a.allowed;
  out.dbg.assist_known = a.known;
  if (config_.lat_require_assist) {
    if (!a.allowed && out.status == "ok")
      out.status = "no_assist";
    else if (a.allowed && out.status == "no_assist")
      out.status = "ok";
    if (a.changed) {
      if (!a.allowed) {
        LOGW("LaneKeep: panda is not passing torque (controls_allowed=%d, known=%d) — command cleared, PID reset",
             static_cast<int>(assist_gate_.lastReportAllowed()), static_cast<int>(a.known));
      } else {
        LOGI("LaneKeep: torque reaching the rack again");
      }
    }
  }
}

void LaneKeep::updateTorqueFromAngle()
{
  LaneKeepOutput out = last_;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : out.chassis_ts_us;

  const int64_t now_us = static_cast<int64_t>(now());
  if (config_.lat_recompute_setpoint)
    recomputeSetpoint(out);

  bool assist = true;
  applyGates(out, now_us, assist);

  const bool active = steer_output_enabled_ && have_desired_ && have_chassis_ && out.has_target && out.status == "ok" &&
                      (!config_.lat_require_assist || assist);
  const auto lat = lat_.update(active, desired_swa_deg_, chassis_.steering_angle_deg, chassis_.speed_mps,
                               chassis_.steering_pressed, desired_swa_no_offset_deg_);

  out.desired_swa_deg = lat.angle_des_deg;
  out.actual_swa_deg = lat.angle_act_deg;
  out.angle_error_deg = lat.angle_error_deg;
  out.steer_norm = lat.steer_norm;
  out.dbg.pid_p = lat.p;
  out.dbg.pid_i = lat.i;
  out.dbg.pid_f = lat.f;
  out.dbg.kappa_solver = kappaSolverName();
  last_ = out;
  publish(topics::kSteerCommand,
          createSteerCommand(out, now_us, max_torque_cnm_, steer_output_enabled_, have_desired_));
}

void LaneKeep::makeSolver()
{
  if (config_.controller == "mpc") {
    solver_ = std::make_unique<lateral::VpPlanner>(vpPlannerConfig());
    return;
  }
  if (config_.controller != "fp") {
    solver_ = std::make_unique<lateral::PpPlanner>(ppPlannerConfig());
    return;
  }

  solver_ = std::make_unique<lateral::FpPlanner>(fpPlannerConfig());
}

lateral::FpPlanner::Config LaneKeep::fpPlannerConfig() const
{
  lateral::FpPlanner::Config c;
  c.Lf = config_.mpc_Lf;
  c.wheelbase_m = config_.wheelbase_m;
  c.max_lateral_jerk = config_.mpc_max_lateral_jerk;
  c.steering_rate_weight = config_.fp_steering_rate_weight;
  c.steer_delay_s = config_.fp_steer_delay_s;
  c.steer_slew_limit_deg = config_.steer_slew_limit_deg;
  c.roll_compensation = config_.roll_compensation;
  c.solver = config_.fp_solver;
  c.limits = {config_.mpc_max_steer_deg, config_.mpc_low_speed_steer_deg, config_.mpc_steer_deg_per_mps};
  return c;
}

const char* LaneKeep::solverName() const { return solver_ ? solver_->name() : config_.controller.c_str(); }

const char* LaneKeep::kappaSolverName() const { return solver_ ? solver_->solverName() : ""; }

LaneKeepOutput LaneKeep::step(double speed_mps, const LanePathMsg& path)
{
  const bool was_open = speed_gate_.isOpen();
  if (!speed_gate_.update(speed_mps)) {
    if (was_open) {
      if (solver_)
        solver_->reset();
      lat_.reset();
      slew_.reset();
    }
    LaneKeepOutput out;
    out.controller = config_.controller;
    out.status = "low_speed";
    out.dbg.speed_mps = speed_mps;
    return out;
  }

  frame_dt_.update(path.capture_ts_us > 0 ? path.capture_ts_us : path.timestamp_us);
  if (!solver_)
    makeSolver();
  slew_.setConfig({config_.steer_slew_limit_deg, config_.mpc_max_lateral_jerk, config_.mpc_rate_min_speed,
                   config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m, 10});

  const lateral::Input in =
      lateral::inputFromMessages(path, speed_mps, have_chassis_ ? chassis_.yaw_rate : 0.0, have_chassis_,
                                 frame_dt_.value(), config_.cam_y_left_m, vehicleParams());

  LaneKeepOutput out;
  lateral::applyToOutput(solver_->update(in), out);
  return out;
}

}  // namespace services
}  // namespace adas
