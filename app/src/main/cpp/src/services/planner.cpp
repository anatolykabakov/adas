#include "adas/services/planner.h"

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
constexpr double kPidRateHz = 100.0;

}  // namespace

Planner::Planner(Config p)
  : config_(std::move(p))
  , veh_({config_.steer_ratio, config_.steer_sign, config_.max_steer_deg, config_.tire_stiffness_factor})
  , max_torque_cnm_(config_.max_torque_cnm)
{
  if (config_.controller != "mpc" && config_.controller != "fp")
    config_.controller = "pp";

  frame_dt_ = IntervalFilter({config_.vision_nominal_dt_s, kMinDtS, kMaxDtS, kMaxGapS, kDtAlpha});
  speed_gate_.setThresholds(config_.min_control_speed_mps,
                            config_.min_control_speed_mps + std::max(0.0, config_.min_control_speed_hyst_mps));
}

void Planner::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, veh_.steerRatio()));
  });
  subscribe<adas::proto::LaneLines>(topics::kVisionLanes, [this](const adas::proto::LaneLines& payload) {
    const LanePathMsg path = laneLinesToPath(payload, config_.lane_path, &lane_fusion_);
    publish(topics::kVisionPath, createLanePath(path));
    onLanes(path);
  });
  subscribe<adas::proto::LanePath>(topics::kVisionPathIn, [this](const adas::proto::LanePath& payload) {
    publish(topics::kVisionPath, payload);
    onLanes(lanePathFromProto(payload));
  });
  makeSolver();
  subscribe<adas::proto::LocalizationPose>(topics::kLocalizationPose, [this](const adas::proto::LocalizationPose& p) {
    {
      setLearnedParams(p.learned_params_valid(), p.learned_stiffness_factor(), p.learned_steer_ratio(),
                       p.learned_angle_offset_deg());
    }
    road_roll_valid_ = p.road_roll_valid();
    road_roll_rad_ = road_roll_valid_ ? p.road_roll_deg() * M_PI / 180.0 : 0.0;
  });
  subscribe<adas::proto::SteerCommand>(topics::kSteerCommand, [this](const adas::proto::SteerCommand& m) { cmd_ = m; });
  if (config_.long_plan_enabled) {
    subscribe<adas::proto::ModelLongPlan>(topics::kVisionModelLong, [this](const adas::proto::ModelLongPlan& m) {
      model_long_ = m;
      have_model_long_ = true;
    });
    scheduleTimer(
        50, [this] { longTick(); }, "long");
  }

  registerParameters();
  LOGI("Planner: controller=%s  ratio=%.1f  cam_y_left=%.3f  → %s / %s", config_.controller.c_str(), veh_.steerRatio(),
       config_.cam_y_left_m, topics::kLaneKeep, topics::kLatPlan);
}

void Planner::registerParameters()
{
  registerParameter<double>(
      "steer_ratio", [this](const double& v) { setSteerRatio(v); }, [this] { return veh_.steerRatio(); });
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
      "tire_stiffness_factor", [this](const double& v) { setTireStiffnessFactor(v); },
      [this] { return config_.tire_stiffness_factor; });
  registerParameter<double>(
      "fp_steer_delay_s", [this](const double& v) { setFpSteerDelayS(v); },
      [this] { return config_.fp_steer_delay_s; });
  registerParameter<double>(
      "fp_steering_rate_weight", [this](const double& v) { setFpSteeringRateWeight(v); },
      [this] { return config_.fp_steering_rate_weight; });
  registerLanePathParameters(config_.lane_path, [this](const char* name, auto setter, auto getter) {
    registerParameter<double>(
        name, [setter](const double& v) { setter(v); }, [getter] { return getter(); });
  });

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
}

void Planner::reset()
{
  if (solver_)
    solver_->reset();
  last_ = LaneKeepOutput{};
  have_chassis_ = false;

  frame_dt_.reset();
  speed_gate_.reset();
  cmd_ = adas::proto::SteerCommand{};
}

void Planner::setController(std::string controller)
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
  frame_dt_.reset();
  speed_gate_.reset();
  LOGI("Planner: controller → %s", solverName());
}

void Planner::setPurePursuit(double k_dd, double ld_min, double ld_max, double shift)
{
  config_.pp_k_dd = k_dd;
  config_.pp_ld_min = ld_min;
  config_.pp_ld_max = ld_max;
  config_.pp_shift = shift;
  solver_.reset();
}

lateral::PpPlanner::Config Planner::ppPlannerConfig() const
{
  lateral::PpPlanner::Config c;
  c.k_dd = config_.pp_k_dd;
  c.waypoint_shift = config_.pp_shift;
  c.ld_min = config_.pp_ld_min;
  c.ld_max = config_.pp_ld_max;
  c.ld_curv_gain = config_.pp_ld_curv_gain;
  c.vehicle = vehicleParams();
  c.max_steer_rad = veh_.maxSteerRad();
  return c;
}

lateral::VpPlanner::Config Planner::vpPlannerConfig() const
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

void Planner::setMaxSteerDeg(double max_steer_deg)
{
  config_.max_steer_deg = max_steer_deg;
  veh_.setMaxSteerDeg(max_steer_deg);
  solver_.reset();
}

/** \brief Curvature equivalent to the planner's command.
 *
 *  `fp` computes curvature first and the angle from it, so its curvature goes out unchanged. `pp` and
 *  `vp` compute an angle, and their path curvature has nothing to do with the command — the lateral
 *  offset correction sits inside the angle. Their angle is therefore converted back to curvature with
 *  the same constants the controller uses in the forward direction, so the controller's output is
 *  exactly the angle the planner asked for.
 *
 *  That keeps the interface in curvature for every solver, not only for the one that thinks in it. */
double Planner::commandCurvature(double speed_mps) const
{
  if (useFlowpilot())
    return last_.curvature;

  const auto vehicle = vehicleParams();
  const double slip = lateral::slipFactorOrZero(vehicle);
  const double Lf = config_.mpc_Lf > 1e-3 ? config_.mpc_Lf : config_.wheelbase_m;
  double kappa = curvatureFromSteer(-last_.steer_rad, speed_mps, Lf, slip);
  if (road_roll_valid_)
    kappa += rollCompensationCurvature(road_roll_rad_, speed_mps, slip);
  return kappa;
}

void Planner::longTick()
{
  if (!have_chassis_ || !have_model_long_)
    return;

  longplan::Input in;
  in.v_ego = std::max(0.0, chassis_.speed_mps);
  in.path = path_.size() >= 3 ? &path_ : nullptr;

  const auto& lead = model_long_.lead0();
  in.lead.prob = lead.prob();
  in.lead.d_rel = lead.d_rel() > 0 ? lead.d_rel() : (lead.x_size() > 0 ? lead.x(0) : 0.0);
  in.lead.v_lead = lead.v_lead() != 0 ? lead.v_lead() : (lead.v_size() > 0 ? lead.v(0) : 0.0);
  in.lead.y_rel = lead.y_rel();

  in.plan_v.valid = model_long_.plan_v_x_size() > 0 || model_long_.plan_v0() != 0.0;
  in.plan_v.v_plan = model_long_.plan_v_x_size() > 0 ? model_long_.plan_v_x(0) : model_long_.plan_v0();
  in.plan_v.pose_valid = model_long_.pose_valid();
  in.plan_v.pose_vx = model_long_.pose_vx();

  publish(topics::kLongPlan, createLongPlan(in, longplan::compute(config_.long_plan, in), nowMs()));
}

void Planner::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void Planner::onLanes(const LanePathMsg& msg)
{
  path_ = msg.polyline;
  const double speed = have_chassis_ ? chassis_.speed_mps : 0.0;
  auto out = step(speed, msg);
  applyLanePath(out, msg);
  applySteerFeedback(out, cmd_);
  // The three that are the service's own, not the path's or the controller's.
  out.dbg.road_roll_deg = road_roll_valid_ ? road_roll_rad_ * 180.0 / M_PI : 0.0;
  out.chassis_ts_us = have_chassis_ ? chassis_.timestamp_us : 0;
  out.dbg.kappa_solver = kappaSolverName();
  last_ = out;
  publish(topics::kLaneKeep, createLaneKeepState(last_, static_cast<int64_t>(now()), max_torque_cnm_));
  publish(topics::kLatPlan, createLatPlan(last_, commandCurvature(speed), frame_dt_.value(), kappaSolverName()));
  publish(topics::kLaneKeepDebug,
          createLaneKeepDebug(last_, static_cast<int64_t>(now()), max_torque_cnm_, frame_dt_.value(), config_));
}

void Planner::makeSolver()
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

lateral::FpPlanner::Config Planner::fpPlannerConfig() const
{
  lateral::FpPlanner::Config c;
  c.Lf = config_.mpc_Lf;
  c.wheelbase_m = config_.wheelbase_m;
  c.max_lateral_jerk = config_.mpc_max_lateral_jerk;
  c.steering_rate_weight = config_.fp_steering_rate_weight;
  c.steer_delay_s = config_.fp_steer_delay_s;
  c.steer_slew_limit_deg = config_.steer_slew_limit_deg;
  c.solver = config_.fp_solver;
  c.limits = {config_.mpc_max_steer_deg, config_.mpc_low_speed_steer_deg, config_.mpc_steer_deg_per_mps};
  return c;
}

const char* Planner::solverName() const { return solver_ ? solver_->name() : config_.controller.c_str(); }

const char* Planner::kappaSolverName() const { return solver_ ? solver_->solverName() : ""; }

LaneKeepOutput Planner::step(double speed_mps, const LanePathMsg& path)
{
  const bool was_open = speed_gate_.isOpen();
  if (!speed_gate_.update(speed_mps)) {
    if (was_open) {
      if (solver_)
        solver_->reset();
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
  const lateral::Input in =
      lateral::inputFromMessages(path, speed_mps, have_chassis_ ? chassis_.yaw_rate : 0.0, have_chassis_,
                                 frame_dt_.value(), config_.cam_y_left_m, vehicleParams());

  LaneKeepOutput out;
  lateral::applyToOutput(solver_->update(in), out);
  return out;
}

}  // namespace services
}  // namespace adas
