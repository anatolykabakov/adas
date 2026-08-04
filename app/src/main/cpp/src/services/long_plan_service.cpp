#include "services/long_plan_service.h"

#include <algorithm>
#include <cmath>

#include "utils/logger.h"
#include "utils/protobuf_utils.h"

namespace adas {
namespace {

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace

void LongPlanService::configure()
{
  subscribe<ai::flow::adas::ZMQMessage>(topics::kVisionModelLong,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onModelLong(m); });
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<LanePathMsg>(topics::kVisionPath, [this](const LanePathMsg& m) { onPath(m); });
  scheduleTimer(
      50, [this] { tick(); }, "tick");
  LOGI("LongPlanService: %s + chassis → %s (ACC stub, no actuation)", topics::kVisionModelLong, topics::kLongPlan);
}

void LongPlanService::reset()
{
  have_chassis_ = false;
  have_model_ = false;
  have_path_ = false;
  model_ = {};
  chassis_ = {};
  path_.clear();
}

void LongPlanService::onPath(const LanePathMsg& msg)
{
  path_ = msg.polyline;
  have_path_ = path_.size() >= 3;
}

void LongPlanService::onModelLong(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_model_long_plan())
    return;
  model_ = msg.model_long_plan();
  have_model_ = true;
}

void LongPlanService::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void LongPlanService::tick()
{
  if (!have_chassis_ || !have_model_)
    return;

  const double v_ego = std::max(0.0, chassis_.speed_mps);
  double v_target = v_ego;
  double a_target = 0.0;
  double lead_d = 0.0;
  double lead_v = 0.0;
  double lead_prob = 0.0;
  bool has_lead = false;
  std::string source = "none";
  std::string status = "ok";

  const auto& l0 = model_.lead0();
  const auto& l1 = model_.lead1();
  const auto& l2 = model_.lead2();
  const auto* lead = &l0;
  if (l1.prob() > lead->prob())
    lead = &l1;
  if (l2.prob() > lead->prob())
    lead = &l2;
  lead_prob = lead->prob();
  lead_d = lead->d_rel() > 0 ? lead->d_rel() : (lead->x_size() > 0 ? lead->x(0) : 0.0);
  lead_v = lead->v_lead() != 0 ? lead->v_lead() : (lead->v_size() > 0 ? lead->v(0) : 0.0);

  if (lead_prob >= config_.lead_prob_thresh && lead_d > 1.0 && lead_d < 120.0) {
    has_lead = true;
    source = "lead";
    const double gap_des = std::max(config_.min_gap_m, config_.t_follow * v_ego);
    const double gap_err = lead_d - gap_des;
    const double v_rel = lead_v - v_ego;
    a_target = config_.kp_gap * gap_err + 0.5 * v_rel;
    v_target = std::max(0.0, lead_v);
  } else if (model_.plan_v_x_size() > 0 || model_.plan_v0() != 0.0) {
    source = "plan_v";
    const double v_plan = model_.plan_v_x_size() > 0 ? model_.plan_v_x(0) : model_.plan_v0();
    const double v_ref = model_.pose_valid() ? 0.5 * (v_plan + model_.pose_vx()) : v_plan;
    a_target = config_.kp_v * (v_ref - v_ego);
    v_target = std::max(0.0, v_ref);
  } else {
    status = "no_ref";
  }

  double kappa_ahead = 0.0;
  double v_curv = 0.0;
  if (config_.curv_enabled && have_path_ && v_ego >= config_.curv_min_speed_ms) {
    const double from_m = std::max(5.0, 0.5 * v_ego);
    const double to_m = std::max(from_m + 15.0, config_.curv_preview_s * v_ego);
    kappa_ahead = maxCurvatureAhead(path_, from_m, to_m);
    v_curv = std::max(config_.curv_v_floor_ms,
                      curvatureSpeedLimit(kappa_ahead, config_.curv_a_lat_max, v_target > 0 ? v_target : v_ego));
    if (v_curv < v_target - 0.1) {
      v_target = v_curv;
      a_target = std::min(a_target, config_.kp_v * (v_curv - v_ego));
      source = source == "none" ? "curv" : source + "+curv";
    }
  }

  a_target = clampd(a_target, config_.a_min, config_.a_max);

  ai::flow::adas::ZMQMessage zmq;
  const int64_t ms = utils::getCurrentTimestamp();
  zmq.set_timestamp(ms);
  zmq.set_topic(topics::kLongPlan);
  auto* lp = zmq.mutable_long_plan();
  lp->set_timestamp(ms);
  lp->set_v_ego(v_ego);
  lp->set_v_target(v_target);
  lp->set_a_target(a_target);
  lp->set_lead_d(lead_d);
  lp->set_lead_v(lead_v);
  lp->set_lead_prob(lead_prob);
  lp->set_has_lead(has_lead);
  lp->set_source(source);
  lp->set_v_curv(v_curv);
  lp->set_kappa_ahead(kappa_ahead);
  lp->set_status(status);
  publish(topics::kLongPlan, zmq);
}

}  // namespace adas
