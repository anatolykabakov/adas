#include "services/long_plan_service.h"

#include <algorithm>
#include <cmath>

#include "utils/logger.h"
#include "utils/protobuf_utils.h"

namespace adas {

void LongPlanService::configure()
{
  subscribe<ai::flow::adas::ZMQMessage>(topics::kVisionModelLong,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onModelLong(m); });
  subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& m) { onChassis(m); });
  subscribe<LanePathMsg>(topics::kVisionPath, [this](const LanePathMsg& m) { onPath(m); });
  scheduleTimer(
      50, [this] { tick(); }, "tick");
  LOGI("LongPlanService: %s + chassis → %s (coast envelope %.2f m/s^2, plan_v %s)", topics::kVisionModelLong,
       topics::kLongPlan, config_.a_coast_ms2, config_.plan_v_enabled ? "on" : "off");
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

  longplan::Input in;
  in.v_ego = std::max(0.0, chassis_.speed_mps);
  in.path = have_path_ ? &path_ : nullptr;

  // Only `lead0`. `lead1` and `lead2` are the model's predictions at +2 s and +4 s, so taking the
  // most probable of the three targets a vehicle that is not there yet — the same defect that was
  // fixed in the warning path on 2026-08-02 and left standing here until run 2026_08_06_00_36_42.
  const auto& lead = model_.lead0();
  in.lead.prob = lead.prob();
  in.lead.d_rel = lead.d_rel() > 0 ? lead.d_rel() : (lead.x_size() > 0 ? lead.x(0) : 0.0);
  in.lead.v_lead = lead.v_lead() != 0 ? lead.v_lead() : (lead.v_size() > 0 ? lead.v(0) : 0.0);
  in.lead.y_rel = lead.y_rel();

  in.plan_v.valid = model_.plan_v_x_size() > 0 || model_.plan_v0() != 0.0;
  in.plan_v.v_plan = model_.plan_v_x_size() > 0 ? model_.plan_v_x(0) : model_.plan_v0();
  in.plan_v.pose_valid = model_.pose_valid();
  in.plan_v.pose_vx = model_.pose_vx();

  const longplan::Plan plan = longplan::compute(config_, in);

  ai::flow::adas::ZMQMessage zmq;
  const int64_t ms = utils::getCurrentTimestamp();
  zmq.set_timestamp(ms);
  zmq.set_topic(topics::kLongPlan);
  auto* lp = zmq.mutable_long_plan();
  lp->set_timestamp(ms);
  lp->set_v_ego(in.v_ego);
  lp->set_v_target(plan.v_target);
  lp->set_a_target(plan.a_target);
  lp->set_lead_d(in.lead.d_rel);
  lp->set_lead_v(in.lead.v_lead);
  lp->set_lead_prob(in.lead.prob);
  lp->set_has_lead(plan.has_lead);
  lp->set_source(plan.source);
  lp->set_v_curv(plan.v_curv);
  lp->set_kappa_ahead(plan.kappa_ahead);
  lp->set_status(plan.status);
  publish(topics::kLongPlan, zmq);
}

}  // namespace adas
