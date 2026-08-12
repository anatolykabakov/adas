#include "adas/services/long_plan.h"

#include "adas/utils/proto_convert.h"

#include <algorithm>
#include <cmath>

#include "adas/utils/logger.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace services {
void LongPlan::configure()
{
  subscribe<adas::proto::CarState>(topics::kVehicleState, [this](const adas::proto::CarState& payload) {
    onChassis(carStateToChassis(payload, config_.steer_ratio));
  });
  subscribe<adas::proto::LanePath>(
      topics::kVisionPath, [this](const adas::proto::LanePath& payload) { onPath(lanePathFromProto(payload)); });
  subscribe<adas::proto::ModelLongPlan>(topics::kVisionModelLong,
                                        [this](const adas::proto::ModelLongPlan& m) { onModelLong(m); });
  scheduleTimer(
      50, [this] { tick(); }, "tick");
  LOGI("LongPlan: %s + chassis → %s (coast envelope %.2f m/s^2, plan_v %s)", topics::kVisionModelLong,
       topics::kLongPlan, config_.a_coast_ms2, config_.plan_v_enabled ? "on" : "off");
}

void LongPlan::reset()
{
  have_chassis_ = false;
  have_model_ = false;
  have_path_ = false;
  model_ = {};
  chassis_ = {};
  path_.clear();
}

void LongPlan::onPath(const LanePathMsg& msg)
{
  path_ = msg.polyline;
  have_path_ = path_.size() >= 3;
}

void LongPlan::onModelLong(const adas::proto::ModelLongPlan& payload)
{
  model_ = payload;
  have_model_ = true;
}

void LongPlan::onChassis(const ChassisSample& msg)
{
  chassis_ = msg;
  have_chassis_ = true;
}

void LongPlan::tick()
{
  if (!have_chassis_ || !have_model_)
    return;

  longplan::Input in;
  in.v_ego = std::max(0.0, chassis_.speed_mps);
  in.path = have_path_ ? &path_ : nullptr;

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

  publish(topics::kLongPlan, createLongPlan(in, plan, nowMs()));
}

}  // namespace services
}  // namespace adas
