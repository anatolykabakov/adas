#pragma once

#include <deque>
#include <string_view>
#include <variant>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "model_long.pb.h"
#include "safety_warn.pb.h"
#include "steer.pb.h"
#include "adas/services/planner.h"
#include "adas/utils/adas_topics.h"

namespace adas {
using HostOutMsg = std::variant<LaneKeepOutput, LocalizationPose, CameraCalibrationState, adas::proto::SafetyWarnState,
                                adas::proto::SteerCommand, adas::proto::LongPlanState>;

namespace services {
/** Collects internal-bus messages for the host application. */
class InternalSubscriber : public adas::middleware::Service {
public:
  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "internal_sub"; }

  /**
   * \brief Drains everything queued since the previous call.
   * \return Payloads in arrival order; empty when nothing arrived.
   */
  std::vector<HostOutMsg> popMessages();

private:
  std::deque<HostOutMsg> out_;
};

}  // namespace services

}  // namespace adas
