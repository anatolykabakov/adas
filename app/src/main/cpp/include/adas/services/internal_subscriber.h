#pragma once

#include <deque>
#include <string_view>
#include <variant>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "safety_warn.pb.h"
#include "steer.pb.h"
#include "adas/services/planner.h"
#include "adas/utils/adas_topics.h"

namespace adas {
using HostOutMsg = std::variant<LaneKeepOutput, LocalizationPose, CameraCalibrationState, adas::proto::SafetyWarnState,
                                adas::proto::SteerCommand>;

namespace services {
/**
 * \brief Collects internal-bus messages for the host application.
 *
 * \details The bus is C++ and the UI is Java or Python; this service is the single place where messages
 * cross that line. It subscribes to what a host has to display or record and queues the payloads for
 * `AdasApp::popMessages`, so the host polls one queue instead of registering callbacks per topic.
 */
class InternalSubscriber : public adas::middleware::Service {
public:
  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "internal_sub"; }

  /**
   * \brief Drains everything queued since the previous call.
   *
   * \return Payloads in arrival order; empty when nothing arrived.
   *
   * \note The queue is bounded: a host that stops polling loses the oldest messages rather than growing
   * memory without limit, since the alternative is an out-of-memory kill on the car.
   */
  std::vector<HostOutMsg> popMessages();

private:
  std::deque<HostOutMsg> out_;
};

}  // namespace services

}  // namespace adas
