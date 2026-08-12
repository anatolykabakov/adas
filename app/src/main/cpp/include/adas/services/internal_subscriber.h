#pragma once

#include <deque>
#include <string_view>
#include <variant>
#include <vector>

#include "adas/middleware/manager.hpp"
#include "safety_warn.pb.h"
#include "steer.pb.h"
#include "adas/services/lane_keep.h"
#include "adas/utils/adas_topics.h"

namespace adas {
using HostOutMsg = std::variant<LaneKeepOutput, LocalizationPose, CameraCalibrationState, adas::proto::SafetyWarnState,
                                adas::proto::SteerCommand>;

namespace services {
class InternalSubscriber : public adas::middleware::Service {
public:
  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "internal_sub"; }

  std::vector<HostOutMsg> popMessages();

private:
  std::deque<HostOutMsg> out_;
};

}  // namespace services

}  // namespace adas
