#pragma once

#include <deque>
#include <string_view>
#include <variant>
#include <vector>

#include "middleware/middleware.hpp"
#include "safety_warn.pb.h"
#include "services/lane_keep_service.h"
#include "utils/adas_topics.h"

namespace adas {

using HostOutMsg =
    std::variant<LaneKeepOutput, LocalizationPose, CameraCalibrationState, ai::flow::adas::SafetyWarnState>;

class InternalSubscriber : public adas::Service {
public:
  void configure() override;
  void reset() override;
  std::string_view getName() const override { return "internal_sub"; }

  std::vector<HostOutMsg> popMessages();

private:
  std::deque<HostOutMsg> out_;
};

}  // namespace adas
