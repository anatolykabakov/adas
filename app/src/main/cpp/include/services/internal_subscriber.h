#pragma once

#include <deque>
#include <string_view>
#include <variant>
#include <vector>

#include "middleware/middleware.hpp"
#include "safety_warn.pb.h"
#include "steer.pb.h"
#include "services/lane_keep_service.h"
#include "utils/adas_topics.h"

namespace adas {

// `SteerCommand` is the app's actuation output — the torque that reaches CarController — and it belongs here
// for the same reason the others do: a host that cannot observe it cannot check the app against anything.
// It was missing, which meant an offline replay could see the planned curvature and not the command produced
// from it. Note that `LaneKeepOutput.steer_norm` is *not* a substitute: on the vision path it carries the
// geometric normalisation of the commanded road-wheel angle, and only the chassis path fills it with the
// angle-PID output.
using HostOutMsg = std::variant<LaneKeepOutput, LocalizationPose, CameraCalibrationState,
                                ai::flow::adas::SafetyWarnState, ai::flow::adas::SteerCommand>;

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
