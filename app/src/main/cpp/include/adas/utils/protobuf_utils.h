#pragma once

#include <vector>
#include <chrono>
#include "adas/utils/logger.h"
#include "messages.pb.h"
#include "can.pb.h"
#include "panda.pb.h"
#include "car_state.pb.h"
#include "adas/panda/health.h"
#include "adas/panda/can.h"
#include "adas/panda/can_frame.h"

namespace utils {
int64_t getCurrentTimestamp();

ai::flow::adas::ZMQMessage createCANMessage(const std::vector<can_frame>& frames);
ai::flow::adas::ZMQMessage createHealthMessage(const health_t& health);
ai::flow::adas::ZMQMessage createCarStateMessage(const ai::flow::adas::CarState& state);
}  // namespace utils
