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

adas::proto::ZMQMessage createCANMessage(const std::vector<can_frame>& frames);
adas::proto::ZMQMessage createHealthMessage(const health_t& health);
adas::proto::ZMQMessage createCarStateMessage(const adas::proto::CarState& state);
}  // namespace utils
