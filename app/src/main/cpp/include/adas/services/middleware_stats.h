#pragma once

#include <string_view>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"

namespace adas {
namespace services {
/**
 * \brief Publishes the middleware's own timing so a drive can be judged for health.
 *
 * \details Per-service tick period, jitter and queue depth, taken from the manager's snapshot. This is
 * the only way to tell "the model was slow" from "the model was starved of frames" after the fact, so it
 * is recorded on every drive rather than enabled for debugging.
 */
class MiddlewareStats : public middleware::Service {
public:
  void configure() override;
  void reset() override {}
  std::string_view getName() const override { return "mw_stats"; }

private:
  void tick();
  bool warned_lagging_ = false;
};

}  // namespace services

}  // namespace adas
