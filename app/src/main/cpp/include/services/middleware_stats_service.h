#pragma once

#include <string_view>

#include "middleware/middleware.hpp"
#include "messages.pb.h"

namespace adas {

/// Publishes MiddlewareSnapshot as ZMQMessage on middleware/stats (~1 Hz) for bag logging.
class MiddlewareStatsService : public Service {
public:
  void configure() override;
  void reset() override {}
  std::string_view getName() const override { return "mw_stats"; }

private:
  void tick();
  bool warned_lagging_ = false;
};

}  // namespace adas
