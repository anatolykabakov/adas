#pragma once

#include <string_view>

#include "adas/middleware/manager.hpp"
#include "messages.pb.h"

namespace adas {
namespace services {
/** Publishes the middleware's own timing so a drive can be judged for health. */
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
