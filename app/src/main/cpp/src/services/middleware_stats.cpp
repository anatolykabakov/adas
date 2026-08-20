#include "adas/services/middleware_stats.h"

#include "adas/utils/adas_topics.h"
#include "adas/utils/logger.h"
#include "adas/utils/proto_convert.h"

namespace adas {
namespace services {
void MiddlewareStats::configure()
{
  scheduleTimer(1000, [this] { tick(); }, "tick");
}

void MiddlewareStats::tick()
{
  middleware::Manager* mw = middleware();
  if (!mw)
    return;

  const middleware::MiddlewareSnapshot snap = mw->snapshotStats();

  if (snap.any_lagging) {
    if (!warned_lagging_) {
      LOGW("middleware::Manager lagging: one or more timers behind period");
      for (const auto& s : snap.services_timing) {
        if (!s.lagging)
          continue;
        for (const auto& t : s.timers) {
          if (!t.lagging)
            continue;
          LOGW("  %s/%s mean_dt=%.2f ms period=%.2f ms max_dt=%.2f ms", s.name.c_str(), t.name.c_str(), t.mean_dt_ms,
               t.period_ms, t.max_dt_ms);
        }
      }
      warned_lagging_ = true;
    }
  } else {
    warned_lagging_ = false;
  }

  publish(topics::kMiddlewareStats, createMiddlewareStats(snap));
}

}  // namespace services
}  // namespace adas
