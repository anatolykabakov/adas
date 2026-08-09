#include "adas/services/middleware_stats.h"

#include "adas/utils/adas_topics.h"
#include "adas/utils/logger.h"

namespace adas {
namespace services {

void MiddlewareStats::configure()
{
  // 1 Hz — enough for bag analysis without flooding.
  scheduleTimer(
      1000, [this] { tick(); }, "tick");
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

  ai::flow::adas::ZMQMessage zmq;
  const int64_t ts_ms = static_cast<int64_t>(snap.timestamp_us / 1000ULL);
  zmq.set_timestamp(ts_ms);
  zmq.set_topic(topics::kMiddlewareStats);

  auto* st = zmq.mutable_middleware_stats();
  st->set_timestamp(ts_ms);
  st->set_dropped_total(snap.dropped_total);
  st->set_services(snap.services);
  st->set_running(snap.running);
  st->set_any_lagging(snap.any_lagging);

  for (const auto& s : snap.services_timing) {
    auto* t = st->add_services_timing();
    t->set_name(s.name);
    t->set_running(s.running);
    t->set_messages_processed(s.messages_processed);
    t->set_timers_fired(s.timers_fired);
    t->set_exceptions(s.exceptions);
    t->set_dropped(s.dropped);
    t->set_inbox_depth(s.inbox_depth);
    t->set_backlog_depth(s.backlog_depth);
    t->set_last_cb_ms(s.last_cb_ms);
    t->set_mean_cb_ms(s.mean_cb_ms);
    t->set_max_cb_ms(s.max_cb_ms);
    t->set_period_ms(s.period_ms);
    t->set_last_dt_ms(s.last_dt_ms);
    t->set_mean_dt_ms(s.mean_dt_ms);
    t->set_max_dt_ms(s.max_dt_ms);
    t->set_lagging(s.lagging);
    for (const auto& tm : s.timers) {
      auto* tt = t->add_timers();
      tt->set_name(tm.name);
      tt->set_period_ms(tm.period_ms);
      tt->set_last_dt_ms(tm.last_dt_ms);
      tt->set_mean_dt_ms(tm.mean_dt_ms);
      tt->set_max_dt_ms(tm.max_dt_ms);
      tt->set_lagging(tm.lagging);
      tt->set_fired(tm.fired);
    }
  }

  publish(topics::kMiddlewareStats, zmq);
}

}  // namespace services
}  // namespace adas
