#include "adas/services/lane_keep_gates.h"

#include <algorithm>

namespace adas {
namespace services {
bool StaleGate::update(int64_t now_us, int64_t capture_ts_us, double max_age_s, double& age_s_out)
{
  just_changed_ = false;
  if (!(max_age_s > 0.0) || capture_ts_us <= 0) {
    age_s_out = 0.0;
    return true;
  }

  age_s_out = static_cast<double>(now_us - capture_ts_us) * 1e-6;
  const bool stale = age_s_out > max_age_s;
  just_changed_ = stale != stale_;
  stale_ = stale;
  return !stale;
}

BlinkerGate::Result BlinkerGate::update(int64_t now_us, bool left, bool right, double resume_delay_s)
{
  Result r;
  r.left = left;

  const bool on = left || right;
  if (on) {
    off_armed_ = false;
  } else if (!off_armed_ && suppressed_) {
    off_us_ = now_us;
    off_armed_ = true;
  }

  r.since_off_s = off_armed_ ? static_cast<double>(now_us - off_us_) * 1e-6 : 0.0;
  const bool hold = off_armed_ && r.since_off_s < std::max(0.0, resume_delay_s);

  r.suppressed = on || hold;
  r.changed = r.suppressed != suppressed_;
  if (r.suppressed) {
    suppressed_ = true;
  } else if (suppressed_) {
    suppressed_ = false;
    off_armed_ = false;
  }
  return r;
}

void AssistGate::onReport(bool allowed, int64_t now_us)
{
  allowed_ = allowed;
  ts_us_ = now_us;
  have_ = true;
}

AssistGate::Result AssistGate::update(int64_t now_us, double max_age_s)
{
  Result r;
  if (!have_) {
    r.known = false;
    r.allowed = true;
  } else {
    const double age_s = static_cast<double>(now_us - ts_us_) * 1e-6;
    if (max_age_s > 0.0 && age_s > max_age_s) {
      r.known = false;
      r.allowed = false;
    } else {
      r.known = true;
      r.allowed = allowed_;
    }
  }

  const bool absent = !r.allowed;
  r.changed = absent != absent_;
  absent_ = absent;
  return r;
}

}  // namespace services
}  // namespace adas
