#pragma once

#include <cstdint>

namespace adas {
/// The reference line is stale: the command came from a picture no longer worth trusting.
class StaleGate {
public:
  /// `max_age_s <= 0` disables the rule.
  bool update(int64_t now_us, int64_t capture_ts_us, double max_age_s, double& age_s_out);

  bool justChanged() const { return just_changed_; }
  bool stale() const { return stale_; }

private:
  bool stale_ = false;
  bool just_changed_ = false;
};

/**
 * \brief Turn signal on: the wheel goes back to the driver.
 *
 * There is no lane-change planner, so holding the lane while the driver crosses the marking means
 * fighting them. Control does not come back the moment the signal goes off: the car is still
 * crossing the line then.
 */
class BlinkerGate {
public:
  struct Result {
    bool suppressed = false;
    bool changed = false;
    double since_off_s = 0.0;
    bool left = false;
  };

  Result update(int64_t now_us, bool left, bool right, double resume_delay_s);

  bool suppressed() const { return suppressed_; }

private:
  int64_t off_us_ = 0;
  bool off_armed_ = false;
  bool suppressed_ = false;
};

/**
 * \brief Whether torque is reaching the rack, according to the panda.
 *
 * The two answers when we do not know are deliberately different. Never having heard from a panda
 * means there is none in the loop — a replay, the bindings, a bench — and closing the gate would
 * silence the command in every offline harness we measure with. Having heard from one and then
 * losing it means we are on the car and the device went quiet, in which case it is not passing
 * torque either. So: never heard, open; heard and lost, closed.
 */
class AssistGate {
public:
  void onReport(bool allowed, int64_t now_us);

  struct Result {
    bool allowed = false;
    bool known = false;
    bool changed = false;
  };

  Result update(int64_t now_us, double max_age_s);

  bool lastReportAllowed() const { return allowed_; }

private:
  bool allowed_ = false;
  int64_t ts_us_ = 0;
  bool have_ = false;
  bool absent_ = false;
};

}  // namespace adas
