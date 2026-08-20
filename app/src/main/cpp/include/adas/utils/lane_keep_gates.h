#pragma once

#include <cstdint>

namespace adas {
/// The reference line is stale: the command came from a picture no longer worth trusting.
class StaleGate {
public:
  /// `max_age_s <= 0` disables the rule.
  bool update(int64_t now_us, int64_t capture_ts_us, double max_age_s, double& age_s_out);

  /// \return True on the tick the verdict flipped.
  bool justChanged() const { return just_changed_; }
  /// \return True when the report aged out.
  bool stale() const { return stale_; }

private:
  bool stale_ = false;
  bool just_changed_ = false;
};

/** Turn signal on: the wheel goes back to the driver. */
class BlinkerGate {
public:
  struct Result {
    bool suppressed = false;
    bool changed = false;
    double since_off_s = 0.0;
    bool left = false;
  };

  /// Blinker gate tick. \return Whether steering is suppressed, with the resume delay applied.
  Result update(int64_t now_us, bool left, bool right, double resume_delay_s);

  /// \return Current suppression state.
  bool suppressed() const { return suppressed_; }

private:
  int64_t off_us_ = 0;
  bool off_armed_ = false;
  bool suppressed_ = false;
};

/** Whether torque is reaching the rack, according to the panda. */
class AssistGate {
public:
  /// Record the panda's actuation verdict at \p now_us.
  void onReport(bool allowed, int64_t now_us);

  struct Result {
    bool allowed = false;
    bool known = false;
    bool changed = false;
  };

  /// Assist gate tick. \return Allowed/known given the report age.
  Result update(int64_t now_us, double max_age_s);

  /// \return The last verdict received.
  bool lastReportAllowed() const { return allowed_; }

private:
  bool allowed_ = false;
  int64_t ts_us_ = 0;
  bool have_ = false;
  bool absent_ = false;
};

}  // namespace adas
