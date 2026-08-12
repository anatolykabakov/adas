#pragma once

#include <algorithm>
#include <cstdint>

namespace adas {
/**
 * @brief Smoothed interval between messages, taken from their timestamps.
 *
 * The source rate enters the filter, rate-limit and PID formulas. A configured constant will not do
 * — vision slows down under load and chassis arrives unevenly — and the raw difference will not do
 * either: one dropped frame would give a step several times larger and a jerk on the output. So the
 * difference is clamped and smoothed, and a large enough gap counts as a break in the stream.
 */
class IntervalFilter {
public:
  struct Config {
    double nominal_s = 0.05;
    double min_s = 0.02;
    double max_s = 0.5;
    double gap_s = 0.5;
    double alpha = 0.3;
  };

  IntervalFilter() = default;
  explicit IntervalFilter(Config cfg) : cfg_(cfg), value_s_(cfg.nominal_s) {}

  double value() const { return value_s_; }
  bool primed() const { return last_ts_us_ > 0; }

  void reset()
  {
    value_s_ = cfg_.nominal_s;
    last_ts_us_ = 0;
  }

  /// A missing timestamp returns the nominal: a source that does not stamp must not set the step.
  double update(int64_t ts_us)
  {
    if (ts_us <= 0) {
      value_s_ = cfg_.nominal_s;
      return value_s_;
    }
    if (last_ts_us_ > 0) {
      const double raw = static_cast<double>(ts_us - last_ts_us_) * 1e-6;
      if (raw > 0.0 && raw <= cfg_.gap_s)
        value_s_ = cfg_.alpha * std::clamp(raw, cfg_.min_s, cfg_.max_s) + (1.0 - cfg_.alpha) * value_s_;
      else
        value_s_ = cfg_.nominal_s;
    }
    last_ts_us_ = ts_us;
    return value_s_;
  }

  /** The same, but a gap keeps the previous interval.
   *
   *  For chassis: a gap there means the link to the car was lost, and returning the PID rate to the
   *  nominal would rescale its gains by a rate that never happened. */
  double updateHoldingOnGap(int64_t ts_us)
  {
    if (ts_us <= 0)
      return value_s_;
    if (last_ts_us_ > 0) {
      const double raw = static_cast<double>(ts_us - last_ts_us_) * 1e-6;
      if (raw > 0.0 && raw <= cfg_.gap_s)
        value_s_ = cfg_.alpha * std::clamp(raw, cfg_.min_s, cfg_.max_s) + (1.0 - cfg_.alpha) * value_s_;
    }
    last_ts_us_ = ts_us;
    return value_s_;
  }

private:
  Config cfg_{};
  double value_s_ = 0.05;
  int64_t last_ts_us_ = 0;
};

/**
 * @brief Gate with hysteresis: opens above one threshold, closes below another.
 *
 * A single threshold for both directions would chatter exactly where the value sits near it — for
 * instance, speed in traffic around the control threshold.
 */
class HysteresisGate {
public:
  HysteresisGate() = default;
  HysteresisGate(double close_below, double open_above) : close_below_(close_below), open_above_(open_above) {}

  bool isOpen() const { return open_; }

  void setThresholds(double close_below, double open_above)
  {
    close_below_ = close_below;
    open_above_ = open_above;
  }

  void reset() { open_ = false; }

  bool update(double value)
  {
    open_ = open_ ? !(value < close_below_) : !(value < open_above_);
    return open_;
  }

private:
  double close_below_ = 0.0;
  double open_above_ = 0.0;
  bool open_ = false;
};

}  // namespace adas
