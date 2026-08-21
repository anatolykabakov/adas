#pragma once

#include <algorithm>
#include <cstdint>

namespace adas {
/** Smoothed interval between messages, taken from their timestamps. */
class IntervalFilter {
public:
  struct Config {
    double nominal_s = 0.05;  ///< Value used before anything is measured and after a gap [s].
    double min_s = 0.02;      ///< Lower clamp on a measured interval [s].
    double max_s = 0.5;       ///< Upper clamp on a measured interval [s].
    double gap_s = 0.5;       ///< Longer than this and the stream counts as broken, not slow [s].
    double alpha = 0.3;       ///< Weight of the new sample, in [0, 1].
  };

  IntervalFilter() = default;
  /// \param[in] cfg Nominal value and smoothing.
  explicit IntervalFilter(Config cfg) : cfg_(cfg), value_s_(cfg.nominal_s) {}

  /// \return Filtered interval [s].
  double value() const { return value_s_; }
  /// \return True after the first sample.
  bool primed() const { return last_ts_us_ > 0; }

  /// Back to the nominal interval.
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

/** Gate with hysteresis: opens above one threshold, closes below another. */
class HysteresisGate {
public:
  HysteresisGate() = default;
  /// \param[in] close_below / open_above Thresholds; the gap is the hysteresis.
  HysteresisGate(double close_below, double open_above) : close_below_(close_below), open_above_(open_above) {}

  /// \return Current gate state.
  bool isOpen() const { return open_; }

  /// Replace the thresholds.
  void setThresholds(double close_below, double open_above)
  {
    close_below_ = close_below;
    open_above_ = open_above;
  }

  /// Close the gate.
  void reset() { open_ = false; }

  /// Feed a value. \return The gate state after it.
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
