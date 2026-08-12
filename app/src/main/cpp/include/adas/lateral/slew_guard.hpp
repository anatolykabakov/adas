#pragma once

#include <algorithm>
#include <cmath>

namespace adas {
namespace lateral {
/**
 * @brief Rate limit on the command between vision frames.
 *
 * It sits after the planner rather than inside one: the planner decides where to go, and this cuts
 * the jump a driver would feel on the wheel, whichever strategy produced it.
 *
 * The ceiling is the smaller of two: the configured limit in degrees and the lateral-jerk limit.
 * The second depends on speed, because at speed the same angle gives far more lateral acceleration.
 */
class SlewGuard {
public:
  struct Config {
    double limit_deg = 0.0;
    double max_lateral_jerk = 5.0;
    double rate_min_speed = 2.0;
    double Lf = 2.67;
    int max_gap_frames = 10;
  };

  SlewGuard() = default;
  explicit SlewGuard(Config cfg) : cfg_(cfg) {}

  void setConfig(const Config& cfg) { cfg_ = cfg; }

  void reset()
  {
    have_prev_ = false;
    gap_frames_ = 0;
  }

  /** Clamp the command and remember it; true when the limit bit.
   *
   *  `commanded` says whether this frame has a command at all: a frame without one sets no
   *  reference point but brings the history closer to being forgotten. */
  bool apply(double& steer_rad, double speed_mps, double frame_dt_s, bool commanded)
  {
    bool clipped = false;
    if (commanded && have_prev_ && cfg_.limit_deg > 1e-9) {
      const double v_eff = std::max(speed_mps, cfg_.rate_min_speed);
      const double dkappa_max = cfg_.max_lateral_jerk / (v_eff * v_eff) * frame_dt_s;
      const double ceil_rad = cfg_.limit_deg * M_PI / 180.0;
      const double slew = std::min(dkappa_max * cfg_.Lf, ceil_rad);
      const double want = steer_rad - last_rad_;
      const double got = std::clamp(want, -slew, slew);
      if (got != want) {
        steer_rad = last_rad_ + got;
        clipped = true;
      }
    }

    if (commanded) {
      last_rad_ = steer_rad;
      have_prev_ = true;
      gap_frames_ = 0;
    } else if (have_prev_ && ++gap_frames_ > cfg_.max_gap_frames) {
      have_prev_ = false;
    }
    return clipped;
  }

private:
  Config cfg_{};
  double last_rad_ = 0.0;
  bool have_prev_ = false;
  int gap_frames_ = 0;
};

}  // namespace lateral
}  // namespace adas
