#pragma once

#include <algorithm>
#include <cmath>

namespace adas {
namespace lateral {
/** Rate limit on the command between vision frames. */
class SlewGuard {
public:
  struct Config {
    double limit_deg = 0.0;         ///< Hard rate limit on the command [deg/s]; 0 leaves only the jerk bound.
    double max_lateral_jerk = 5.0;  ///< Jerk bound the limit is derived from at speed [m/s^3].
    double rate_min_speed = 2.0;    ///< Speed floor used in that derivation [m/s].
    double Lf = 2.67;               ///< Lever arm converting a curvature step into an angle step [m].
    int max_gap_frames = 10;        ///< Frames without a command after which the guard forgets its previous value.
  };

  SlewGuard() = default;
  /// \param[in] cfg Rate limits and the jerk bound.
  explicit SlewGuard(Config cfg) : cfg_(cfg) {}

  /// Replace the config.
  void setConfig(const Config& cfg) { cfg_ = cfg; }

  /// Forget the previous command, so the next one is not rate-limited against it.
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
