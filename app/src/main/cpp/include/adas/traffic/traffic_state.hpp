#pragma once

#include <cstdint>
#include <string>

namespace adas {
namespace traffic {
struct Config {
  int64_t speed_limit_hold_ms = 30000;  ///< How long a seen limit stays in force without being seen again [ms].
  int64_t tfl_hold_ms = 2000;           ///< How long a traffic-light state is held after the last detection [ms].
  double overspeed_margin_kmh = 5.0;    ///< Tolerated excess over the limit before it counts as speeding [km/h].
};

struct State {
  int speed_limit_kmh = 0;
  std::string speed_limit_label;
  int64_t speed_limit_ts_ms = 0;

  int tfl_color = 0;
  float tfl_conf = 0.f;
  int64_t tfl_ts_ms = 0;

  int n_dets = 0;
  std::string status = "init";
};

struct Assessment {
  double v_kmh = 0.0;
  bool overspeed = false;
  float overspeed_kmh = 0.f;
  int speed_limit_age_ms = -1;
  int tfl_age_ms = -1;
};

/** Forget what stopped being true: the sign is behind us, the light is out of view, the signal
 *  changed. Without this the state lives forever and lies from a stale observation. */
inline void expire(State& s, int64_t now_ms, const Config& cfg)
{
  if (s.speed_limit_kmh > 0 && s.speed_limit_ts_ms > 0 && (now_ms - s.speed_limit_ts_ms) > cfg.speed_limit_hold_ms) {
    s.speed_limit_kmh = 0;
    s.speed_limit_label.clear();
  }
  if (s.tfl_ts_ms > 0 && (now_ms - s.tfl_ts_ms) > cfg.tfl_hold_ms) {
    s.tfl_color = 0;
    s.tfl_conf = 0.f;
  }
}

/// Apply the hold times to the fused state. \return What the HUD should show now.
inline Assessment assess(const State& s, double speed_mps, bool have_speed, int64_t now_ms, const Config& cfg)
{
  Assessment a;
  a.v_kmh = have_speed ? speed_mps * 3.6 : 0.0;
  if (s.speed_limit_kmh > 0 && a.v_kmh > s.speed_limit_kmh + cfg.overspeed_margin_kmh) {
    a.overspeed = true;
    a.overspeed_kmh = static_cast<float>(a.v_kmh - s.speed_limit_kmh);
  }
  a.speed_limit_age_ms = s.speed_limit_ts_ms > 0 ? static_cast<int>(now_ms - s.speed_limit_ts_ms) : -1;
  a.tfl_age_ms = s.tfl_ts_ms > 0 ? static_cast<int>(now_ms - s.tfl_ts_ms) : -1;
  return a;
}

}  // namespace traffic
}  // namespace adas
