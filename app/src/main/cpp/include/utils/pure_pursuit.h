#pragma once

#include <optional>
#include <vector>

#include "utils/math_utils.h"

namespace adas {

struct PurePursuitResult {
  double lookahead_m = 0.0;
  std::optional<Vec2> target_ra;
  std::optional<Vec2> target_ego;
  double alpha_rad = 0.0;
  double steer_rad = 0.0;
  double speed_mps = 0.0;
  double wheel_base = 2.636;

  double curvature() const;
};

class PurePursuit {
public:
  PurePursuit(double K_dd = 0.4, double wheel_base = 2.636, double waypoint_shift = 1.4, double ld_min = 3.0,
              double ld_max = 20.0, double ld_curv_gain = 0.0);

  PurePursuitResult compute(const std::vector<Vec2>& polyline_ego, double speed_mps) const;

  double K_dd = 0.4;
  double wheel_base = 2.636;
  double waypoint_shift = 1.4;
  double ld_min = 3.0;
  double ld_max = 20.0;
  double ld_curv_gain = 0.0;
};

std::optional<Vec2> getTargetPoint(double lookahead, const std::vector<Vec2>& polyline);

}  // namespace adas
