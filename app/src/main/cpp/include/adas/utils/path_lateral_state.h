#pragma once

#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
struct PathLateralState {
  bool valid = false;
  double cte_m = 0.0;
  double epsi_rad = 0.0;
  double kappa = 0.0;
  double path_a = 0.0;
  double path_b = 0.0;
  double path_c = 0.0;
  int n_points = 0;
};

PathLateralState estimatePathLateralState(const std::vector<Vec2>& polyline_ego, double x_min_m = 1.0,
                                          double x_max_m = 12.0);

}  // namespace adas
