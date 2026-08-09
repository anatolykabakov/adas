#include "adas/utils/path_lateral_state.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace adas {

PathLateralState estimatePathLateralState(const std::vector<Vec2>& polyline_ego, double x_min_m, double x_max_m)
{
  PathLateralState out;
  std::vector<Vec2> pts;
  pts.reserve(polyline_ego.size());
  for (const auto& p : polyline_ego) {
    if (p.x() < x_min_m || p.x() > x_max_m)
      continue;
    pts.push_back(p);
  }
  out.n_points = static_cast<int>(pts.size());
  if (pts.size() < 5)
    return out;

  // Least-squares fit y = a x² + b x + c  (y right+)
  // Normal equations for [a,b,c].
  double s0 = 0, sx = 0, sx2 = 0, sx3 = 0, sx4 = 0;
  double sy = 0, sxy = 0, sx2y = 0;
  for (const auto& p : pts) {
    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x2 * x2;
    s0 += 1.0;
    sx += x;
    sx2 += x2;
    sx3 += x3;
    sx4 += x4;
    sy += y;
    sxy += x * y;
    sx2y += x2 * y;
  }

  // Solve 3x3 via Cramer's / elimination
  // | sx4 sx3 sx2 | |a|   |sx2y|
  // | sx3 sx2 sx  | |b| = |sxy |
  // | sx2 sx  s0  | |c|   |sy  |
  const double A11 = sx4, A12 = sx3, A13 = sx2;
  const double A21 = sx3, A22 = sx2, A23 = sx;
  const double A31 = sx2, A32 = sx, A33 = s0;
  const double det = A11 * (A22 * A33 - A23 * A32) - A12 * (A21 * A33 - A23 * A31) + A13 * (A21 * A32 - A22 * A31);
  if (std::abs(det) < 1e-12)
    return out;

  const double det_a = sx2y * (A22 * A33 - A23 * A32) - A12 * (sxy * A33 - A23 * sy) + A13 * (sxy * A32 - A22 * sy);
  const double det_b = A11 * (sxy * A33 - A23 * sy) - sx2y * (A21 * A33 - A23 * A31) + A13 * (A21 * sy - sxy * A31);
  const double det_c = A11 * (A22 * sy - sxy * A32) - A12 * (A21 * sy - sxy * A31) + sx2y * (A21 * A32 - A22 * A31);

  const double a = det_a / det;
  const double b = det_b / det;
  const double c = det_c / det;

  out.path_a = a;
  out.path_b = b;
  out.path_c = c;

  out.cte_m = -c;
  out.epsi_rad = -std::atan(b);
  const double yp = b;
  const double ypp = 2.0 * a;
  const double kappa_right = ypp / std::pow(1.0 + yp * yp, 1.5);
  out.kappa = -kappa_right;
  out.valid = std::abs(out.cte_m) < 3.5;
  return out;
}

}  // namespace adas
