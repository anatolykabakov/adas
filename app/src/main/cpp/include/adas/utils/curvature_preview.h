#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
inline double maxCurvatureAhead(const std::vector<Vec2>& polyline, double from_m, double to_m, double window_m = 25.0,
                                double percentile = 0.8)
{
  if (polyline.size() < 5 || to_m <= from_m + window_m)
    return 0.0;

  const double slide_m = std::max(2.0, 0.2 * window_m);
  std::vector<double> kappas;

  for (double w0 = from_m; w0 + window_m <= to_m + 1e-9; w0 += slide_m) {
    const double w1 = w0 + window_m;
    const double xc = 0.5 * (w0 + w1);

    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    double t0 = 0, t1 = 0, t2 = 0;
    int n = 0;
    for (const auto& p : polyline) {
      if (p.x() < w0 || p.x() > w1)
        continue;
      const double dx = p.x() - xc;
      const double dx2 = dx * dx;
      s0 += 1.0;
      s1 += dx;
      s2 += dx2;
      s3 += dx2 * dx;
      s4 += dx2 * dx2;
      t0 += p.y();
      t1 += dx * p.y();
      t2 += dx2 * p.y();
      ++n;
    }
    if (n < 5)
      continue;

    const double d = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
    if (std::abs(d) < 1e-12)
      continue;
    const double c = (t0 * (s2 * s4 - s3 * s3) - s1 * (t1 * s4 - s3 * t2) + s2 * (t1 * s3 - s2 * t2)) / d;
    const double b = (s0 * (t1 * s4 - s3 * t2) - t0 * (s1 * s4 - s3 * s2) + s2 * (s1 * t2 - t1 * s2)) / d;
    const double a = (s0 * (s2 * t2 - t1 * s3) - s1 * (s1 * t2 - t1 * s2) + t0 * (s1 * s3 - s2 * s2)) / d;
    (void)c;

    const double slope2 = 1.0 + b * b;
    kappas.push_back(std::abs(2.0 * a) / (slope2 * std::sqrt(slope2)));
  }

  if (kappas.empty())
    return 0.0;
  std::sort(kappas.begin(), kappas.end());
  const double p = std::clamp(percentile, 0.0, 1.0);
  const size_t idx = static_cast<size_t>(std::lround(p * static_cast<double>(kappas.size() - 1)));
  return kappas[idx];
}

inline double curvatureSpeedLimit(double kappa, double a_lat_max, double v_max)
{
  const double k = std::abs(kappa);
  if (k < 1e-5 || a_lat_max <= 0.0)
    return v_max;
  return std::min(v_max, std::sqrt(a_lat_max / k));
}

}  // namespace adas
