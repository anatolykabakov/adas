#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace adas {
namespace mapmatch {

struct LocalFrame {
  double lat0_deg = 55.7539;
  double lon0_deg = 37.6208;

  static constexpr double kA = 6378137.0;
  static constexpr double kF = 1.0 / 298.257223563;
  static constexpr double kE2 = kF * (2.0 - kF);

  std::pair<double, double> toLocal(double lat_deg, double lon_deg) const
  {
    double x, y, z, x0, y0, z0;
    ecef(lat_deg, lon_deg, x, y, z);
    ecef(lat0_deg, lon0_deg, x0, y0, z0);
    const double dx = x - x0, dy = y - y0, dz = z - z0;

    const double lat0 = lat0_deg * M_PI / 180.0;
    const double lon0 = lon0_deg * M_PI / 180.0;
    const double sla = std::sin(lat0), cla = std::cos(lat0);
    const double slo = std::sin(lon0), clo = std::cos(lon0);

    const double east = -slo * dx + clo * dy;
    const double north = -sla * clo * dx - sla * slo * dy + cla * dz;
    return {east, north};
  }

  void toLocalMany(const std::vector<double>& lat_deg, const std::vector<double>& lon_deg, std::vector<double>& east,
                   std::vector<double>& north) const
  {
    const std::size_t n = std::min(lat_deg.size(), lon_deg.size());
    east.resize(n);
    north.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const auto [e, nn] = toLocal(lat_deg[i], lon_deg[i]);
      east[i] = e;
      north[i] = nn;
    }
  }

  std::pair<double, double> toGeo(double east, double north) const
  {
    double lat = lat0_deg, lon = lon0_deg;
    for (int i = 0; i < 4; ++i) {
      const auto [e, n] = toLocal(lat, lon);
      const double lat_rad = lat * M_PI / 180.0;
      const double s = std::sin(lat_rad);
      const double m_rad = kA * (1.0 - kE2) / std::pow(1.0 - kE2 * s * s, 1.5);
      const double p_rad = kA / std::sqrt(1.0 - kE2 * s * s) * std::cos(lat_rad);
      lat += (north - n) / m_rad * 180.0 / M_PI;
      lon += (east - e) / p_rad * 180.0 / M_PI;
    }
    return {lat, lon};
  }

private:
  void ecef(double lat_deg, double lon_deg, double& x, double& y, double& z) const
  {
    const double lat = lat_deg * M_PI / 180.0;
    const double lon = lon_deg * M_PI / 180.0;
    const double s = std::sin(lat), c = std::cos(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * s * s);
    x = n * c * std::cos(lon);
    y = n * c * std::sin(lon);
    z = n * (1.0 - kE2) * s;
  }
};

}  // namespace mapmatch
}  // namespace adas
