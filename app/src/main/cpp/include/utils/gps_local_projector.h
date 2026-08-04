#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utils/adas_topics.h"
#include "utils/math_utils.h"

namespace adas {

class GpsLocalProjector {
public:
  void reset()
  {
    have_origin_ = false;
    lat0_deg_ = lon0_deg_ = 0;
  }

  GpsSample project(int64_t timestamp_us, double lat_deg, double lon_deg, bool valid_fix = true, double speed_mps = 0.0,
                    double bearing_deg = 0.0)
  {
    GpsSample s;
    s.timestamp_us = timestamp_us;
    s.speed_mps = speed_mps;
    s.bearing_deg = bearing_deg;
    if (!valid_fix || !std::isfinite(lat_deg) || !std::isfinite(lon_deg)) {
      s.valid = false;
      return s;
    }
    if (!have_origin_) {
      lat0_deg_ = lat_deg;
      lon0_deg_ = lon_deg;
      have_origin_ = true;
    }
    constexpr double kR = 6371000.0;
    const double dlat = (lat_deg - lat0_deg_) * (M_PI / 180.0);
    const double dlon = (lon_deg - lon0_deg_) * (M_PI / 180.0);
    const double lat0 = lat0_deg_ * (M_PI / 180.0);
    s.y = dlat * kR;                   // north
    s.x = dlon * std::cos(lat0) * kR;  // east
    s.valid = true;

    // Course usable when moving; ignore bearing==0 at standstill (common GPS quirk).
    s.course_valid = std::isfinite(speed_mps) && std::isfinite(bearing_deg) && speed_mps > 2.0 &&
                     !(speed_mps < 0.5 && std::abs(bearing_deg) < 1e-3);
    if (s.course_valid) {
      s.yaw_enu = yawEnuFromBearingDeg(bearing_deg);
      const double br = bearing_deg * (M_PI / 180.0);
      s.vx = speed_mps * std::sin(br);  // east
      s.vy = speed_mps * std::cos(br);  // north
    }
    return s;
  }

  bool haveOrigin() const { return have_origin_; }
  double lat0() const { return lat0_deg_; }
  double lon0() const { return lon0_deg_; }

private:
  bool have_origin_ = false;
  double lat0_deg_ = 0;
  double lon0_deg_ = 0;
};

}  // namespace adas
