#pragma once

#include <algorithm>

namespace adas {
namespace mapmatch {

/**
 * \brief Squared distance from a point to a segment, and optionally the closest point on it.
 * \param[in] px,py Point to measure from.
 * \param[in] ax,ay Segment start.
 * \param[in] bx,by Segment end.
 * \param[out] qx,qy Closest point on the segment; ignored when null.
 * \return Squared distance [m²] — squared to keep the hot loop free of `sqrt`.
 */
inline double distSqToSegment(double px, double py, double ax, double ay, double bx, double by, double* qx = nullptr,
                              double* qy = nullptr)
{
  const double vx = bx - ax, vy = by - ay;
  const double wx = px - ax, wy = py - ay;
  const double vv = vx * vx + vy * vy;
  double t = vv > 1e-12 ? (wx * vx + wy * vy) / vv : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  const double dx = wx - t * vx, dy = wy - t * vy;
  if (qx)
    *qx = ax + t * vx;
  if (qy)
    *qy = ay + t * vy;
  return dx * dx + dy * dy;
}

}  // namespace mapmatch
}  // namespace adas
