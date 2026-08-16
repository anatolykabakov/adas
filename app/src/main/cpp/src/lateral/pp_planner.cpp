#include "adas/lateral/pp_planner.h"

#include <algorithm>
#include <cmath>

#include "adas/lateral/limits.h"
#include "adas/utils/path_lateral_state.h"

namespace adas {
namespace lateral {
namespace {
/** Circle-segment intersections. The formulas solve the same system as substituting the line into
 *  the circle equation, but through a determinant, so the vertical segment — where the slope goes
 *  to infinity — is not lost. */
std::vector<Vec2> circleSegmentIntersections(double radius, const Vec2& pt1, const Vec2& pt2)
{
  const Vec2 d = pt2 - pt1;
  const double dr = d.norm();
  if (dr < 1e-12)
    return {};

  const double big_d = pt1.x() * pt2.y() - pt2.x() * pt1.y();
  const double discriminant = radius * radius * dr * dr - big_d * big_d;
  if (discriminant < 0.0)
    return {};

  const double dx = d.x();
  const double dy = d.y();
  const int dy_sign = dy < 0.0 ? -1 : 1;
  const double sqrt_disc = std::sqrt(discriminant);

  std::vector<Vec2> hits;
  for (const int sign : (dy < 0.0 ? std::vector<int>{1, -1} : std::vector<int>{-1, 1})) {
    hits.emplace_back((big_d * dy + sign * dy_sign * dx * sqrt_disc) / (dr * dr),
                      (-big_d * dx + sign * std::abs(dy) * sqrt_disc) / (dr * dr));
  }

  std::vector<Vec2> on_segment;
  for (const auto& pt : hits) {
    const Vec2 delta = pt - pt1;
    const double frac = std::abs(dx) > std::abs(dy) ? delta.x() / dx : delta.y() / dy;
    if (frac >= 0.0 && frac <= 1.0)
      on_segment.push_back(pt);
  }
  if (on_segment.size() == 2 && std::abs(discriminant) <= 1e-9)
    on_segment.resize(1);
  return on_segment;
}

}  // namespace

std::optional<Vec2> PpPlanner::targetPoint(double lookahead_m, const std::vector<Vec2>& poly)
{
  if (poly.size() < 2)
    return std::nullopt;
  for (std::size_t j = 0; j + 1 < poly.size(); ++j) {
    for (const auto& hit : circleSegmentIntersections(lookahead_m, poly[j], poly[j + 1])) {
      if (hit.x() > 0.0)
        return hit;
    }
  }
  return std::nullopt;
}

double PpPlanner::lookaheadFor(double speed_mps, const std::vector<Vec2>& poly) const
{
  double ld_min = cfg_.ld_min;
  if (cfg_.ld_curv_gain > 0.0) {
    const auto lat = estimatePathLateralState(poly);
    if (lat.valid)
      ld_min = cfg_.ld_min / (1.0 + cfg_.ld_curv_gain * std::abs(lat.kappa));
  }
  return std::clamp(cfg_.k_dd * speed_mps, std::min(ld_min, cfg_.ld_max), cfg_.ld_max);
}

Output PpPlanner::update(const Input& in)
{
  Output out;
  out.controller = "pp";
  out.dbg.speed_mps = in.speed_mps;
  out.dbg.n_points = static_cast<int>(in.polyline_ego.size());

  if (in.polyline_ego.size() < 2) {
    out.status = "no_polyline";
    return out;
  }

  const double speed = std::max(0.0, in.speed_mps);
  const double lookahead_m = lookaheadFor(speed, in.polyline_ego);
  out.lookahead_m = lookahead_m;
  out.dbg.pp_lookahead_m = lookahead_m;

  std::vector<Vec2> poly_rear_axle = in.polyline_ego;
  for (auto& p : poly_rear_axle)
    p.x() += cfg_.waypoint_shift;

  const auto target = targetPoint(lookahead_m, poly_rear_axle);
  if (target) {
    const double alpha_rad = std::atan2(target->y(), target->x());
    const double wheelbase = std::max(cfg_.vehicle.wheelbase_m, 1e-6);
    out.steer_rad = std::atan(2.0 * wheelbase * std::sin(alpha_rad) / std::max(lookahead_m, 1e-3));
    out.curvature = std::tan(out.steer_rad) / wheelbase;
    out.has_target = true;
    out.target_x = target->x() - cfg_.waypoint_shift;
    out.target_y = target->y();
  }
  out.dbg.pp_steer_raw_rad = out.steer_rad;

  const double lim = cfg_.max_steer_rad;
  out.max_steer_rad = lim;
  out.dbg.max_steer_rad = lim;
  if (lim > 1e-6) {
    out.steer_rad = std::clamp(out.steer_rad, -lim, lim);
    out.steer_norm = out.steer_rad / lim;
  }

  out.ok = true;
  out.status = "ok";
  return out;
}

}  // namespace lateral
}  // namespace adas
