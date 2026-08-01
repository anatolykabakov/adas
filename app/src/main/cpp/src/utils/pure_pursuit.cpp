#include "utils/pure_pursuit.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "utils/path_lateral_state.h"

namespace adas {
namespace {

std::vector<Vec2> circleLineSegmentIntersection(const Vec2& center, double radius, const Vec2& pt1, const Vec2& pt2,
                                                bool full_line)
{
  const Vec2 p1 = pt1 - center;
  const Vec2 p2 = pt2 - center;
  const Vec2 d = p2 - p1;
  const double dr = d.norm();
  if (dr < 1e-12)
    return {};

  const double big_d = p1.x() * p2.y() - p2.x() * p1.y();
  const double discriminant = radius * radius * dr * dr - big_d * big_d;
  if (discriminant < 0)
    return {};

  std::vector<Vec2> intersections;
  const double dy = d.y();
  const double dx = d.x();
  const int dy_sign = (dy < 0) ? -1 : 1;
  const double sqrt_disc = std::sqrt(discriminant);
  for (int sign : ((dy < 0) ? std::vector<int>{1, -1} : std::vector<int>{-1, 1})) {
    const Vec2 local((big_d * dy + sign * dy_sign * dx * sqrt_disc) / (dr * dr),
                     (-big_d * dx + sign * std::abs(dy) * sqrt_disc) / (dr * dr));
    intersections.push_back(center + local);
  }

  if (!full_line) {
    std::vector<Vec2> filtered;
    for (const auto& pt : intersections) {
      const Vec2 delta = pt - pt1;
      const double frac = (std::abs(dx) > std::abs(dy)) ? delta.x() / dx : delta.y() / dy;
      if (frac >= 0.0 && frac <= 1.0)
        filtered.push_back(pt);
    }
    intersections.swap(filtered);
  }
  if (intersections.size() == 2 && std::abs(discriminant) <= 1e-9) {
    intersections.resize(1);
  }
  return intersections;
}

}  // namespace

double PurePursuitResult::curvature() const { return std::tan(steer_rad) / std::max(wheel_base, 1e-6); }

PurePursuit::PurePursuit(double K_dd_, double wheel_base_, double waypoint_shift_, double ld_min_, double ld_max_,
                         double ld_curv_gain_)
  : K_dd(K_dd_)
  , wheel_base(wheel_base_)
  , waypoint_shift(waypoint_shift_)
  , ld_min(ld_min_)
  , ld_max(ld_max_)
  , ld_curv_gain(ld_curv_gain_)
{
}

std::optional<Vec2> getTargetPoint(double lookahead, const std::vector<Vec2>& polyline)
{
  if (polyline.size() < 2)
    return std::nullopt;
  std::vector<Vec2> intersections;
  for (size_t j = 0; j + 1 < polyline.size(); ++j) {
    auto hits = circleLineSegmentIntersection(Vec2::Zero(), lookahead, polyline[j], polyline[j + 1], false);
    intersections.insert(intersections.end(), hits.begin(), hits.end());
  }
  for (const auto& p : intersections) {
    if (p.x() > 0.0)
      return p;
  }
  return std::nullopt;
}

PurePursuitResult PurePursuit::compute(const std::vector<Vec2>& polyline_ego, double speed_mps) const
{
  PurePursuitResult out;
  out.speed_mps = std::max(0.0, speed_mps);
  out.wheel_base = wheel_base;

  double ld_min_eff = ld_min;
  if (ld_curv_gain > 0.0) {
    const auto lat = estimatePathLateralState(polyline_ego);
    if (lat.valid)
      ld_min_eff = ld_min / (1.0 + ld_curv_gain * std::abs(lat.kappa));
  }
  out.lookahead_m = std::clamp(K_dd * out.speed_mps, std::min(ld_min_eff, ld_max), ld_max);

  std::vector<Vec2> poly_ra = polyline_ego;
  for (auto& p : poly_ra)
    p.x() += waypoint_shift;

  out.target_ra = getTargetPoint(out.lookahead_m, poly_ra);
  if (out.target_ra) {
    out.alpha_rad = std::atan2(out.target_ra->y(), out.target_ra->x());
    out.steer_rad = std::atan((2.0 * wheel_base * std::sin(out.alpha_rad)) / std::max(out.lookahead_m, 1e-3));
    out.target_ego = Vec2(out.target_ra->x() - waypoint_shift, out.target_ra->y());
  }
  return out;
}

}  // namespace adas
