#include "adas/mapmatch/road_route.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace adas {
namespace mapmatch {
namespace {

constexpr double kDeg = M_PI / 180.0;

/** Closest point on segment a->b, plus the parameter t along it. */
double distSqToSegment(double px, double py, double ax, double ay, double bx, double by, double& qx, double& qy,
                       double& t)
{
  const double vx = bx - ax, vy = by - ay;
  const double wx = px - ax, wy = py - ay;
  const double vv = vx * vx + vy * vy;
  t = vv > 1e-12 ? (wx * vx + wy * vy) / vv : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  qx = ax + t * vx;
  qy = ay + t * vy;
  const double dx = px - qx, dy = py - qy;
  return dx * dx + dy * dy;
}

/** Directed edges whose geometry passes within `search_m` of (x, y), one-way applied. `outgoing()` cannot be
 *  used here — it needs a node, and we only have a position. */
std::vector<std::uint32_t> candidatesNear(const RoadMap& map, double x, double y, double search_m)
{
  std::vector<std::uint32_t> out;
  for (const std::uint32_t ei : map.edgesInBBox(x - search_m, y - search_m, x + search_m, y + search_m)) {
    out.push_back(makeDir(ei, false));
    if (!map.edge(ei).oneway)
      out.push_back(makeDir(ei, true));
  }
  return out;
}

}  // namespace

void resamplePolyline(const std::vector<double>& xs, const std::vector<double>& ys, double step_m,
                      std::vector<double>& out_s, std::vector<double>& out_x, std::vector<double>& out_y)
{
  out_s.clear();
  out_x.clear();
  out_y.clear();
  if (xs.size() < 2 || xs.size() != ys.size() || step_m <= 0.0)
    return;

  out_s.push_back(0.0);
  out_x.push_back(xs[0]);
  out_y.push_back(ys[0]);

  double carry = 0.0;  // distance already walked past the last emitted sample
  double s_total = 0.0;
  for (std::size_t k = 1; k < xs.size(); ++k) {
    const double dx = xs[k] - xs[k - 1];
    const double dy = ys[k] - ys[k - 1];
    const double len = std::hypot(dx, dy);
    if (len < 1e-9)
      continue;

    double pos = step_m - carry;  // distance into this segment of the next sample
    while (pos <= len) {
      const double u = pos / len;
      s_total = out_s.back() + step_m;
      out_s.push_back(s_total);
      out_x.push_back(xs[k - 1] + u * dx);
      out_y.push_back(ys[k - 1] + u * dy);
      pos += step_m;
    }
    carry = len - (pos - step_m);
  }
}

HeadingProfile headingProfile(const std::vector<double>& xs, const std::vector<double>& ys)
{
  HeadingProfile hp;
  const std::size_t n = std::min(xs.size(), ys.size());
  if (n < 2)
    return hp;

  hp.s_m.reserve(n - 1);
  hp.theta_rad.reserve(n - 1);

  double run = 0.0;
  double unwrapped = 0.0;
  bool first = true;
  for (std::size_t i = 1; i < n; ++i) {
    const double dx = xs[i] - xs[i - 1];
    const double dy = ys[i] - ys[i - 1];
    const double len = std::hypot(dx, dy);
    if (len < 1e-9)
      continue;
    const double th = std::atan2(dy, dx);
    unwrapped = first ? th : unwrapped + wrapPi(th - unwrapped);
    first = false;
    hp.s_m.push_back(run + 0.5 * len);  // midpoint: where this chord's heading is the true heading
    hp.theta_rad.push_back(unwrapped);
    run += len;
  }
  hp.length_m = run;
  return hp;
}

namespace {

double thetaAt(const HeadingProfile& hp, double s)
{
  if (hp.s_m.empty())
    return 0.0;
  if (s <= hp.s_m.front())
    return hp.theta_rad.front();
  if (s >= hp.s_m.back())
    return hp.theta_rad.back();
  const auto it = std::upper_bound(hp.s_m.begin(), hp.s_m.end(), s);
  const std::size_t i = static_cast<std::size_t>(it - hp.s_m.begin());
  const double s0 = hp.s_m[i - 1], s1 = hp.s_m[i];
  const double w = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
  return hp.theta_rad[i - 1] + w * (hp.theta_rad[i] - hp.theta_rad[i - 1]);
}

}  // namespace

std::vector<double> curvatureAlong(const std::vector<double>& src_x, const std::vector<double>& src_y,
                                   const std::vector<double>& query_s, double window_m)
{
  std::vector<double> kappa(query_s.size(), 0.0);
  const HeadingProfile hp = headingProfile(src_x, src_y);
  if (hp.s_m.size() < 2 || window_m <= 0.0)
    return kappa;

  const double half = 0.5 * window_m;
  for (std::size_t i = 0; i < query_s.size(); ++i) {
    const double lo = std::max(query_s[i] - half, hp.s_m.front());
    const double hi = std::min(query_s[i] + half, hp.s_m.back());
    const double span = hi - lo;
    if (span < 1e-3)
      continue;
    kappa[i] = (thetaAt(hp, hi) - thetaAt(hp, lo)) / span;
  }
  return kappa;
}

namespace {

TurnSection sectionFrom(const std::vector<double>& s, const std::vector<double>& kappa, std::size_t from,
                        std::size_t to, const RouteConfig& cfg)
{
  TurnSection sec;
  sec.start_m = s[from];
  sec.end_m = s[to];
  std::size_t peak = from;
  for (std::size_t i = from; i <= to; ++i) {
    if (std::abs(kappa[i]) > std::abs(kappa[peak]))
      peak = i;
  }
  sec.kappa = std::abs(kappa[peak]);
  sec.sign = kappa[peak] > 0 ? 1 : -1;
  sec.speed_mps = sec.kappa > 1e-9 ? std::sqrt(cfg.max_lat_acc / sec.kappa) : 0.0;
  return sec;
}

/** dragonpilot's `split_speed_section_by_curv_degree`: isolate a sharp peak inside a long section so one
 *  tight corner does not impose its speed on half a kilometre of gentle curve. */
void splitByDegree(const std::vector<double>& s, const std::vector<double>& kappa, std::size_t from, std::size_t to,
                   const RouteConfig& cfg, std::vector<TurnSection>& out)
{
  const double length = s[to] - s[from];
  if (length <= cfg.min_section_m) {
    out.push_back(sectionFrom(s, kappa, from, to, cfg));
    return;
  }

  std::size_t peak = from;
  double sum = 0.0;
  for (std::size_t i = from; i <= to; ++i) {
    sum += std::abs(kappa[i]);
    if (std::abs(kappa[i]) > std::abs(kappa[peak]))
      peak = i;
  }
  const double max_curv = std::abs(kappa[peak]);
  const double mean_curv = sum / static_cast<double>(to - from + 1);
  if (mean_curv <= 1e-12 || max_curv / mean_curv <= cfg.max_curv_deviation) {
    out.push_back(sectionFrom(s, kappa, from, to, cfg));
    return;
  }

  // Half-arc of `max_curv_split_arc_deg` around the peak, in samples.
  const double arc_side_m = (cfg.max_curv_split_arc_deg * kDeg / std::max(max_curv, 1e-9)) / 2.0;
  const auto arc_side = static_cast<std::size_t>(std::ceil(arc_side_m / std::max(cfg.step_m, 1e-6)));

  const bool cut_lo = peak > from + arc_side;
  const bool cut_hi = peak + arc_side < to;
  if (!cut_lo && !cut_hi) {
    out.push_back(sectionFrom(s, kappa, from, to, cfg));
    return;
  }

  const std::size_t lo = cut_lo ? peak - arc_side : from;
  const std::size_t hi = cut_hi ? peak + arc_side : to;
  if (cut_lo)
    splitByDegree(s, kappa, from, lo, cfg, out);
  splitByDegree(s, kappa, lo, hi, cfg, out);
  if (cut_hi)
    splitByDegree(s, kappa, hi, to, cfg, out);
}

}  // namespace

std::vector<TurnSection> turnSections(const std::vector<double>& s, const std::vector<double>& kappa,
                                      const RouteConfig& cfg)
{
  std::vector<TurnSection> out;
  const std::size_t n = std::min(s.size(), kappa.size());
  if (n < 2)
    return out;

  std::size_t i = 0;
  while (i < n) {
    if (std::abs(kappa[i]) < cfg.turn_kappa) {
      ++i;
      continue;
    }
    // Run of over-threshold samples, cut at every sign change (dragonpilot splits an S-bend in two).
    const std::size_t from = i;
    const int sign = kappa[i] > 0 ? 1 : -1;
    std::size_t j = i;
    while (j + 1 < n && std::abs(kappa[j + 1]) >= cfg.turn_kappa && (kappa[j + 1] > 0 ? 1 : -1) == sign)
      ++j;
    if (j > from)
      splitByDegree(s, kappa, from, j, cfg, out);
    i = j + 1;
  }
  return out;
}

std::uint32_t matchDirectedEdge(const RoadMap& map, double x, double y, double yaw_rad, const RouteConfig& cfg,
                                double* snap_x, double* snap_y, double* dist_m, double* heading_delta_rad,
                                double* s_into_edge_m)
{
  std::uint32_t best_de = kNoEdge;
  double best_score = std::numeric_limits<double>::max();
  double best_x = 0.0, best_y = 0.0, best_dist = -1.0, best_delta = 0.0, best_s = 0.0;

  const double max_delta = cfg.max_match_heading_deg * kDeg;
  std::vector<double> xs, ys;

  for (const std::uint32_t de : candidatesNear(map, x, y, cfg.match_search_m)) {
    dirPolyline(map, de, xs, ys);
    if (xs.size() < 2)
      continue;

    // Closest segment of this edge, and the arc length up to the projection.
    double run = 0.0, at_s = 0.0, qx = 0.0, qy = 0.0, seg_heading = 0.0;
    double d2 = std::numeric_limits<double>::max();
    for (std::size_t k = 1; k < xs.size(); ++k) {
      double cx = 0.0, cy = 0.0, t = 0.0;
      const double seg_len = std::hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1]);
      const double c2 = distSqToSegment(x, y, xs[k - 1], ys[k - 1], xs[k], ys[k], cx, cy, t);
      if (c2 < d2) {
        d2 = c2;
        qx = cx;
        qy = cy;
        at_s = run + t * seg_len;
        seg_heading = std::atan2(ys[k] - ys[k - 1], xs[k] - xs[k - 1]);
      }
      run += seg_len;
    }

    const double dist = std::sqrt(d2);
    if (dist > cfg.max_match_dist_m)
      continue;
    const double delta = wrapPi(seg_heading - yaw_rad);
    if (std::abs(delta) > max_delta)
      continue;

    const double score = dist + cfg.heading_weight_m_per_rad * std::abs(delta);
    if (score < best_score) {
      best_score = score;
      best_de = de;
      best_x = qx;
      best_y = qy;
      best_dist = dist;
      best_delta = delta;
      best_s = at_s;
    }
  }

  if (best_de == kNoEdge)
    return kNoEdge;
  if (snap_x)
    *snap_x = best_x;
  if (snap_y)
    *snap_y = best_y;
  if (dist_m)
    *dist_m = best_dist;
  if (heading_delta_rad)
    *heading_delta_rad = best_delta;
  if (s_into_edge_m)
    *s_into_edge_m = best_s;
  return best_de;
}

RouteAhead buildRouteAhead(const RoadMap& map, double x, double y, double yaw_rad, const RouteConfig& cfg)
{
  RouteAhead out;
  if (map.empty())
    return out;

  double snap_x = 0.0, snap_y = 0.0, s_into = 0.0;
  const std::uint32_t start_de =
      matchDirectedEdge(map, x, y, yaw_rad, cfg, &snap_x, &snap_y, &out.match_dist_m, &out.heading_delta_rad, &s_into);
  if (start_de == kNoEdge)
    return out;

  out.matched = true;
  out.dir_edge = start_de;
  out.x_m = snap_x;
  out.y_m = snap_y;
  out.road_name = map.edgeName(edgeOf(start_de));

  // Geometry of the current edge from the snapped point onwards.
  std::vector<double> xs, ys, px, py;
  dirPolyline(map, start_de, xs, ys);
  {
    double run = 0.0;
    px.push_back(snap_x);
    py.push_back(snap_y);
    for (std::size_t k = 1; k < xs.size(); ++k) {
      run += std::hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1]);
      if (run > s_into) {
        px.push_back(xs[k]);
        py.push_back(ys[k]);
      }
    }
  }
  out.dir_edges.push_back(start_de);

  double have_m = 0.0;
  for (std::size_t k = 1; k < px.size(); ++k)
    have_m += std::hypot(px[k] - px[k - 1], py[k] - py[k - 1]);

  // Grow forward: straightest continuation, with the same road name winning ties. Without the name rule the
  // route takes an exit ramp whenever the ramp happens to leave more straight-on than the road does — the
  // single most common way this kind of walk goes wrong.
  std::unordered_set<std::uint32_t> used{edgeOf(start_de)};
  std::uint32_t cur = start_de;
  std::string cur_name = out.road_name;

  while (have_m < cfg.horizon_m) {
    const auto cands = outgoing(map, endNode(map, cur));

    std::uint32_t best = kNoEdge;
    double best_turn = std::numeric_limits<double>::max();
    std::uint32_t named = kNoEdge;
    double named_turn = std::numeric_limits<double>::max();

    for (const std::uint32_t nde : cands) {
      if (used.count(edgeOf(nde)) != 0)
        continue;
      const double turn = std::abs(turnBetweenRad(map, cur, nde));
      if (turn < best_turn) {
        best_turn = turn;
        best = nde;
      }
      if (!cur_name.empty() && turn <= cfg.straight_max_deg * kDeg && map.edgeName(edgeOf(nde)) == cur_name &&
          turn < named_turn) {
        named_turn = turn;
        named = nde;
      }
    }

    const std::uint32_t next = named != kNoEdge ? named : best;
    if (next == kNoEdge)
      break;

    dirPolyline(map, next, xs, ys);
    if (xs.size() < 2)
      break;
    for (std::size_t k = 1; k < xs.size(); ++k) {  // k=1: first point repeats the previous edge's last
      px.push_back(xs[k]);
      py.push_back(ys[k]);
      have_m += std::hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1]);
    }

    used.insert(edgeOf(next));
    out.dir_edges.push_back(next);
    cur = next;
    const std::string next_name = map.edgeName(edgeOf(next));
    if (!next_name.empty())
      cur_name = next_name;
  }

  resamplePolyline(px, py, cfg.step_m, out.s_m, out.x_m_pts, out.y_m_pts);
  if (out.s_m.size() < 3)
    return out;

  // Trim to the horizon: the last edge usually overshoots it.
  if (out.s_m.back() > cfg.horizon_m) {
    const auto keep = static_cast<std::size_t>(cfg.horizon_m / cfg.step_m) + 1;
    if (keep >= 3 && keep < out.s_m.size()) {
      out.s_m.resize(keep);
      out.x_m_pts.resize(keep);
      out.y_m_pts.resize(keep);
    }
  }

  out.length_m = out.s_m.back();
  // Curvature comes from the map geometry (`px`/`py`), sampled onto the resampled grid — not from the
  // resampled points themselves, which no longer carry where the map's nodes were.
  out.kappa = curvatureAlong(px, py, out.s_m, cfg.window_m);
  out.turns = turnSections(out.s_m, out.kappa, cfg);

  std::vector<double> gaps;
  gaps.reserve(px.size());
  for (std::size_t k = 1; k < px.size(); ++k) {
    const double g = std::hypot(px[k] - px[k - 1], py[k] - py[k - 1]);
    if (g > 1e-6)
      gaps.push_back(g);
  }
  if (!gaps.empty()) {
    std::nth_element(gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(gaps.size() / 2), gaps.end());
    out.node_spacing_m = gaps[gaps.size() / 2];
  }
  return out;
}

}  // namespace mapmatch
}  // namespace adas
