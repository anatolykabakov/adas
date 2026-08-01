#include "mapmatch/search.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "utils/logger.h"

namespace adas {
namespace mapmatch {
namespace {

constexpr double kDeg = M_PI / 180.0;

inline std::uint32_t edgeOf(std::uint32_t de) { return de >> 1; }
inline bool reversed(std::uint32_t de) { return (de & 1u) != 0u; }
inline std::uint32_t makeDir(std::uint32_t edge, bool rev) { return (edge << 1) | (rev ? 1u : 0u); }

double wrapPi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

double headingIn(const RoadMap& map, std::uint32_t de)
{
  return reversed(de) ? map.headingAtEnd(edgeOf(de)) + M_PI : map.headingAtStart(edgeOf(de));
}

double headingOut(const RoadMap& map, std::uint32_t de)
{
  return reversed(de) ? map.headingAtStart(edgeOf(de)) + M_PI : map.headingAtEnd(edgeOf(de));
}

std::uint32_t startNode(const RoadMap& map, std::uint32_t de)
{
  const RoadEdge& e = map.edge(edgeOf(de));
  return reversed(de) ? e.node_b : e.node_a;
}

std::uint32_t endNode(const RoadMap& map, std::uint32_t de)
{
  const RoadEdge& e = map.edge(edgeOf(de));
  return reversed(de) ? e.node_a : e.node_b;
}

double turnBetween(const RoadMap& map, std::uint32_t from_de, std::uint32_t to_de)
{
  return wrapPi(headingIn(map, to_de) - headingOut(map, from_de)) / kDeg;
}

std::vector<std::uint32_t> outgoing(const RoadMap& map, std::uint32_t node)
{
  std::vector<std::uint32_t> out;
  std::size_t count = 0;
  const std::uint32_t* ids = map.outEdges(node, &count);
  out.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint32_t ei = ids[i];
    const RoadEdge& e = map.edge(ei);
    if (e.node_a == node)
      out.push_back(makeDir(ei, false));
    else if (e.node_b == node && !e.oneway)
      out.push_back(makeDir(ei, true));
  }
  return out;
}

void dirPolyline(const RoadMap& map, std::uint32_t de, std::vector<double>& xs, std::vector<double>& ys)
{
  map.edgePolyline(edgeOf(de), xs, ys);
  if (reversed(de)) {
    std::reverse(xs.begin(), xs.end());
    std::reverse(ys.begin(), ys.end());
  }
}

double trackHeadingAt(const Track& track, double s)
{
  if (track.size() < 2)
    return 0.0;
  if (s <= track.s_m.front())
    return track.theta_rad.front();
  if (s >= track.s_m.back())
    return track.theta_rad.back();
  const double ds = std::max(track.s_m[1] - track.s_m[0], 1e-6);
  const std::size_t i = std::min(track.size() - 2, static_cast<std::size_t>(s / ds));
  const double w = (s - track.s_m[i]) / ds;
  return track.theta_rad[i] + w * (track.theta_rad[i + 1] - track.theta_rad[i]);
}

struct State {
  std::uint32_t de = 0;
  double s_route_m = 0.0;
  double theta0_rad = 0.0;
  double s_track0_m = 0.0;
  double cost = 0.0;
  int parent = -1;
};

}  // namespace

std::vector<RouteCandidate> searchRoutes(const RoadMap& map, const Track& track, const SearchConfig& cfg)
{
  std::vector<RouteCandidate> out;
  if (map.empty() || track.size() < 4)
    return out;

  const Maneuver* first_turn = nullptr;
  for (const auto& mv : track.maneuvers) {
    if (mv.isTurn()) {
      first_turn = &mv;
      break;
    }
  }
  if (first_turn == nullptr)
    return out;

  const double sigma = std::max(1.0, cfg.heading_sigma_deg);
  const double step = std::max(5.0, cfg.heading_step_m);
  const double track_len = track.lengthM();
  const double s_ref = first_turn->s_start_m;
  const double turn_deg = first_turn->angle_deg;

  std::vector<std::vector<std::uint32_t>> incoming(map.nodeCount());
  for (std::uint32_t ei = 0; ei < map.edgeCount(); ++ei) {
    const RoadEdge& e = map.edge(ei);
    incoming[e.node_b].push_back(makeDir(ei, false));
    if (!e.oneway)
      incoming[e.node_a].push_back(makeDir(ei, true));
  }

  std::vector<State> pool;
  pool.reserve(1 << 18);
  std::vector<int> beam;

  for (std::uint32_t node = 0; node < map.nodeCount(); ++node) {
    if (incoming[node].empty())
      continue;
    const auto outs = outgoing(map, node);
    for (const std::uint32_t in_de : incoming[node]) {
      for (const std::uint32_t out_de : outs) {
        if (edgeOf(out_de) == edgeOf(in_de))
          continue;
        const double angle = turnBetween(map, in_de, out_de);
        if (angle * turn_deg <= 0.0)
          continue;
        if (std::abs(angle) < cfg.turn_start_min_deg)
          continue;
        if (std::abs(angle) > std::abs(turn_deg) + cfg.turn_tol_deg + 30.0)
          continue;

        const double approach_len = map.edge(edgeOf(in_de)).length_m;
        State s;
        s.de = in_de;
        s.s_route_m = approach_len;
        s.theta0_rad = headingOut(map, in_de) - trackHeadingAt(track, s_ref);
        s.s_track0_m = s_ref - approach_len;
        s.parent = -1;
        pool.push_back(s);
        beam.push_back(static_cast<int>(pool.size()) - 1);
      }
    }
  }

  if (cfg.verbose)
    LOGI("searchRoutes: seeds %zu (first turn %.0f deg at s=%.0f m of %.0f)", beam.size(), turn_deg, s_ref, track_len);
  if (beam.empty())
    return out;

  std::unordered_map<std::uint64_t, double> best;
  auto key_of = [](const State& s) {
    const std::uint64_t sb = static_cast<std::uint64_t>(std::max(0.0, s.s_route_m) / 50.0) & 0xFFF;
    const std::uint64_t tb = static_cast<std::uint64_t>((s.theta0_rad / kDeg + 720.0) / 15.0) & 0xFF;
    return (static_cast<std::uint64_t>(s.de) << 20) | (tb << 12) | sb;
  };

  auto norm_cost = [&](const State& s) { return s.cost / std::max(100.0, s.s_route_m); };

  std::vector<double> xs, ys;
  std::vector<int> finished;
  const int max_rounds = 800;

  for (int round = 0; round < max_rounds && !beam.empty(); ++round) {
    std::vector<int> next;
    next.reserve(beam.size() * 2);

    for (const int si : beam) {
      const State s = pool[si];
      if (s.s_track0_m + s.s_route_m >= track_len) {
        finished.push_back(si);
        continue;
      }
      if (s.s_route_m > track_len * cfg.max_overshoot_rel + 500.0)
        continue;

      for (const std::uint32_t nde : outgoing(map, endNode(map, s.de))) {
        if (edgeOf(nde) == edgeOf(s.de))
          continue;

        dirPolyline(map, nde, xs, ys);
        if (xs.size() < 2)
          continue;

        double seg_cost = 0.0;
        double seg_len = 0.0;
        double sampled = step;
        int n_samples = 0;
        for (std::size_t k = 1; k < xs.size(); ++k) {
          const double dx = xs[k] - xs[k - 1];
          const double dy = ys[k] - ys[k - 1];
          const double len = std::hypot(dx, dy);
          if (len < 1e-6)
            continue;
          seg_len += len;
          sampled += len;
          if (sampled >= step) {
            const double s_track = s.s_track0_m + s.s_route_m + seg_len;
            if (s_track >= 0.0 && s_track <= track_len) {
              const double diff = wrapPi(std::atan2(dy, dx) - s.theta0_rad - trackHeadingAt(track, s_track)) / kDeg;
              seg_cost += (diff / sigma) * (diff / sigma) * len;
              ++n_samples;
            }
            sampled = 0.0;
          }
        }
        if (n_samples == 0)
          continue;

        State t;
        t.de = nde;
        t.s_route_m = s.s_route_m + seg_len;
        t.theta0_rad = s.theta0_rad;
        t.s_track0_m = s.s_track0_m;
        t.cost = s.cost + seg_cost;
        t.parent = si;

        const std::uint64_t k = key_of(t);
        auto it = best.find(k);
        if (it == best.end() || t.cost < it->second) {
          best[k] = t.cost;
          pool.push_back(t);
          next.push_back(static_cast<int>(pool.size()) - 1);
        }
      }
    }

    std::sort(next.begin(), next.end(), [&](int a, int b) { return norm_cost(pool[a]) < norm_cost(pool[b]); });
    const int width = std::max(cfg.beam_width, 3000);
    if (static_cast<int>(next.size()) > width)
      next.resize(width);
    beam.swap(next);

    if (cfg.verbose && (round < 4 || round % 25 == 0))
      LOGI("  round %d: beam %zu, finished %zu", round, beam.size(), finished.size());
  }

  if (finished.empty())
    finished = beam;
  if (cfg.verbose)
    LOGI("searchRoutes: routes to track end %zu", finished.size());

  std::sort(finished.begin(), finished.end(), [&](int a, int b) { return norm_cost(pool[a]) < norm_cost(pool[b]); });

  auto extend_back = [&](std::vector<std::uint32_t>& route, double need_m) {
    std::unordered_set<std::uint32_t> seen(route.begin(), route.end());
    double have = 0.0;
    while (have < need_m) {
      const std::uint32_t cur = route.front();
      const std::uint32_t node = startNode(map, cur);
      std::uint32_t best_de = 0xFFFFFFFFu;
      double best_turn = cfg.straight_max_deg;
      for (const std::uint32_t pde : incoming[node]) {
        if (edgeOf(pde) == edgeOf(cur) || seen.count(pde))
          continue;
        const double turn = std::abs(turnBetween(map, pde, cur));
        if (turn <= best_turn) {
          best_turn = turn;
          best_de = pde;
        }
      }
      if (best_de == 0xFFFFFFFFu)
        break;
      seen.insert(best_de);
      route.insert(route.begin(), best_de);
      have += map.edge(edgeOf(best_de)).length_m;
    }
    return have;
  };

  std::unordered_set<std::uint32_t> seen_start;
  for (const int si : finished) {
    if (static_cast<int>(out.size()) >= cfg.max_candidates)
      break;

    std::vector<std::uint32_t> route;
    for (int i = si; i >= 0; i = pool[i].parent)
      route.push_back(pool[i].de);
    std::reverse(route.begin(), route.end());
    if (route.size() < 2)
      continue;
    if (!seen_start.insert(route.front()).second)
      continue;

    const double need_back = std::max(0.0, pool[si].s_track0_m);
    const double got_back = extend_back(route, need_back);

    RouteCandidate c;
    c.dir_edges = route;
    c.cost = norm_cost(pool[si]);
    if (need_back > 1.0)
      c.cost += std::max(0.0, need_back - got_back) / std::max(need_back, 1.0);
    c.matched_turns = 0;
    for (const std::uint32_t de : route)
      c.length_m += map.edge(edgeOf(de)).length_m;

    dirPolyline(map, route.front(), xs, ys);
    c.start_x_m = xs.front();
    c.start_y_m = ys.front();
    c.start_heading_rad = headingIn(map, route.front());
    out.push_back(std::move(c));
  }
  return out;
}

}  // namespace mapmatch
}  // namespace adas
