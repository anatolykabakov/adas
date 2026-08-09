#include "mapmatch/window_search.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace adas {
namespace mapmatch {
namespace {

constexpr double kRad2Deg = 57.29577951308232;

double wrapDeg(double a)
{
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

/** Курс в начале и в конце направленного ребра плюс изгиб внутри него. */
struct DirGeometry {
  std::vector<double> start_deg, end_deg, bend_deg, length_m;

  explicit DirGeometry(const RoadMap& map)
  {
    const std::size_t n = map.edgeCount();
    start_deg.assign(2 * n, 0.0);
    end_deg.assign(2 * n, 0.0);
    bend_deg.assign(2 * n, 0.0);
    length_m.assign(2 * n, 0.0);

    std::vector<double> xs, ys;
    for (std::size_t e = 0; e < n; ++e) {
      map.edgePolyline(static_cast<std::uint32_t>(e), xs, ys);
      if (xs.size() < 2) continue;
      const double h0 = std::atan2(ys[1] - ys[0], xs[1] - xs[0]) * kRad2Deg;
      const double h1 =
          std::atan2(ys.back() - ys[ys.size() - 2], xs.back() - xs[xs.size() - 2]) * kRad2Deg;

      // изгиб внутри ребра: без него кольцевая дорога проходит как прямая — на стыках рёбер
      // поворотов нет, а кривизна спрятана в полилинии
      double bend = 0.0;
      double prev = std::atan2(ys[1] - ys[0], xs[1] - xs[0]) * kRad2Deg;
      for (std::size_t i = 2; i < xs.size(); ++i) {
        const double cur = std::atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]) * kRad2Deg;
        bend += wrapDeg(cur - prev);
        prev = cur;
      }

      const double len = map.edge(static_cast<std::uint32_t>(e)).length_m;
      start_deg[2 * e] = h0;
      end_deg[2 * e] = h1;
      bend_deg[2 * e] = bend;
      length_m[2 * e] = len;

      start_deg[2 * e + 1] = wrapDeg(h1 + 180.0);
      end_deg[2 * e + 1] = wrapDeg(h0 + 180.0);
      bend_deg[2 * e + 1] = -bend;
      length_m[2 * e + 1] = len;
    }
  }

  double turn(std::uint32_t de_in, std::uint32_t de_out) const
  {
    return wrapDeg(start_deg[de_out] - end_deg[de_in]);
  }
};

struct State {
  double cost = 0.0;
  int node = -1;        ///< индекс в дереве путей
  double dist = 0.0;    ///< пройдено по маршруту
  double head = 0.0;    ///< курс маршрута
  double head0 = 0.0;   ///< курс на начало текущего окна
};

std::uint32_t headNode(const RoadMap& map, std::uint32_t de)
{
  const RoadEdge& e = map.edge(de >> 1);
  return (de & 1u) ? e.node_a : e.node_b;
}

}  // namespace

std::vector<WindowRoute> searchByWindows(const RoadMap& map, const std::vector<double>& window_deg,
                                         const WindowSearchConfig& cfg)
{
  std::vector<WindowRoute> result;
  if (map.empty() || window_deg.empty()) return result;

  const DirGeometry geo(map);
  const std::size_t n_dir = 2 * map.edgeCount();

  std::vector<int> parent;
  std::vector<std::uint32_t> edge_of;
  parent.reserve(1u << 20);
  edge_of.reserve(1u << 20);
  auto add_node = [&](std::uint32_t de, int par) {
    parent.push_back(par);
    edge_of.push_back(de);
    return static_cast<int>(parent.size()) - 1;
  };

  std::vector<State> layer;
  layer.reserve(n_dir);
  for (std::uint32_t de = 0; de < n_dir; ++de) {
    if (geo.length_m[de] <= 0.0) continue;
    // Односторонняя дорога только в свою сторону — так же, как при расширении луча ниже. Без этой
    // проверки маршрут мог начаться с движения против односторонней: и ширина луча тратится впустую,
    // и наверх может выйти вариант, по которому проехать нельзя.
    if ((de & 1u) && map.edge(de >> 1).oneway) continue;
    State s;
    s.node = add_node(de, -1);
    s.dist = geo.length_m[de];
    s.head = geo.bend_deg[de];
    layer.push_back(s);
  }

  double information = 0.0;  // сколько градусов «смысла» уже прошли
  std::vector<State> next;
  std::vector<State> grown;

  for (std::size_t k = 0; k < window_deg.size(); ++k) {
    const double border = static_cast<double>(k + 1) * cfg.window_m;
    const double want = window_deg[k];
    information += std::fabs(want);

    next.clear();
    for (const State& st : layer) {
      // дорастить состояние, пока оно не закроет окно
      grown.clear();
      grown.push_back(st);
      for (int step = 0; step < cfg.max_expand && !grown.empty(); ++step) {
        std::vector<State> tmp;
        tmp.reserve(grown.size() * 2);
        for (const State& g : grown) {
          if (g.dist >= border) {
            const double diff = std::fabs((g.head - g.head0) - want);
            State done = g;
            done.cost += diff > cfg.tol_deg ? std::min(diff, cfg.clip_deg) : 0.0;
            done.head0 = g.head;
            next.push_back(done);
            continue;
          }
          const std::uint32_t de = edge_of[g.node];
          std::size_t count = 0;
          const std::uint32_t* outs = map.outEdges(headNode(map, de), &count);
          for (std::size_t i = 0; i < count; ++i) {
            const std::uint32_t eid = outs[i];
            if (eid == (de >> 1)) continue;
            const RoadEdge& e = map.edge(eid);
            // направление выбирается так, чтобы въехать в ребро с того узла, где мы стоим
            const std::uint32_t node = headNode(map, de);
            std::uint32_t de2;
            if (e.node_a == node) {
              de2 = 2 * eid;
            } else if (!e.oneway && e.node_b == node) {
              de2 = 2 * eid + 1;
            } else {
              continue;
            }
            State s2;
            s2.cost = g.cost;
            s2.node = add_node(de2, g.node);
            s2.dist = g.dist + geo.length_m[de2];
            s2.head = g.head + geo.turn(de, de2) + geo.bend_deg[de2];
            s2.head0 = g.head0;
            tmp.push_back(s2);
          }
        }
        grown.swap(tmp);
        if (grown.size() > static_cast<std::size_t>(cfg.beam)) {
          grown.resize(static_cast<std::size_t>(cfg.beam));
        }
      }
    }

    if (next.empty()) break;

    // Отбор по стоимости откладывается: на первых окнах приращения близки к нулю, стоимости
    // почти равны, и резать луч — значит выбрасывать правильный старт случайно. Но совсем без
    // ограничения слой растёт лавиной, поэтому дубли по ребру снимаются всегда.
    std::sort(next.begin(), next.end(),
              [](const State& a, const State& b) { return a.cost < b.cost; });
    const int limit = information >= cfg.defer_deg ? cfg.beam : cfg.defer_beam;
    std::unordered_map<std::uint32_t, int> per_edge;
    std::unordered_map<std::int64_t, int> per_cell;
    per_edge.reserve(next.size());
    per_cell.reserve(next.size());
    layer.clear();
    layer.reserve(std::min<std::size_t>(next.size(), static_cast<std::size_t>(limit)));
    for (const State& s : next) {
      const std::uint32_t de = edge_of[s.node];
      int& used = per_edge[de];
      if (used >= cfg.per_edge) continue;
      // Разнообразие по местности. Ключа по ребру мало: соседние рёбра одной улицы — это одно
      // и то же место, и луч целиком оседает в одном районе, выдавая двадцать вариантов одного
      // неверного ответа с одинаковой стоимостью.
      const std::uint32_t node = headNode(map, de);
      const std::int64_t cx = static_cast<std::int64_t>(std::floor(map.nodeX(node) / cfg.cell_m));
      const std::int64_t cy = static_cast<std::int64_t>(std::floor(map.nodeY(node) / cfg.cell_m));
      int& in_cell = per_cell[(cx << 24) ^ cy];
      if (in_cell >= cfg.per_cell) continue;
      ++used;
      ++in_cell;
      layer.push_back(s);
      if (static_cast<int>(layer.size()) >= limit) break;
    }

    if (cfg.verbose) {
      std::printf("окно %2zu: цель %+6.1f°, состояний %zu, лучшая стоимость %.1f\n", k, want,
                  layer.size(), layer.empty() ? 0.0 : layer.front().cost);
    }
  }

  std::sort(layer.begin(), layer.end(),
            [](const State& a, const State& b) { return a.cost < b.cost; });
  const std::size_t take = std::min<std::size_t>(layer.size(), 20);
  for (std::size_t i = 0; i < take; ++i) {
    WindowRoute route;
    route.cost = layer[i].cost;
    for (int cur = layer[i].node; cur >= 0; cur = parent[cur]) {
      route.dir_edges.push_back(edge_of[cur]);
    }
    std::reverse(route.dir_edges.begin(), route.dir_edges.end());
    result.push_back(std::move(route));
  }
  return result;
}

}  // namespace mapmatch
}  // namespace adas
