#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "adas/mapmatch/road_map.h"

namespace adas {
namespace mapmatch {
inline constexpr std::uint32_t kNoEdge = 0xFFFFFFFFu;

inline std::uint32_t edgeOf(std::uint32_t de) { return de >> 1; }
inline bool reversed(std::uint32_t de) { return (de & 1u) != 0u; }
inline std::uint32_t makeDir(std::uint32_t edge, bool rev) { return (edge << 1) | (rev ? 1u : 0u); }

inline double wrapPi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

/** Heading, rad, entering the directed edge (i.e. at its first point). */
inline double headingIn(const RoadMap& map, std::uint32_t de)
{
  return reversed(de) ? map.headingAtEnd(edgeOf(de)) + M_PI : map.headingAtStart(edgeOf(de));
}

/** Heading, rad, leaving the directed edge (i.e. at its last point). */
inline double headingOut(const RoadMap& map, std::uint32_t de)
{
  return reversed(de) ? map.headingAtStart(edgeOf(de)) + M_PI : map.headingAtEnd(edgeOf(de));
}

inline std::uint32_t startNode(const RoadMap& map, std::uint32_t de)
{
  const RoadEdge& e = map.edge(edgeOf(de));
  return reversed(de) ? e.node_b : e.node_a;
}

inline std::uint32_t endNode(const RoadMap& map, std::uint32_t de)
{
  const RoadEdge& e = map.edge(edgeOf(de));
  return reversed(de) ? e.node_a : e.node_b;
}

/** Turn angle between two directed edges, radians, positive left. */
inline double turnBetweenRad(const RoadMap& map, std::uint32_t from_de, std::uint32_t to_de)
{
  return wrapPi(headingIn(map, to_de) - headingOut(map, from_de));
}

inline double turnBetweenDeg(const RoadMap& map, std::uint32_t from_de, std::uint32_t to_de)
{
  return turnBetweenRad(map, from_de, to_de) * (180.0 / M_PI);
}

/** Directed edges leaving `node`, one-way restrictions applied: an edge can only be entered at its
 *  `node_a` if it is one-way. */
inline std::vector<std::uint32_t> outgoing(const RoadMap& map, std::uint32_t node)
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

/** Edge geometry in travel order. */
inline void dirPolyline(const RoadMap& map, std::uint32_t de, std::vector<double>& xs, std::vector<double>& ys)
{
  map.edgePolyline(edgeOf(de), xs, ys);
  if (reversed(de)) {
    std::reverse(xs.begin(), xs.end());
    std::reverse(ys.begin(), ys.end());
  }
}

}  // namespace mapmatch
}  // namespace adas
