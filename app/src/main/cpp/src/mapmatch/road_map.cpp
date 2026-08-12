#include "adas/mapmatch/road_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

#include "adas/utils/logger.h"

namespace adas {
namespace mapmatch {
namespace {
constexpr char kMagic[8] = {'A', 'D', 'A', 'S', 'M', 'A', 'P', '1'};

template <typename T>
T take(const std::vector<char>& buf, std::size_t& off)
{
  T v{};
  std::memcpy(&v, buf.data() + off, sizeof(T));
  off += sizeof(T);
  return v;
}

double distSqToSegment(double px, double py, double ax, double ay, double bx, double by, double* qx = nullptr,
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

}  // namespace

bool RoadMap::load(const std::string& path)
{
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) {
    LOGE("RoadMap: failed to open %s", path.c_str());
    return false;
  }
  const std::streamsize size = in.tellg();
  in.seekg(0);
  std::vector<char> buf(static_cast<std::size_t>(size));
  if (!in.read(buf.data(), size)) {
    LOGE("RoadMap: failed to read %s", path.c_str());
    return false;
  }
  if (buf.size() < 64 || std::memcmp(buf.data(), kMagic, sizeof(kMagic)) != 0) {
    LOGE("RoadMap: %s — not ADASMAP1 format", path.c_str());
    return false;
  }

  std::size_t off = sizeof(kMagic);
  frame_.lat0_deg = take<double>(buf, off);
  frame_.lon0_deg = take<double>(buf, off);
  const auto n_nodes = take<std::uint32_t>(buf, off);
  const auto n_edges = take<std::uint32_t>(buf, off);
  const auto n_points = take<std::uint32_t>(buf, off);
  const auto n_names = take<std::uint32_t>(buf, off);
  grid_x0_ = take<double>(buf, off);
  grid_y0_ = take<double>(buf, off);
  grid_nx_ = take<std::uint32_t>(buf, off);
  grid_ny_ = take<std::uint32_t>(buf, off);
  grid_cell_m_ = take<double>(buf, off);

  node_x_.resize(n_nodes);
  node_y_.resize(n_nodes);
  for (std::uint32_t i = 0; i < n_nodes; ++i) {
    node_x_[i] = take<std::int32_t>(buf, off) / 100.0;
    node_y_[i] = take<std::int32_t>(buf, off) / 100.0;
  }

  edges_.resize(n_edges);
  for (std::uint32_t i = 0; i < n_edges; ++i) {
    RoadEdge& e = edges_[i];
    e.node_a = take<std::uint32_t>(buf, off);
    e.node_b = take<std::uint32_t>(buf, off);
    e.point_first = take<std::uint32_t>(buf, off);
    e.point_count = take<std::uint16_t>(buf, off);
    e.name_id = take<std::uint32_t>(buf, off);
    e.length_m = take<float>(buf, off);
    e.oneway = take<std::uint8_t>(buf, off) != 0;
  }

  point_x_.resize(n_points);
  point_y_.resize(n_points);
  for (std::uint32_t i = 0; i < n_points; ++i) {
    point_x_[i] = take<std::int32_t>(buf, off) / 100.0;
    point_y_[i] = take<std::int32_t>(buf, off) / 100.0;
  }

  node_edge_off_.resize(n_nodes + 1);
  for (auto& v : node_edge_off_)
    v = take<std::uint32_t>(buf, off);
  node_edge_val_.resize(node_edge_off_.back());
  for (auto& v : node_edge_val_)
    v = take<std::uint32_t>(buf, off);

  const std::size_t cells = static_cast<std::size_t>(grid_nx_) * grid_ny_;
  grid_off_.resize(cells + 1);
  for (auto& v : grid_off_)
    v = take<std::uint32_t>(buf, off);
  grid_val_.resize(grid_off_.back());
  for (auto& v : grid_val_)
    v = take<std::uint32_t>(buf, off);

  const auto blob_len = take<std::uint32_t>(buf, off);
  (void)blob_len;
  names_.reserve(n_names);
  for (std::uint32_t i = 0; i < n_names; ++i) {
    const auto len = take<std::uint16_t>(buf, off);
    names_.emplace_back(buf.data() + off, len);
    off += len;
  }

  LOGI("RoadMap: %s — nodes %u, edges %u, points %u, names %u", path.c_str(), n_nodes, n_edges, n_points, n_names);
  return true;
}

std::string RoadMap::edgeName(std::uint32_t i) const
{
  if (i >= edges_.size())
    return {};
  const std::uint32_t id = edges_[i].name_id;
  return id < names_.size() ? names_[id] : std::string{};
}

void RoadMap::edgePolyline(std::uint32_t i, std::vector<double>& xs, std::vector<double>& ys) const
{
  xs.clear();
  ys.clear();
  if (i >= edges_.size())
    return;
  const RoadEdge& e = edges_[i];
  xs.reserve(e.point_count);
  ys.reserve(e.point_count);
  for (std::uint32_t k = 0; k < e.point_count; ++k) {
    xs.push_back(point_x_[e.point_first + k]);
    ys.push_back(point_y_[e.point_first + k]);
  }
}

std::vector<std::uint32_t> RoadMap::edgesInBBox(double x0, double y0, double x1, double y1) const
{
  std::vector<std::uint32_t> out;
  if (grid_nx_ == 0 || grid_ny_ == 0)
    return out;
  const auto cx0 = static_cast<long>(std::floor((std::min(x0, x1) - grid_x0_) / grid_cell_m_));
  const auto cx1 = static_cast<long>(std::floor((std::max(x0, x1) - grid_x0_) / grid_cell_m_));
  const auto cy0 = static_cast<long>(std::floor((std::min(y0, y1) - grid_y0_) / grid_cell_m_));
  const auto cy1 = static_cast<long>(std::floor((std::max(y0, y1) - grid_y0_) / grid_cell_m_));

  std::vector<char> seen(edges_.size(), 0);
  for (long cy = std::max<long>(0, cy0); cy <= std::min<long>(grid_ny_ - 1, cy1); ++cy) {
    for (long cx = std::max<long>(0, cx0); cx <= std::min<long>(grid_nx_ - 1, cx1); ++cx) {
      const std::size_t c = static_cast<std::size_t>(cy) * grid_nx_ + static_cast<std::size_t>(cx);
      for (std::uint32_t k = grid_off_[c]; k < grid_off_[c + 1]; ++k) {
        const std::uint32_t ei = grid_val_[k];
        if (!seen[ei]) {
          seen[ei] = 1;
          out.push_back(ei);
        }
      }
    }
  }
  return out;
}

std::uint32_t RoadMap::nearestEdge(double x, double y, double* dist_m, double search_m) const
{
  std::uint32_t best = 0xFFFFFFFFu;
  double best_d2 = search_m * search_m;
  for (double r = grid_cell_m_; r <= std::max(search_m, grid_cell_m_) * 4.0; r *= 2.0) {
    const auto cand = edgesInBBox(x - r, y - r, x + r, y + r);
    for (const std::uint32_t ei : cand) {
      const RoadEdge& e = edges_[ei];
      for (std::uint32_t k = 1; k < e.point_count; ++k) {
        const double d2 = distSqToSegment(x, y, point_x_[e.point_first + k - 1], point_y_[e.point_first + k - 1],
                                          point_x_[e.point_first + k], point_y_[e.point_first + k]);
        if (d2 < best_d2) {
          best_d2 = d2;
          best = ei;
        }
      }
    }
    if (best != 0xFFFFFFFFu)
      break;
  }
  if (dist_m)
    *dist_m = best == 0xFFFFFFFFu ? -1.0 : std::sqrt(best_d2);
  return best;
}

bool RoadMap::nearestPoint(double x, double y, double& nx, double& ny, double& dist_m, std::uint32_t& edge,
                           double search_m) const
{
  edge = 0xFFFFFFFFu;
  double best_d2 = search_m * search_m;
  double bx = 0.0, by = 0.0;
  for (double r = grid_cell_m_; r <= std::max(search_m, grid_cell_m_) * 4.0; r *= 2.0) {
    for (const std::uint32_t ei : edgesInBBox(x - r, y - r, x + r, y + r)) {
      const RoadEdge& e = edges_[ei];
      for (std::uint32_t k = 1; k < e.point_count; ++k) {
        double qx = 0.0, qy = 0.0;
        const double d2 = distSqToSegment(x, y, point_x_[e.point_first + k - 1], point_y_[e.point_first + k - 1],
                                          point_x_[e.point_first + k], point_y_[e.point_first + k], &qx, &qy);
        if (d2 < best_d2) {
          best_d2 = d2;
          bx = qx;
          by = qy;
          edge = ei;
        }
      }
    }
    if (edge != 0xFFFFFFFFu)
      break;
  }
  if (edge == 0xFFFFFFFFu) {
    dist_m = -1.0;
    return false;
  }
  nx = bx;
  ny = by;
  dist_m = std::sqrt(best_d2);
  return true;
}

const std::uint32_t* RoadMap::outEdges(std::uint32_t node, std::size_t* count) const
{
  if (node + 1 >= node_edge_off_.size()) {
    if (count)
      *count = 0;
    return nullptr;
  }
  const std::uint32_t a = node_edge_off_[node];
  const std::uint32_t b = node_edge_off_[node + 1];
  if (count)
    *count = b - a;
  return node_edge_val_.data() + a;
}

double RoadMap::headingAtStart(std::uint32_t edge) const
{
  const RoadEdge& e = edges_[edge];
  if (e.point_count < 2)
    return 0.0;
  return std::atan2(point_y_[e.point_first + 1] - point_y_[e.point_first],
                    point_x_[e.point_first + 1] - point_x_[e.point_first]);
}

double RoadMap::headingAtEnd(std::uint32_t edge) const
{
  const RoadEdge& e = edges_[edge];
  if (e.point_count < 2)
    return 0.0;
  const std::uint32_t last = e.point_first + e.point_count - 1;
  return std::atan2(point_y_[last] - point_y_[last - 1], point_x_[last] - point_x_[last - 1]);
}

double RoadMap::turnAngle(std::uint32_t edge_in, std::uint32_t edge_out) const
{
  double d = headingAtStart(edge_out) - headingAtEnd(edge_in);
  while (d > M_PI)
    d -= 2.0 * M_PI;
  while (d < -M_PI)
    d += 2.0 * M_PI;
  return d;
}

}  // namespace mapmatch
}  // namespace adas
