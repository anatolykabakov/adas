#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "adas/mapmatch/geo.h"

namespace adas {
namespace mapmatch {
struct RoadEdge {
  std::uint32_t node_a = 0;
  std::uint32_t node_b = 0;
  std::uint32_t point_first = 0;
  std::uint16_t point_count = 0;
  std::uint32_t name_id = 0xFFFFFFFFu;
  float length_m = 0.0f;
  bool oneway = false;
};

class RoadMap {
public:
  bool load(const std::string& path);

  bool empty() const { return edges_.empty(); }
  std::size_t nodeCount() const { return node_x_.size(); }
  std::size_t edgeCount() const { return edges_.size(); }
  const LocalFrame& frame() const { return frame_; }

  double nodeX(std::uint32_t i) const { return node_x_[i]; }
  double nodeY(std::uint32_t i) const { return node_y_[i]; }
  const RoadEdge& edge(std::uint32_t i) const { return edges_[i]; }
  std::string edgeName(std::uint32_t i) const;

  void edgePolyline(std::uint32_t i, std::vector<double>& xs, std::vector<double>& ys) const;

  std::vector<std::uint32_t> edgesInBBox(double x0, double y0, double x1, double y1) const;

  std::uint32_t nearestEdge(double x, double y, double* dist_m = nullptr, double search_m = 200.0) const;

  bool nearestPoint(double x, double y, double& nx, double& ny, double& dist_m, std::uint32_t& edge,
                    double search_m = 200.0) const;

  const std::uint32_t* outEdges(std::uint32_t node, std::size_t* count) const;

  double headingAtStart(std::uint32_t edge) const;
  double headingAtEnd(std::uint32_t edge) const;

  double turnAngle(std::uint32_t edge_in, std::uint32_t edge_out) const;

private:
  double pointX(std::uint32_t i) const { return point_x_[i]; }
  double pointY(std::uint32_t i) const { return point_y_[i]; }

  LocalFrame frame_{};
  std::vector<double> node_x_, node_y_;
  std::vector<RoadEdge> edges_;
  std::vector<double> point_x_, point_y_;
  std::vector<std::uint32_t> node_edge_off_, node_edge_val_;
  std::vector<std::uint32_t> grid_off_, grid_val_;
  std::vector<std::string> names_;
  double grid_x0_ = 0.0, grid_y0_ = 0.0, grid_cell_m_ = 500.0;
  std::uint32_t grid_nx_ = 0, grid_ny_ = 0;
};

}  // namespace mapmatch
}  // namespace adas
