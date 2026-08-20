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

/** The compact road graph: edges with polylines, names, and a spatial index. */
class RoadMap {
public:
  /**
   * \brief Load a prebuilt `.admap`.
   * \param[in] path File path.
   * \return False when the file is missing or its format is not the one this build expects.
   */
  bool load(const std::string& path);

  /// \return True before a successful load().
  bool empty() const { return edges_.empty(); }
  /// \return Number of graph nodes.
  std::size_t nodeCount() const { return node_x_.size(); }
  /// \return Number of edges.
  std::size_t edgeCount() const { return edges_.size(); }
  /// The projection this map's metres are in. Not the same anchor a drive's own local frame uses.
  const LocalFrame& frame() const { return frame_; }

  /// \return Node east coordinate [m].
  double nodeX(std::uint32_t i) const { return node_x_[i]; }
  /// \return Node north coordinate [m].
  double nodeY(std::uint32_t i) const { return node_y_[i]; }
  /// \return Edge record by index.
  const RoadEdge& edge(std::uint32_t i) const { return edges_[i]; }
  /// Street name, or empty for an unnamed way.
  std::string edgeName(std::uint32_t i) const;

  /**
   * \brief Centreline of an edge, in map metres.
   * \param[in] i Edge index.
   * \param[out] xs East coordinates; cleared and refilled.
   * \param[out] ys North coordinates.
   */
  void edgePolyline(std::uint32_t i, std::vector<double>& xs, std::vector<double>& ys) const;

  /**
   * \brief Edges intersecting an axis-aligned box.
   * \param[in] x0,y0 One corner in map metres; \param[in] x1,y1 the opposite one. Order does not matter.
   * \return Edge indices, unordered.
   */
  std::vector<std::uint32_t> edgesInBBox(double x0, double y0, double x1, double y1) const;

  /**
   * \brief Edge nearest to a point.
   * \param[in] x,y Query point in map metres.
   * \param[out] dist_m Distance to it [m]; may be null.
   * \param[in] search_m Radius to search within [m].
   * \return Edge index, or `0xFFFFFFFF` when nothing was found — an unsigned value, so a `>= 0` check
   */
  std::uint32_t nearestEdge(double x, double y, double* dist_m = nullptr, double search_m = 200.0) const;

  /**
   * \brief Nearest point on the road graph, not just the nearest edge.
   * \param[in] x,y Query point in map metres.
   * \param[out] nx,ny The projected point.
   * \param[out] dist_m Distance to it [m].
   * \param[out] edge Edge the point lies on.
   * \return False when nothing was found within the search radius.
   */
  bool nearestPoint(double x, double y, double& nx, double& ny, double& dist_m, std::uint32_t& edge,
                    double search_m = 200.0) const;

  /**
   * \brief Edges leaving a node, for route expansion.
   * \param[in] node Node index.
   * \param[out] count How many edges the returned pointer covers.
   * \return Pointer into the map's own storage; valid as long as the map is.
   */
  const std::uint32_t* outEdges(std::uint32_t node, std::size_t* count) const;

  /// Heading of an edge at its ends [rad], for matching direction of travel.
  double headingAtStart(std::uint32_t edge) const;
  /// \return Heading [rad] at the edge's far end.
  double headingAtEnd(std::uint32_t edge) const;

  /// Turn between two connected edges [rad]; how a route penalises implausible turns.
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
