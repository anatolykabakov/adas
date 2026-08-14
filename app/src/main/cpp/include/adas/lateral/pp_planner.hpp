#pragma once

#include <optional>
#include <vector>

#include <cmath>

#include "adas/lateral/limits.hpp"
#include "adas/lateral/planner.hpp"
#include "adas/lateral/types.hpp"

namespace adas {
namespace lateral {
/**
 * \brief Pure pursuit: steer by the angle to a target taken on the reference line.
 *
 * The steering ceiling here is constant rather than speed-dependent as in the MPC family: pure
 * pursuit has always been clamped that way, and the speed-dependent limit would silently change it.
 */
class PpPlanner final : public IPlanner {
public:
  struct Config {
    double k_dd = 0.4;            ///< Lookahead per unit speed [s].
    double waypoint_shift = 1.4;  ///< Lateral offset applied to the reference [m], positive left.
    double ld_min = 3.0;          ///< Lower clamp on the lookahead [m].
    double ld_max = 20.0;         ///< Upper clamp on the lookahead [m].
    double ld_curv_gain = 0.0;    ///< Extra lookahead per unit curvature [m].

    VehicleParams vehicle{};  ///< The car: geometry, mass distribution, stiffness, road bank.

    double max_steer_rad = 8.0 * M_PI / 180.0;  ///< Ceiling on the commanded angle [rad].
  };

  explicit PpPlanner(Config config) : cfg_(std::move(config)) {}

  const char* name() const override { return "pp"; }

  Output update(const Input& in) override;

  const Config& config() const { return cfg_; }
  void setConfig(const Config& config) { cfg_ = config; }

private:
  double lookaheadFor(double speed_mps, const std::vector<Vec2>& poly) const;

  /// First intersection of the lookahead circle with the line ahead of the car.
  static std::optional<Vec2> targetPoint(double lookahead_m, const std::vector<Vec2>& poly);

  Config cfg_;
};

}  // namespace lateral
}  // namespace adas
