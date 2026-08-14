#pragma once

#include <cstdint>
#include <vector>

#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/track.h"

namespace adas {
namespace mapmatch {
struct FitConfig {
  double sample_m = 10.0;                 ///< Spacing of the points being fitted [m].
  double sigma_road_m = 4.0;              ///< Assumed distance from the true road [m].
  double sigma_speed_scale = 0.03;        ///< Prior spread on the speed-scale correction.
  double sigma_yaw_scale = 0.05;          ///< Prior spread on the yaw-scale correction.
  double sigma_turn_deg = 6.0;            ///< Assumed error of a turn angle [deg].
  double sigma_straight_rel = 0.08;       ///< Assumed relative error of a straight length.
  double drift_block_m = 300.0;           ///< Length over which heading drift is modelled as constant [m].
  double sigma_drift_deg_per_100m = 0.6;  ///< Assumed heading drift [deg per 100 m].
  double max_residual_m = 25.0;           ///< Points further off than this are treated as outliers [m].
  int iterations = 30;                    ///< Optimiser iterations.
  int anneal_steps = 4;                   ///< Annealing stages, from a wide search to a tight one.
  double anneal_start_scale = 12.0;       ///< How much wider the first stage searches.
};

struct FitResult {
  bool ok = false;
  double x0_m = 0.0, y0_m = 0.0, heading_rad = 0.0;
  double speed_scale = 1.0, yaw_rate_scale = 1.0;
  std::vector<double> turn_corr_deg;
  std::vector<double> straight_corr;
  std::vector<double> drift_deg_per_100m;
  std::vector<double> x_m, y_m;

  double rms_m = 0.0;
  double median_m = 0.0;
  double p95_m = 0.0;
  double deform_cost = 0.0;
  double score = 0.0;
  int iterations = 0;
};

FitResult fitTrack(const RoadMap& map, const Track& track, double x0_m, double y0_m, double heading_rad,
                   const FitConfig& cfg = {});

FitResult fitTrackToRoute(const RoadMap& map, const Track& track, const std::vector<std::uint32_t>& dir_edges,
                          const FitConfig& cfg = {});

}  // namespace mapmatch
}  // namespace adas
