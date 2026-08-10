#pragma once

#include <memory>
#include <vector>

#include "adas/utils/math_utils.h"

namespace adas {
namespace flowpilot {

class AcadosLatMpc {
public:
  struct Result {
    bool ok = false;
    double desired_curvature = 0.0;
    double desired_curvature_rate = 0.0;
    double cost = 0.0;
    int status = -1;
    std::vector<double> psi_sol;
    std::vector<double> r_sol;
  };

  struct Weights {
    double path = 1.0;
    double heading = 0.11;
    double lat_accel = 0.0;
    double lat_jerk = 0.05;
    double steering_rate = 400.0;
  };

  AcadosLatMpc();
  ~AcadosLatMpc();
  AcadosLatMpc(const AcadosLatMpc&) = delete;
  AcadosLatMpc& operator=(const AcadosLatMpc&) = delete;

  bool available() const;

  static int horizonNodes();
  static double nodeTime(int i);

  void setWeights(const Weights& w);
  void reset();

  Result solve(double v_ego, double rotation_radius, double yaw_rate, const std::vector<double>& y_ref,
               const std::vector<double>& psi_ref, const std::vector<double>& r_ref, double steer_delay_s);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flowpilot
}  // namespace adas
