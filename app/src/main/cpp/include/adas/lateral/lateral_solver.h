#pragma once

#include "adas/utils/adas_topics.h"

namespace adas {

struct LaneKeepOutput;

namespace lateral {

struct SolverInput {
  double speed_mps = 0.0;
  const LanePathMsg* path = nullptr;
  double yaw_rate = 0.0;
  bool have_chassis = false;
  double frame_dt_s = 0.05;
};

class KappaSolver {
public:
  virtual ~KappaSolver() = default;
  virtual const char* name() const = 0;
  virtual bool available() const { return true; }
  virtual void reset() {}
  virtual bool solve(const SolverInput& in, const std::vector<Vec2>& poly, const std::vector<Vec2>& plan_poly,
                     double yaw, double Lf, double& kappa, double& kappa_rate) = 0;
};

class Solver {
public:
  virtual ~Solver() = default;

  virtual const char* name() const = 0;

  virtual bool available() const { return true; }

  virtual void reset() {}

  virtual LaneKeepOutput solve(const SolverInput& in) = 0;
};

}  // namespace lateral
}  // namespace adas
