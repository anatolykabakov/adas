#pragma once

#include "adas/lateral/types.hpp"

namespace adas {

namespace lateral {

class KappaSolver {
public:
  virtual ~KappaSolver() = default;
  virtual const char* name() const = 0;
  virtual bool available() const { return true; }
  virtual void reset() {}
  virtual bool solve(const Input& in, const std::vector<Vec2>& poly, const std::vector<Vec2>& plan_poly, double yaw,
                     double Lf, double& kappa, double& kappa_rate) = 0;
};

}  // namespace lateral
}  // namespace adas
