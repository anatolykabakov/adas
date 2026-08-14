#pragma once

#include "adas/lateral/types.hpp"

namespace adas {
namespace lateral {
/**
 * \brief Interface every lateral planner implements: lane lines in, steering plan out.
 *
 * \details Three planners sit behind it — pure pursuit, visionpilot and flowpilot — and the choice is a
 * runtime parameter, so the service that drives them never names one. `available()` exists because a
 * planner can be compiled out or depend on a solver missing from the build; asking rather than assuming
 * is what lets the configuration fall back instead of crashing.
 */
class IPlanner {
public:
  virtual ~IPlanner() = default;

  virtual const char* name() const = 0;

  /** Which numerical method computes the curvature. Empty when the planner has no choice. */
  virtual const char* solverName() const { return ""; }

  virtual bool available() const { return true; }

  virtual void reset() {}

  virtual Output update(const Input& in) = 0;
};

}  // namespace lateral
}  // namespace adas
