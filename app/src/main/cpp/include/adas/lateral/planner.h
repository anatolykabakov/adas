#pragma once

#include "adas/lateral/types.h"

namespace adas {
namespace lateral {
/** Interface every lateral planner implements: lane lines in, steering plan out. */
class IPlanner {
public:
  virtual ~IPlanner() = default;

  /// Planner name as written to the bag: "pp", "vp" or "fp".
  virtual const char* name() const = 0;

  /** Which numerical method computes the curvature. Empty when the planner has no choice. */
  virtual const char* solverName() const { return ""; }

  /// \return False when the backing solver cannot run (missing library).
  virtual bool available() const { return true; }

  /// Drop accumulated state between runs.
  virtual void reset() {}

  /// One planning tick. \param[in] in Speed and reference path. \return Curvature plan and diagnostics.
  virtual Output update(const Input& in) = 0;
};

}  // namespace lateral
}  // namespace adas
