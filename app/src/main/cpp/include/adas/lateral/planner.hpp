#pragma once

#include "adas/lateral/types.hpp"

namespace adas {
namespace lateral {

class IPlanner {
public:
  virtual ~IPlanner() = default;

  virtual const char* name() const = 0;

  /** Какой численный метод считает кривизну. Пусто, если у планера выбора нет. */
  virtual const char* solverName() const { return ""; }

  virtual bool available() const { return true; }

  virtual void reset() {}

  virtual Output update(const Input& in) = 0;
};

}  // namespace lateral
}  // namespace adas
