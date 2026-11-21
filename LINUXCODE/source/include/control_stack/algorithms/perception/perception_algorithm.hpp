#pragma once

#include "control_stack/common/types.hpp"

namespace control_stack {

class IPerceptionAlgorithm {
 public:
  virtual ~IPerceptionAlgorithm() = default;
  virtual void reset() = 0;
  virtual PerceptionState update(const SensorFrame&) = 0;
};

}
