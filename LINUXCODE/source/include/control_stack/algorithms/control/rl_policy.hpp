#pragma once

#include "control_stack/common/types.hpp"

namespace control_stack {

class IRlPolicy {
 public:
  virtual ~IRlPolicy() = default;
  virtual void reset() = 0;
  virtual ControlOutput infer(const SensorFrame&, const Command&) = 0;
};

}
