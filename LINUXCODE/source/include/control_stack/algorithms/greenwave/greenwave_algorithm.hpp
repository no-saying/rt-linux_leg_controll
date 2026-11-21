#pragma once

#include "control_stack/common/types.hpp"

namespace control_stack {

class IGreenWaveAlgorithm {
 public:
  virtual ~IGreenWaveAlgorithm() = default;
  virtual void reset() = 0;
  virtual GreenWavePlan update(const SensorFrame&, const Command&) = 0;
};

}
