#pragma once

#include "control_stack/algorithms/control/control_algorithm.hpp"
#include "control_stack/algorithms/greenwave/greenwave_algorithm.hpp"
#include "control_stack/algorithms/perception/perception_algorithm.hpp"
#include "control_stack/platform/logging/logger.hpp"

namespace control_stack {

class ControlApplication {
 public:
  struct Dependencies {
    IControlAlgorithm& control;
    IPerceptionAlgorithm& perception;
    IGreenWaveAlgorithm& greenwave;
    ILogSink* log{nullptr};
  };

  explicit ControlApplication(Dependencies deps) : deps_(deps) {}
  void reset(ControlSelection selection);
  ControlOutput step(const SensorFrame& sensors, const Command& command);

 private:
  Dependencies deps_;
};

}
