#pragma once

#include "control_stack/algorithms/control/control_algorithm.hpp"
#include "control_stack/algorithms/greenwave/greenwave_algorithm.hpp"
#include "control_stack/algorithms/perception/basic_robot_perception.hpp"
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

  explicit ControlApplication(Dependencies deps)
      : control_(&deps.control), perception_(&deps.perception),
        greenwave_(&deps.greenwave), log_(deps.log) {}
  ControlApplication(IControlAlgorithm& control, IGreenWaveAlgorithm& greenwave,
                     ILogSink* log = nullptr)
      : control_(&control), perception_(&default_perception_),
        greenwave_(&greenwave), log_(log) {}
  void reset(ControlSelection selection);
  PerceptionState perceive(const SensorFrame& sensors);
  ControlOutput step(const SensorFrame& sensors, const Command& command);
  ControlOutput step(const SensorFrame& sensors, const Command& command,
                     const PerceptionState& perception);

 private:
  IControlAlgorithm* control_;
  BasicRobotPerception default_perception_{};
  IPerceptionAlgorithm* perception_;
  IGreenWaveAlgorithm* greenwave_;
  ILogSink* log_;
};

}
