#pragma once

#include "control_stack/common/fixed_ring_buffer.hpp"
#include "control_stack/common/types.hpp"

namespace control_stack {

enum class Topic : std::uint8_t {
  Command = 0,
  Sensor = 1,
  Perception = 2,
  Plan = 3,
  ControlOutput = 4,
  Fault = 5,
};

class MessageCenter {
 public:
  bool publishCommand(const Command& value) { return commands_.push(value); }
  bool publishSensor(const SensorFrame& value) { return sensors_.push(value); }
  bool publishPerception(const PerceptionState& value) { return perceptions_.push(value); }
  bool publishPlan(const GreenWavePlan& value) { return plans_.push(value); }
  bool publishControlOutput(const ControlOutput& value) { return outputs_.push(value); }
  bool publishFault(const FaultEvent& value) { return faults_.push(value); }

  bool pollCommand(Command& value) { return commands_.pop(value); }
  bool pollSensor(SensorFrame& value) { return sensors_.pop(value); }
  bool pollPerception(PerceptionState& value) { return perceptions_.pop(value); }
  bool pollPlan(GreenWavePlan& value) { return plans_.pop(value); }
  bool pollControlOutput(ControlOutput& value) { return outputs_.pop(value); }
  bool pollFault(FaultEvent& value) { return faults_.pop(value); }

  void clear() {
    commands_.clear(); sensors_.clear(); perceptions_.clear();
    plans_.clear(); outputs_.clear(); faults_.clear();
  }

 private:
  StaticRingBuffer<Command, 8> commands_{};
  StaticRingBuffer<SensorFrame, 4> sensors_{};
  StaticRingBuffer<PerceptionState, 4> perceptions_{};
  StaticRingBuffer<GreenWavePlan, 4> plans_{};
  StaticRingBuffer<ControlOutput, 4> outputs_{};
  StaticRingBuffer<FaultEvent, 8> faults_{};
};

}
