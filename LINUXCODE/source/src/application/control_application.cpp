#include "control_stack/application/control_application.hpp"

namespace control_stack {

void ControlApplication::reset(ControlSelection selection) {
  greenwave_->reset();
  control_->reset(selection);
  if (log_ != nullptr)
    log_->write({LogLevel::Info, 0, "control_reset", "selection changed"});
}

PerceptionState ControlApplication::perceive(const SensorFrame& sensors) {
  return perception_->update(sensors);
}

ControlOutput ControlApplication::step(const SensorFrame& sensors,
                                       const Command& command) {
  return step(sensors, command, perceive(sensors));
}

ControlOutput ControlApplication::step(const SensorFrame& sensors,
                                       const Command& command,
                                       const PerceptionState& perception) {
  // Application code only composes algorithms. Platform services remain
  // injected and are not referenced by the algorithms.
  SensorFrame observed = sensors;
  observed.wheel_slip = perception.wheel_slip;
  observed.wheel_torque_scale = perception.wheel_torque_scale;
  observed.attitude_valid = perception.attitude_valid;
  observed.fallen = perception.fallen;
  observed.body_parallel = perception.body_parallel;
  observed.legs_centered = perception.legs_centered;
  observed.support_valid = perception.support_valid;
  observed.support = perception.support;
  observed.support_confidence = perception.support_confidence;
  const auto plan = greenwave_->update(observed, command);
  const auto output = control_->update(observed, plan.valid ? plan.command : command);
  if (!output.valid && log_ != nullptr)
    log_->write({LogLevel::Warning, sensors.timestamp_us,
                      "invalid_control_output", "algorithm returned invalid output"});
  return output;
}

}
