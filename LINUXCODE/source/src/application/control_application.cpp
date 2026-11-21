#include "control_stack/application/control_application.hpp"

namespace control_stack {

void ControlApplication::reset(ControlSelection selection) {
  deps_.perception.reset();
  deps_.greenwave.reset();
  deps_.control.reset(selection);
  if (deps_.log != nullptr)
    deps_.log->write({LogLevel::Info, 0, "control_reset", "selection changed"});
}

ControlOutput ControlApplication::step(const SensorFrame& sensors,
                                       const Command& command) {
  // Application code only composes algorithms. Platform services remain
  // injected and are not referenced by the algorithms.
  SensorFrame observed = sensors;
  const auto perception = deps_.perception.update(sensors);
  observed.wheel_slip = perception.wheel_slip;
  observed.wheel_torque_scale = perception.wheel_torque_scale;
  const auto plan = deps_.greenwave.update(observed, command);
  const auto output = deps_.control.update(observed, plan.valid ? plan.command : command);
  if (!output.valid && deps_.log != nullptr)
    deps_.log->write({LogLevel::Warning, sensors.timestamp_us,
                      "invalid_control_output", "algorithm returned invalid output"});
  return output;
}

}
