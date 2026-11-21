#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"

#include <cassert>
#include <cmath>

namespace {
struct Capture {
  control_stack::Command command{};
  unsigned solves{};
  unsigned failures_remaining{};
  double last_dt{};
};

int solve_block(void* context, p112::DecoupledAxis,
                const p112::AxisState4&, const p112::AxisReference4&,
                double, double dt, p112::AxisTorque2& torque) noexcept {
  auto& capture = *static_cast<Capture*>(context);
  ++capture.solves;
  capture.last_dt = dt;
  if (capture.failures_remaining > 0) {
    --capture.failures_remaining;
    return -1;
  }
  torque.value = {2.0, -2.0};
  return 0;
}

bool map_input(void* context, const control_stack::SensorFrame&,
               const control_stack::Command& command,
               control_stack::ControlSelection,
               p112::DecoupledInput& input) noexcept {
  static_cast<Capture*>(context)->command = command;
  for (auto& state : input.axis_state) state.value = {};
  for (auto& reference : input.reference) reference.value = {};
  return true;
}
}  // namespace

int main() {
  Capture capture{};
  p112::DecoupledController controller(&capture, solve_block);
  control_stack::ThreeNmpcControlAlgorithm algorithm(
      controller, &capture, map_input, 0.01);
  algorithm.reset({control_stack::MpcProfile::Locomotion,
                   control_stack::ControlBackend::Mpc});

  control_stack::SensorFrame sensors{};
  control_stack::Command command{};
  command.x_speed_m_s = 0.4;
  command.z_speed_m_s = 1.2;
  sensors.timestamp_us = 10000;
  sensors.eso_valid = true;
  auto output = algorithm.update(sensors, command);
  assert(!output.valid && capture.solves == 0);
  sensors.attitude_valid = true;
  sensors.kinematic_state_valid = true;
  sensors.wheel_speed_valid = true;
  sensors.body_velocity_valid = true;
  sensors.wheel_slip = {true, false};
  sensors.wheel_torque_scale = {0.25, 1.0};
  output = algorithm.update(sensors, command);
  assert(output.valid && capture.solves == 0);
  for (unsigned sample = 1; sample < 5; ++sample) {
    sensors.timestamp_us += 2000;
    output = algorithm.update(sensors, command);
    assert(output.valid);
    assert(capture.solves == (sample == 4 ? 3U : 0U));
  }
  assert(std::abs(capture.last_dt - 0.01) < 1e-12);
  assert(capture.command.x_speed_m_s == command.x_speed_m_s);
  assert(capture.command.z_speed_m_s == command.z_speed_m_s);
  assert(output.torque[0] > 0.0 && output.torque[4] < 0.0);
  assert(std::abs(output.torque[1]) < 1e-4 && std::abs(output.torque[3]) < 1e-4);
  assert(std::abs(output.torque[2] + 0.2) < 1e-12);
  assert(std::abs(output.torque[5] - 0.8) < 1e-12);

  for (unsigned sample = 0; sample < 4; ++sample) {
    sensors.timestamp_us += 2000;
    output = algorithm.update(sensors, command);
    assert(output.valid && capture.solves == 3);
  }
  sensors.timestamp_us += 2000;
  sensors.wheel_slip = {true, true};
  sensors.wheel_torque_scale = {2.0, -1.0};
  const auto clamped = algorithm.update(sensors, command);
  assert(clamped.valid && capture.solves == 6);
  assert(clamped.torque[2] < 0.0);
  assert(clamped.torque[5] == 0.0);
  assert(algorithm.acknowledgeAppliedTorque(clamped.torque));

  for (unsigned sample = 0; sample < 4; ++sample) {
    sensors.timestamp_us += 2000;
    const auto held = algorithm.update(sensors, command);
    assert(held.valid && capture.solves == 6);
  }
  sensors.timestamp_us += 2000;
  sensors.wheel_slip = {false, false};
  sensors.wheel_torque_scale = {0.1, 0.1};
  const auto no_slip = algorithm.update(sensors, command);
  assert(no_slip.valid && capture.solves == 9);
  assert(no_slip.torque[2] < 0.0);
  assert(no_slip.torque[5] > 0.0);

  Capture retry_capture{};
  p112::DecoupledController retry_controller(&retry_capture, solve_block);
  control_stack::ThreeNmpcControlAlgorithm retry_algorithm(
      retry_controller, &retry_capture, map_input, 0.01);
  retry_algorithm.reset({control_stack::MpcProfile::Locomotion,
                         control_stack::ControlBackend::Mpc});
  sensors.timestamp_us = 100000;
  sensors.wheel_slip = {};
  sensors.wheel_torque_scale = {1.0, 1.0};
  retry_capture.failures_remaining = 1;
  for (unsigned sample = 0; sample < 5; ++sample) {
    if (sample != 0) sensors.timestamp_us += 2000;
    output = retry_algorithm.update(sensors, command);
  }
  assert(!output.valid && retry_capture.solves == 1);
  sensors.timestamp_us += 2000;
  output = retry_algorithm.update(sensors, command);
  assert(output.valid && retry_capture.solves == 4);
  assert(std::abs(retry_capture.last_dt - 0.012) < 1e-12);
}
