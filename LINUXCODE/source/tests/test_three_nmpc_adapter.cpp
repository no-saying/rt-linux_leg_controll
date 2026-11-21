#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"

#include <cassert>
#include <cmath>

namespace {
struct Capture {
  control_stack::Command command{};
  unsigned solves{};
};

int solve_block(void* context, p112::DecoupledAxis,
                const p112::AxisState4&, const p112::AxisReference4&,
                double, double, p112::AxisTorque2& torque) noexcept {
  auto& capture = *static_cast<Capture*>(context);
  ++capture.solves;
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
  sensors.timestamp_us = 10000;
  sensors.eso_valid = true;
  sensors.wheel_slip = {true, false};
  sensors.wheel_torque_scale = {0.25, 1.0};
  control_stack::Command command{};
  command.x_speed_m_s = 0.4;
  command.z_speed_m_s = 1.2;

  const auto output = algorithm.update(sensors, command);
  assert(output.valid && capture.solves == 3);
  assert(capture.command.x_speed_m_s == command.x_speed_m_s);
  assert(capture.command.z_speed_m_s == command.z_speed_m_s);
  assert(output.torque[0] == 2.0 && output.torque[3] == -2.0);
  assert(output.torque[1] == 2.0 && output.torque[4] == -2.0);
  assert(std::abs(output.torque[2] - 0.2) < 1e-12);
  assert(output.torque[5] == -0.8);

  sensors.timestamp_us += 10000;
  sensors.wheel_slip = {true, true};
  sensors.wheel_torque_scale = {2.0, -1.0};
  const auto clamped = algorithm.update(sensors, command);
  assert(clamped.valid);
  assert(std::abs(clamped.torque[2] - 0.8) < 1e-12);
  assert(clamped.torque[5] == 0.0);

  sensors.timestamp_us += 10000;
  sensors.wheel_slip = {false, false};
  sensors.wheel_torque_scale = {0.1, 0.1};
  const auto no_slip = algorithm.update(sensors, command);
  assert(no_slip.valid);
  assert(std::abs(no_slip.torque[2] - 0.8) < 1e-12);
  assert(no_slip.torque[5] == -0.8);
}
