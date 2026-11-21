#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"

#include <cassert>
#include <cmath>
#include <thread>

namespace {
struct Capture {
  std::array<double, 3> disturbance{};
  std::array<unsigned, 3> calls{};
  std::array<std::thread::id, 3> worker{};
};

int solve_block(void* context, p112::DecoupledAxis axis,
                const p112::AxisState4&, const p112::AxisReference4&,
                double disturbance, double,
                p112::AxisTorque2& torque) noexcept {
  auto& capture = *static_cast<Capture*>(context);
  const auto index = static_cast<unsigned>(axis);
  capture.disturbance[index] = disturbance;
  capture.worker[index] = std::this_thread::get_id();
  ++capture.calls[index];
  torque.value = {double(index + 1), -double(index + 1)};
  return 0;
}

bool map_input(void*, const control_stack::SensorFrame&,
               const control_stack::Command&, control_stack::ControlSelection,
               p112::DecoupledInput& input) noexcept {
  for (auto& reference : input.reference) reference.value = {0, 0, 0, 0};
  for (auto& state : input.axis_state) state.value = {0, 0, 0, 0};
  return true;
}
}  // namespace

int main() {
  Capture capture{};
  p112::DecoupledController controller(&capture, solve_block);
  assert(controller.start_parallel());
  control_stack::ThreeNmpcControlAlgorithm algorithm(controller, nullptr,
                                                     map_input, 0.01);
  algorithm.reset({control_stack::MpcProfile::Locomotion,
                   control_stack::ControlBackend::Mpc});

  control_stack::SensorFrame sensors{};
  sensors.timestamp_us = 10000;
  sensors.eso_valid = true;
  sensors.attitude_valid = true;
  sensors.kinematic_state_valid = true;
  sensors.wheel_speed_valid = true;
  sensors.body_velocity_valid = true;
  sensors.eso_disturbance = {0.3, -0.2, 0.1};
  auto output = algorithm.update(sensors, {});
  assert(output.valid && capture.calls[0] + capture.calls[1] + capture.calls[2] == 0);
  for (unsigned sample = 1; sample < 5; ++sample) {
    sensors.timestamp_us += 2000;
    output = algorithm.update(sensors, {});
  }
  assert(output.valid);
  assert(capture.calls[0] == 1 && capture.calls[1] == 1 && capture.calls[2] == 1);
  const auto caller = std::this_thread::get_id();
  for (const auto worker : capture.worker) assert(worker != caller);
  assert(std::abs(capture.disturbance[0] - 0.3) < 1e-12);
  assert(std::abs(capture.disturbance[1] + 0.2) < 1e-12);
  assert(std::abs(capture.disturbance[2] - 0.1) < 1e-12);
  assert(output.torque[0] > 0.9 && output.torque[4] < -0.9);
  assert(output.torque[2] < 0.0 && output.torque[5] > 0.0);

  sensors.timestamp_us += 2000;
  sensors.eso_valid = false;
  assert(!algorithm.update(sensors, {}).valid);
  assert(capture.calls[0] == 1 && capture.calls[1] == 1 && capture.calls[2] == 1);
}
