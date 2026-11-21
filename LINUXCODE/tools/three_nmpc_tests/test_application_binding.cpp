#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"

#include <cassert>
#include <cmath>

namespace {
struct Capture {
  std::array<double, 3> disturbance{};
  unsigned calls{};
};

int solve_block(void* context, p112::DecoupledAxis axis,
                const p112::AxisState4&, const p112::AxisReference4&,
                double disturbance, double,
                p112::AxisTorque2& torque) noexcept {
  auto& capture = *static_cast<Capture*>(context);
  const auto index = static_cast<unsigned>(axis);
  capture.disturbance[index] = disturbance;
  ++capture.calls;
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
  control_stack::ThreeNmpcControlAlgorithm algorithm(controller, nullptr,
                                                     map_input, 0.01);
  algorithm.reset({control_stack::MpcProfile::Locomotion,
                   control_stack::ControlBackend::Mpc});

  control_stack::SensorFrame sensors{};
  sensors.timestamp_us = 10000;
  sensors.eso_valid = true;
  sensors.eso_disturbance = {0.3, -0.2, 0.1};
  const auto output = algorithm.update(sensors, {});
  assert(output.valid);
  assert(capture.calls == 3);
  assert(std::abs(capture.disturbance[0] - 0.3) < 1e-12);
  assert(std::abs(capture.disturbance[1] + 0.2) < 1e-12);
  assert(std::abs(capture.disturbance[2] - 0.1) < 1e-12);
  assert(output.torque[0] == 1.0 && std::abs(output.torque[5] + 0.8) < 1e-12);

  sensors.timestamp_us += 10000;
  sensors.eso_valid = false;
  assert(!algorithm.update(sensors, {}).valid);
  assert(capture.calls == 3);
}
