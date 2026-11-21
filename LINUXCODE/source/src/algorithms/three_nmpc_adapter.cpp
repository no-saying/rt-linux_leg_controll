#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace control_stack {

ThreeNmpcControlAlgorithm::ThreeNmpcControlAlgorithm(
    p112::DecoupledController& controller, void* mapper_context,
    ThreeNmpcInputMapper mapper, double nominal_period_s) noexcept
    : controller_(controller),
      mapper_context_(mapper_context),
      mapper_(mapper),
      nominal_period_s_(nominal_period_s) {}

void ThreeNmpcControlAlgorithm::reset(ControlSelection selection) {
  selection_ = selection;
  last_timestamp_us_ = 0;
  have_timestamp_ = false;
  controller_.reset();
}

ControlOutput ThreeNmpcControlAlgorithm::update(const SensorFrame& sensors,
                                                 const Command& command) {
  ControlOutput output{};
  if (mapper_ == nullptr || !sensors.eso_valid ||
      !std::isfinite(nominal_period_s_) || nominal_period_s_ <= 0.0)
    return output;

  double dt = nominal_period_s_;
  if (have_timestamp_) {
    if (sensors.timestamp_us <= last_timestamp_us_) return output;
    dt = static_cast<double>(sensors.timestamp_us - last_timestamp_us_) * 1e-6;
  }
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.02) return output;

  p112::DecoupledInput input{};
  if (!mapper_(mapper_context_, sensors, command, selection_, input)) return output;
  input.sequence = sensors.timestamp_us;
  input.dt = dt;
  std::copy(sensors.eso_disturbance.begin(), sensors.eso_disturbance.end(),
            input.observed_eso.disturbance.begin());

  p112::DecoupledOutput result{};
  if (!controller_.step(input, result) || result.sequence != input.sequence)
    return output;

  std::copy(result.torque.begin(), result.torque.end(), output.torque.begin());
  // Motor order is LF, LR, left wheel, RF, RR, right wheel. Slip perception
  // may reduce only the matching drive-wheel torque; references stay intact.
  constexpr std::array<std::size_t, 2> kWheelMotorIndex{{2, 5}};
  for (std::size_t side = 0; side < kWheelMotorIndex.size(); ++side) {
    const double scale = sensors.wheel_torque_scale[side];
    if (sensors.wheel_slip[side] && std::isfinite(scale))
      output.torque[kWheelMotorIndex[side]] *= std::clamp(scale, 0.0, 1.0);
  }
  output.mode = static_cast<std::uint32_t>(selection_.profile);
  output.valid = true;
  output.confidence = 1.0;
  last_timestamp_us_ = sensors.timestamp_us;
  have_timestamp_ = true;
  return output;
}

bool ThreeNmpcControlAlgorithm::acknowledgeAppliedTorque(
    const TorqueVector& actual) noexcept {
  p112::Torque6 applied{};
  std::copy(actual.begin(), actual.end(), applied.begin());
  return controller_.acknowledge(applied);
}

}  // namespace control_stack
