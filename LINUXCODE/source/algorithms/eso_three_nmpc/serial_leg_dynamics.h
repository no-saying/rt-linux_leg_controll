#ifndef P112_SERIAL_LEG_DYNAMICS_H
#define P112_SERIAL_LEG_DYNAMICS_H

#include "axis_types.h"
#include <array>

namespace p112 {

// Reduced four-state axis model for the serial-leg controller. The current
// defaults are simulation placeholders, not a geometry-derived multibody model.
// State order is [body angle, body rate, leg coordinate, leg-coordinate rate];
// input order is the two axis torques supplied by the verified board mapper.
struct SerialLegAxisDynamics {
    std::array<double, 16> vector_field{};
    std::array<double, 8> input_gain{};
};

SerialLegAxisDynamics serial_leg_axis_dynamics_defaults(
    DecoupledAxis axis) noexcept;

void evaluate_serial_leg_axis_dynamics(
    const SerialLegAxisDynamics&, const AxisState4&, const AxisTorque2&,
    AxisState4&, double disturbance = 0.0) noexcept;

void linearize_serial_leg_axis_dynamics(
    const SerialLegAxisDynamics&, const AxisState4&,
    std::array<double, 16>& jacobian) noexcept;

} // namespace p112

#endif
