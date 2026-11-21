#include "serial_leg_dynamics.h"

#include <cmath>

namespace p112 {

SerialLegAxisDynamics serial_leg_axis_dynamics_defaults(
    DecoupledAxis axis) noexcept {
    SerialLegAxisDynamics model{};
    model.vector_field = {
        0., 1., 0., 0., -18., -2.8, 0., 0.,
        0., 0., 0., 1., 0., 0., -12., -2.0};

    switch (axis) {
    case DecoupledAxis::roll:
        model.input_gain = {0., 0., 0., 0., 0., 0., 1.8, -1.8};
        break;
    case DecoupledAxis::pitch:
        model.input_gain = {0., 0., 0., 0., 0., 0., 1.5, -1.5};
        break;
    case DecoupledAxis::yaw:
        model.vector_field = {
            0., 1., 0., 0., -10., -2.0, 0., 0.,
            0., 0., 0., 1., 0., 0., -7., -1.2};
        model.input_gain = {0., 0., 0., 0., 0., 0., 1.2, -1.2};
        break;
    }
    return model;
}

void evaluate_serial_leg_axis_dynamics(
    const SerialLegAxisDynamics& model, const AxisState4& state,
    const AxisTorque2& input, AxisState4& derivative,
    double disturbance) noexcept {
    const auto& x = state.value;
    const auto& u = input.value;
    auto& f = derivative.value;

    f[0] = x[1];
    f[1] = model.vector_field[4] * std::sin(x[0]) +
           model.vector_field[5] * x[1] +
           model.vector_field[6] * std::sin(x[2]) +
           model.vector_field[7] * x[3] +
           model.input_gain[2] * u[0] + model.input_gain[3] * u[1] +
           disturbance;
    f[2] = x[3];
    f[3] = model.vector_field[12] * std::sin(x[0]) +
           model.vector_field[13] * x[1] +
           model.vector_field[14] * std::sin(x[2]) +
           model.vector_field[15] * x[3] +
           model.input_gain[6] * u[0] + model.input_gain[7] * u[1];
}

void linearize_serial_leg_axis_dynamics(
    const SerialLegAxisDynamics& model, const AxisState4& state,
    std::array<double, 16>& jacobian) noexcept {
    const auto& x = state.value;
    jacobian.fill(0.0);
    jacobian[1] = 1.0;
    jacobian[4] = model.vector_field[4] * std::cos(x[0]);
    jacobian[5] = model.vector_field[5];
    jacobian[6] = model.vector_field[6] * std::cos(x[2]);
    jacobian[7] = model.vector_field[7];
    jacobian[11] = 1.0;
    jacobian[12] = model.vector_field[12] * std::cos(x[0]);
    jacobian[13] = model.vector_field[13];
    jacobian[14] = model.vector_field[14] * std::cos(x[2]);
    jacobian[15] = model.vector_field[15];
}

} // namespace p112
