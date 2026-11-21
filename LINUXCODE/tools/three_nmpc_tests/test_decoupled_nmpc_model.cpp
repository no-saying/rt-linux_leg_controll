#include "decoupled_nmpc.h"
#include <cassert>
#include <cmath>

int main() {
    using namespace p112;
    NmpcAxisConfig config{};
    auto& model = config.dynamics;
    model.vector_field = {0., 1., 0., 0., -18., -2.8, 1.1, 0.15,
                          0., 0., 0., 1., 0.8, 0.1, -12., -2.};
    model.input_gain = {0., 0., 0., 0., 0., 0., 1.8, -1.8};
    AxisState4 state{{0.37, -0.21, -0.19, 0.14}};
    AxisTorque2 input{{0.8, -0.3}};
    AxisState4 derivative{};
    std::array<double,16> jacobian{};
    evaluate_serial_leg_axis_dynamics(model, state, input, derivative);
    linearize_serial_leg_axis_dynamics(model, state, jacobian);

    // Finite differences verify the nonlinear derivative and catch an
    // accidental return to a fixed A matrix.
    constexpr double eps = 1e-7;
    for (unsigned col = 0; col < 4; ++col) {
        AxisState4 plus = state, minus = state, fp{}, fm{};
        plus.value[col] += eps;
        minus.value[col] -= eps;
        evaluate_serial_leg_axis_dynamics(model, plus, input, fp);
        evaluate_serial_leg_axis_dynamics(model, minus, input, fm);
        for (unsigned row = 0; row < 4; ++row) {
            const double numeric = (fp.value[row] - fm.value[row]) / (2.0 * eps);
            assert(std::abs(numeric - jacobian[row * 4 + col]) < 2e-7);
        }
    }
    const double expected = model.vector_field[4] * std::cos(state.value[0]);
    assert(std::abs(jacobian[4] - expected) < 1e-12);
    assert(std::abs(jacobian[4] - model.vector_field[4]) > 1e-3);

    std::array<double, kDecoupledNmpcVariables> warm{};
    DecoupledNmpcQp nonlinear_qp{}, zero_qp{};
    AxisState4 zero_state{};
    AxisReference4 zero_reference{};
    assert(build_decoupled_nmpc_qp(config, state, zero_reference, .01, 0., warm, nonlinear_qp));
    assert(build_decoupled_nmpc_qp(config, zero_state, zero_reference, .01, 0., warm, zero_qp));
    bool nonzero_defect = false;
    for (double value : nonlinear_qp.affine_defect) {
        if (std::abs(value) > 1e-8) nonzero_defect = true;
    }
    assert(nonzero_defect);
    bool state_dependent_hessian = false;
    for (unsigned i = 0; i < nonlinear_qp.h.size(); ++i) {
        if (std::abs(nonlinear_qp.h[i] - zero_qp.h[i]) > 1e-8) state_dependent_hessian = true;
    }
    assert(state_dependent_hessian);
    DecoupledNmpcQp disturbed_qp{};
    assert(build_decoupled_nmpc_qp(config, state, zero_reference, .01, .4, warm, disturbed_qp));
    bool disturbance_changes_prediction = false;
    for (unsigned i = 0; i < nonlinear_qp.affine_defect.size(); ++i) {
        if (std::abs(nonlinear_qp.affine_defect[i] - disturbed_qp.affine_defect[i]) > 1e-10)
            disturbance_changes_prediction = true;
    }
    assert(disturbance_changes_prediction);
    for (unsigned i = 0; i < kDecoupledNmpcHorizon; ++i) {
        assert(nonlinear_qp.a[i * kDecoupledNmpcVariables + i] == 1.0);
        assert(nonlinear_qp.lower[i] < 0.0 && nonlinear_qp.upper[i] > 0.0);
    }

    std::array<double, kDecoupledNmpcVariables> solution{};
    for (unsigned i = 0; i < solution.size(); ++i) solution[i] = static_cast<double>(i);
    std::array<double, kDecoupledNmpcVariables> shifted{};
    shift_decoupled_nmpc_warm_start(shifted, solution);
    for (unsigned k = 0; k + 1 < kDecoupledNmpcHorizon; ++k) {
        for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
            assert(shifted[k * kDecoupledNmpcInputSize + j] ==
                   solution[(k + 1) * kDecoupledNmpcInputSize + j]);
        }
    }
    assert(shifted.back() == solution.back());

    const auto defaults = DecoupledNmpcConfig::defaults();
    assert(defaults.axis[0].dynamics.input_gain[6] == 1.8);
    assert(defaults.axis[1].dynamics.input_gain[6] == 1.5);
    assert(defaults.axis[2].dynamics.input_gain[6] == 1.2);
    assert(defaults.axis[2].input_limit[0] == 3.0);
    return 0;
}
