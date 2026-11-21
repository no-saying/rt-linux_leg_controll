#include "decoupled_nmpc.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace p112 {
namespace {

void matvec4(const double *a, const double *x, double *y) noexcept {
    for (unsigned i = 0; i < kDecoupledNmpcStateSize; ++i) {
        y[i] = 0.0;
        for (unsigned j = 0; j < kDecoupledNmpcStateSize; ++j) {
            y[i] += a[i * kDecoupledNmpcStateSize + j] * x[j];
        }
    }
}

bool finite(const double *values, std::size_t count) noexcept {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) return false;
    }
    return true;
}

} // namespace

bool build_decoupled_nmpc_qp(
    const NmpcAxisConfig& config, const AxisState4& state,
    const AxisReference4& reference, double dt,
    double disturbance,
    const std::array<double, kDecoupledNmpcVariables>& warm_start,
    DecoupledNmpcQp& qp) noexcept {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.02 || !std::isfinite(disturbance) ||
        !finite(state.value.data(), kDecoupledNmpcStateSize) ||
        !finite(reference.value.data(), kDecoupledNmpcStateSize) ||
        !finite(warm_start.data(), kDecoupledNmpcVariables)) {
        return false;
    }

    qp = {};
    double nominal[(kDecoupledNmpcHorizon + 1) * kDecoupledNmpcStateSize]{};
    double ad[kDecoupledNmpcHorizon * kDecoupledNmpcStateSize * kDecoupledNmpcStateSize]{};
    double bd[kDecoupledNmpcHorizon * kDecoupledNmpcStateSize * kDecoupledNmpcInputSize]{};
    std::copy_n(state.value.data(), kDecoupledNmpcStateSize, nominal);

    for (unsigned k = 0; k < kDecoupledNmpcHorizon; ++k) {
        double u_nom[kDecoupledNmpcInputSize]{};
        for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
            const double limit = std::abs(config.input_limit[j]);
            u_nom[j] = std::clamp(warm_start[k * kDecoupledNmpcInputSize + j], -limit, limit);
        }
        AxisState4 nominal_state{}, derivative{};
        AxisTorque2 input{};
        std::copy_n(&nominal[k * kDecoupledNmpcStateSize],
                    kDecoupledNmpcStateSize, nominal_state.value.begin());
        std::copy_n(u_nom, kDecoupledNmpcInputSize, input.value.begin());
        std::array<double, kDecoupledNmpcStateSize * kDecoupledNmpcStateSize> jacobian{};
        evaluate_serial_leg_axis_dynamics(config.dynamics, nominal_state, input,
                                          derivative, disturbance);
        linearize_serial_leg_axis_dynamics(config.dynamics, nominal_state,
                                           jacobian);
        for (unsigned i = 0; i < kDecoupledNmpcStateSize; ++i) {
            for (unsigned j = 0; j < kDecoupledNmpcStateSize; ++j) {
                ad[k * kDecoupledNmpcStateSize * kDecoupledNmpcStateSize +
                   i * kDecoupledNmpcStateSize + j] =
                    (i == j ? 1.0 : 0.0) + dt * jacobian[i * kDecoupledNmpcStateSize + j];
            }
            for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
                bd[k * kDecoupledNmpcStateSize * kDecoupledNmpcInputSize +
                   i * kDecoupledNmpcInputSize + j] =
                    dt * config.dynamics.input_gain[
                        i * kDecoupledNmpcInputSize + j];
            }
        }
        for (unsigned i = 0; i < kDecoupledNmpcStateSize; ++i) {
            nominal[(k + 1) * kDecoupledNmpcStateSize + i] =
                nominal[k * kDecoupledNmpcStateSize + i] + dt * derivative.value[i];
        }
        double predicted[kDecoupledNmpcStateSize]{};
        matvec4(&ad[k * kDecoupledNmpcStateSize * kDecoupledNmpcStateSize],
                &nominal[k * kDecoupledNmpcStateSize], predicted);
        for (unsigned i = 0; i < kDecoupledNmpcStateSize; ++i) {
            for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
                predicted[i] += bd[k * kDecoupledNmpcStateSize * kDecoupledNmpcInputSize +
                                   i * kDecoupledNmpcInputSize + j] * u_nom[j];
            }
            qp.affine_defect[k * kDecoupledNmpcStateSize + i] =
                nominal[(k + 1) * kDecoupledNmpcStateSize + i] - predicted[i];
        }
    }

    // x_k = free_x_k + S_k u, where the defect is included in free_x_k.
    // Therefore g is for absolute physical inputs, not input increments.
    for (unsigned k = 1; k <= kDecoupledNmpcHorizon; ++k) {
        double free_x[kDecoupledNmpcStateSize]{};
        std::copy_n(state.value.data(), kDecoupledNmpcStateSize, free_x);
        for (unsigned stage = 0; stage < k; ++stage) {
            double next[kDecoupledNmpcStateSize]{};
            matvec4(&ad[stage * kDecoupledNmpcStateSize * kDecoupledNmpcStateSize], free_x, next);
            for (unsigned q = 0; q < kDecoupledNmpcStateSize; ++q) {
                free_x[q] = next[q] + qp.affine_defect[stage * kDecoupledNmpcStateSize + q];
            }
        }
        double error[kDecoupledNmpcStateSize]{};
        for (unsigned q = 0; q < kDecoupledNmpcStateSize; ++q) {
            error[q] = free_x[q] - reference.value[q];
        }
        double sens[kDecoupledNmpcHorizon * kDecoupledNmpcInputSize * kDecoupledNmpcStateSize]{};
        for (unsigned j = 0; j < k; ++j) {
            double p[kDecoupledNmpcStateSize * kDecoupledNmpcInputSize]{};
            std::memcpy(p, &bd[j * kDecoupledNmpcStateSize * kDecoupledNmpcInputSize], sizeof(p));
            for (unsigned stage = j + 1; stage < k; ++stage) {
                double next[kDecoupledNmpcStateSize * kDecoupledNmpcInputSize]{};
                for (unsigned col = 0; col < kDecoupledNmpcInputSize; ++col) {
                    double v[kDecoupledNmpcStateSize]{};
                    for (unsigned row = 0; row < kDecoupledNmpcStateSize; ++row) {
                        v[row] = p[row * kDecoupledNmpcInputSize + col];
                    }
                    double w[kDecoupledNmpcStateSize]{};
                    matvec4(&ad[stage * kDecoupledNmpcStateSize * kDecoupledNmpcStateSize], v, w);
                    for (unsigned row = 0; row < kDecoupledNmpcStateSize; ++row) {
                        next[row * kDecoupledNmpcInputSize + col] = w[row];
                    }
                }
                std::memcpy(p, next, sizeof(p));
            }
            for (unsigned q = 0; q < kDecoupledNmpcStateSize; ++q) {
                for (unsigned r = 0; r < kDecoupledNmpcInputSize; ++r) {
                    sens[(j * kDecoupledNmpcInputSize + r) * kDecoupledNmpcStateSize + q] =
                        p[q * kDecoupledNmpcInputSize + r];
                }
            }
        }
        for (unsigned j = 0; j < k; ++j) {
            for (unsigned r = 0; r < kDecoupledNmpcInputSize; ++r) {
                const unsigned col = j * kDecoupledNmpcInputSize + r;
                for (unsigned l = 0; l < k; ++l) {
                    for (unsigned t = 0; t < kDecoupledNmpcInputSize; ++t) {
                        const unsigned col2 = l * kDecoupledNmpcInputSize + t;
                        double value = 0.0;
                        for (unsigned q = 0; q < kDecoupledNmpcStateSize; ++q) {
                    value += sens[col * kDecoupledNmpcStateSize + q] * config.state_weight[q] *
                                     sens[col2 * kDecoupledNmpcStateSize + q];
                        }
                        qp.h[col * kDecoupledNmpcVariables + col2] += 2.0 * value;
                    }
                }
                double value = 0.0;
                for (unsigned q = 0; q < kDecoupledNmpcStateSize; ++q) {
                    value += sens[col * kDecoupledNmpcStateSize + q] * config.state_weight[q] * error[q];
                }
                qp.g[col] += 2.0 * value;
            }
        }
    }
    for (unsigned k = 0; k < kDecoupledNmpcHorizon; ++k) {
        for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
            const unsigned i = k * kDecoupledNmpcInputSize + j;
            qp.h[i * kDecoupledNmpcVariables + i] += 2.0 * config.input_weight[j];
            qp.a[i * kDecoupledNmpcVariables + i] = 1.0;
            qp.lower[i] = -std::abs(config.input_limit[j]);
            qp.upper[i] = std::abs(config.input_limit[j]);
        }
    }
    return finite(qp.h.data(), qp.h.size()) && finite(qp.g.data(), qp.g.size()) &&
           finite(qp.affine_defect.data(), qp.affine_defect.size());
}

void shift_decoupled_nmpc_warm_start(
    std::array<double, kDecoupledNmpcVariables>& destination,
    const std::array<double, kDecoupledNmpcVariables>& solution) noexcept {
    for (unsigned k = 0; k + 1 < kDecoupledNmpcHorizon; ++k) {
        for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
            destination[k * kDecoupledNmpcInputSize + j] =
                solution[(k + 1) * kDecoupledNmpcInputSize + j];
        }
    }
    for (unsigned j = 0; j < kDecoupledNmpcInputSize; ++j) {
        destination[(kDecoupledNmpcHorizon - 1) * kDecoupledNmpcInputSize + j] =
            solution[(kDecoupledNmpcHorizon - 1) * kDecoupledNmpcInputSize + j];
    }
}

} // namespace p112
