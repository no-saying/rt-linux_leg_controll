#ifndef P112_DECOUPLED_NMPC_H
#define P112_DECOUPLED_NMPC_H

#include "decoupled_control.h"
#include "serial_leg_dynamics.h"
#include <array>
#include <cstddef>

namespace p112 {

constexpr unsigned kDecoupledNmpcStateSize = 4;
constexpr unsigned kDecoupledNmpcInputSize = 2;
constexpr unsigned kDecoupledNmpcHorizon = 18;
constexpr unsigned kDecoupledNmpcVariables = kDecoupledNmpcHorizon * kDecoupledNmpcInputSize;
constexpr unsigned kDecoupledNmpcRows = kDecoupledNmpcVariables;

// Optimizer costs and actuator bounds are kept separate from the dynamics.
struct NmpcAxisConfig {
    SerialLegAxisDynamics dynamics{};
    std::array<double,4> state_weight{8., 1., 2., .2};
    std::array<double,2> input_weight{.08, .08};
    std::array<double,2> input_limit{8., 8.};
};

struct DecoupledNmpcConfig {
    std::array<NmpcAxisConfig,3> axis{};
    unsigned horizon{kDecoupledNmpcHorizon};
    unsigned qp_rows{kDecoupledNmpcRows};
    static DecoupledNmpcConfig defaults() noexcept;
};

struct DecoupledNmpcAxis {
    NmpcAxisConfig config{};
    std::array<double,36> warm_start{};
    void *qp_context{};
    double last_dt{};
    bool initialized{};
};

// Dense absolute-input QP assembled from one nonlinear RTI rollout.  The
// decision vector is [u_0, ..., u_17], in physical torque units.  The affine
// defect is retained for tests and diagnostics so a caller can verify that
// the local model reproduces the nonlinear nominal trajectory.
struct DecoupledNmpcQp {
    std::array<double, kDecoupledNmpcVariables * kDecoupledNmpcVariables> h{};
    std::array<double, kDecoupledNmpcVariables> g{};
    std::array<double, kDecoupledNmpcRows * kDecoupledNmpcVariables> a{};
    std::array<double, kDecoupledNmpcRows> lower{};
    std::array<double, kDecoupledNmpcRows> upper{};
    std::array<double, kDecoupledNmpcHorizon * kDecoupledNmpcStateSize> affine_defect{};
};

struct DecoupledNmpc {
    DecoupledNmpcConfig config{};
    std::array<DecoupledNmpcAxis,3> axis{};
    bool initialized{};
};

// Build the condensed local QP without touching an external solver.  The
// warm start supplies the nominal absolute torque sequence used by the
// nonlinear rollout; it is never interpreted as an input increment.
bool build_decoupled_nmpc_qp(
    const NmpcAxisConfig&, const AxisState4&, const AxisReference4&, double dt,
    double disturbance,
    const std::array<double, kDecoupledNmpcVariables>& warm_start,
    DecoupledNmpcQp&) noexcept;

void shift_decoupled_nmpc_warm_start(
    std::array<double, kDecoupledNmpcVariables>& destination,
    const std::array<double, kDecoupledNmpcVariables>& solution) noexcept;

// Initializes all three HPIPM contexts.  Must be called before the real-time
// loop and destroyed with destroy_decoupled_nmpc() after workers stop.
bool initialize_decoupled_nmpc(
    DecoupledNmpc&, const DecoupledNmpcConfig& = DecoupledNmpcConfig::defaults()) noexcept;
void destroy_decoupled_nmpc(DecoupledNmpc&) noexcept;

// Callback compatible with DecoupledController.  Each axis owns its own QP
// context and warm start, so the callback is safe for the three worker threads
// used by step_parallel().
int decoupled_nmpc_callback(void*, DecoupledAxis, const AxisState4&,
                           const AxisReference4&, double, double,
                           AxisTorque2&) noexcept;

} // namespace p112
#endif
