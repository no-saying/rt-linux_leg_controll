#include "decoupled_nmpc.h"
#include "acados_adapter.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace p112 {
namespace {
constexpr unsigned NX = kDecoupledNmpcStateSize;
constexpr unsigned NU = kDecoupledNmpcInputSize;
constexpr unsigned NV = kDecoupledNmpcVariables;
constexpr unsigned ROWS = kDecoupledNmpcRows;

bool finite(const double *v, std::size_t n) noexcept {
    for (std::size_t i=0; i<n; ++i) if (!std::isfinite(v[i])) return false;
    return true;
}

bool valid_model(const NmpcAxisConfig& m) noexcept {
    return finite(m.dynamics.vector_field.data(), m.dynamics.vector_field.size()) &&
           finite(m.dynamics.input_gain.data(), m.dynamics.input_gain.size()) &&
           finite(m.state_weight.data(), m.state_weight.size()) &&
           finite(m.input_weight.data(), m.input_weight.size()) &&
           finite(m.input_limit.data(), m.input_limit.size()) &&
           std::all_of(m.state_weight.begin(), m.state_weight.end(),
                       [](double value) { return value >= 0.0; }) &&
           std::all_of(m.input_weight.begin(), m.input_weight.end(),
                       [](double value) { return value > 0.0; }) &&
           std::all_of(m.input_limit.begin(), m.input_limit.end(),
                       [](double value) { return value > 0.0; });
}

int solve_axis(DecoupledNmpcAxis& s, const AxisState4& state,
               const AxisReference4& reference, double dt,
               double disturbance,
               AxisTorque2& out) noexcept {
    if (!std::isfinite(dt) || dt<=0. || dt>.02 || !finite(state.value.data(),NX) ||
        !finite(reference.value.data(),NX)) return -1;
    if (!s.initialized) { for(double &v:s.warm_start) v=0.; s.initialized=true; }
    DecoupledNmpcQp qp{};
    if (!build_decoupled_nmpc_qp(s.config, state, reference, dt, disturbance, s.warm_start, qp)) return -1;
    double x[NV]{};
    int status=nmpc_acados_solve(s.qp_context,ROWS,qp.h.data(),qp.g.data(),qp.a.data(),
                                 qp.lower.data(),qp.upper.data(),x);
    if(status!=0 || !finite(x,NV)) return status ? status : -2;
    std::array<double,NV> solution{};
    std::copy_n(x, NV, solution.begin());
    shift_decoupled_nmpc_warm_start(s.warm_start, solution);
    out.value[0]=x[0]; out.value[1]=x[1]; s.last_dt=dt;
    return finite(out.value.data(),NU)?0:-3;
}
}

DecoupledNmpcConfig DecoupledNmpcConfig::defaults() noexcept {
    DecoupledNmpcConfig c{};
    for (unsigned axis=0;axis<3;++axis) {
        auto& config = c.axis[axis];
        config.dynamics = serial_leg_axis_dynamics_defaults(
            static_cast<DecoupledAxis>(axis));
        if (axis == 2) config.input_limit = {3., 3.};
    }
    return c;
}

bool initialize_decoupled_nmpc(DecoupledNmpc& n, const DecoupledNmpcConfig& c) noexcept {
    if(c.horizon!=18 || c.qp_rows!=ROWS) return false;
    for (const auto& model : c.axis) if (!valid_model(model)) return false;
    n={}; n.config=c;
    for(unsigned i=0;i<3;++i) {
        n.axis[i].config=c.axis[i];
        n.axis[i].qp_context=nmpc_acados_create(ROWS);
        if(!n.axis[i].qp_context) { destroy_decoupled_nmpc(n); return false; }
    }
    n.initialized=true; return true;
}

void destroy_decoupled_nmpc(DecoupledNmpc& n) noexcept {
    for(auto &a:n.axis) { if(a.qp_context) nmpc_acados_destroy(a.qp_context); a.qp_context=nullptr; }
    n.initialized=false;
}

int decoupled_nmpc_callback(void *ctx, DecoupledAxis axis, const AxisState4& state,
                           const AxisReference4& reference, double disturbance,
                           double dt, AxisTorque2& out) noexcept {
    auto *n=static_cast<DecoupledNmpc *>(ctx);
    if(!n || !n->initialized || static_cast<unsigned>(axis)>=3 || !std::isfinite(disturbance)) return -1;
    int status=solve_axis(n->axis[static_cast<unsigned>(axis)],state,reference,dt,disturbance,out);
    return status;
}
}
