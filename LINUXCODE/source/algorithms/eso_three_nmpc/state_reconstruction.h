#ifndef P112_STATE_RECONSTRUCTION_H
#define P112_STATE_RECONSTRUCTION_H
#include "control_pipeline.h"
#include <array>
namespace p112 {
using State15 = std::array<double,15>;
struct ContactVelocityProjection { double row[2][7]{}; };
struct CadGeometryPacket {
    std::array<double,6> coordinates{}; // L,swing,wheel for left then right
    std::array<double,36> jacobian{};    // row-major virtual coordinates / motor
};
using GeometryCallback = int (*)(void*, const double*, CadGeometryPacket*) noexcept;

// Reconstructs the exact motor-coordinate MPC state layout used by
// GroundModel.sensor_state. The callback validates the current closed-chain
// branch and supplies virtual-leg diagnostics; the projection is frozen at
// initialization from the native contact map and contains no Python state.
class StateReconstructor {
public:
    StateReconstructor(void* geometry_context, GeometryCallback callback,
                       const ContactVelocityProjection& projection);
    bool reconstruct(const CanonicalSensor& canonical, const Vec6& attitude,
                     State15& state, CadGeometryPacket* packet = nullptr) const noexcept;
private:
    void* context_;
    GeometryCallback callback_;
    ContactVelocityProjection projection_;
};
}
#endif
