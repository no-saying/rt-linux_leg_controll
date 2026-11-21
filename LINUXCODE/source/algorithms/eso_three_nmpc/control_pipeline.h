#ifndef P112_CONTROL_PIPELINE_H
#define P112_CONTROL_PIPELINE_H

#include "estimation.h"
#include <array>
#include <cstddef>

namespace p112 {

using Torque6 = std::array<double, 6>;
using CanonicalSensor = std::array<double, 18>;

struct SensorPacket {
    Vec3 gyro{};
    Vec3 specific_force{};
    Torque6 motor_position{};
    Torque6 motor_velocity{};
    double dt{.001};
};

// Runtime equivalent of runtime.sensors.GroundSensors. CAD state assembly is
// intentionally supplied by the native model; this class owns only the
// synchronized IMU/encoder preprocessing and yaw reference.
class GroundSensorPipeline {
public:
    explicit GroundSensorPipeline(std::size_t alignment_samples = 1000);
    void reset() noexcept;
    bool update(const SensorPacket& packet, CanonicalSensor& canonical, Vec6& attitude) noexcept;
    bool ready() const noexcept { return ready_; }
    const Vec3& filtered_specific_force() const noexcept { return filtered_force_; }
    const Torque6& motor_position() const noexcept { return motor_; }
    const Torque6& motor_velocity() const noexcept { return rate_; }

private:
    MotionImu imu_;
    bool ready_{};
    double yaw_zero_{};
    Vec3 filtered_force_{};
    Torque6 motor_{};
    Torque6 rate_{};
};

struct JumpSupportEstimate {
    enum class Mode : unsigned char { ground, unloading, flight, recover };
    Mode mode{Mode::ground};
    bool likely_airborne{};
    bool use_floating_model{};
    bool recontact_hypothesis{};
    double confidence{1.};
    double filtered_force_m_s2{9.81};
    unsigned transitions{};
};

// CAD centroidal velocity transfer plus the hysteretic support hypothesis used
// by the jump actor. The caller supplies the current COM-site offset and its
// 3x6 Jacobian, keeping CAD ownership in the native geometry backend.
class CentroidalSuspensionObserver {
public:
    CentroidalSuspensionObserver() noexcept { reset(); }
    void reset() noexcept;
    bool update(const Vec3& specific_force, const Vec3& gyro,
                const Vec3& com_site_offset, const std::array<double,18>& jacobian,
                const Torque6& motor_velocity, double dt,
                JumpSupportEstimate& estimate) noexcept;
    const Vec3& filtered_force() const noexcept { return force_; }
    double low_dwell() const noexcept { return low_; }
    double high_dwell() const noexcept { return high_; }
    double mode_age() const noexcept { return age_; }
    JumpSupportEstimate::Mode mode() const noexcept { return mode_; }

private:
    Vec3 force_{}, previous_relative_velocity_{};
    bool has_previous_{};
    Vec3 com_site_offset_{};
    double low_{}, high_{}, age_{};
    unsigned transitions_{};
    double confidence_{1.};
    JumpSupportEstimate::Mode mode_{JumpSupportEstimate::Mode::ground};
};

// Direct jump actor command protection. `action` is normalized [-1,1] total
// torque; previous is the actually acknowledged actuator command.
bool protected_motor_torque(const Torque6& action, const Torque6& previous,
                            double dt, Torque6& output) noexcept;

double skill_gate(double forward_m_s, double yaw_rate_rad_s, unsigned skill) noexcept;
// skill: 0 joint, 1 locomotion, 2 spin, 3 step_down.
double command_profile(double t_s, double target, double duration_s) noexcept;

// Residual policies provide normalized additive torque, allocated by the
// caller's CAD wrench adapter. This function preserves the Python contract's
// exact-zero nominal path and physical torque/rate bounds.
bool mix_residual(const Torque6& nominal, const Torque6& previous,
                  const Torque6& normalized_residual, double dt,
                  Torque6& output) noexcept;

} // namespace p112
#endif
