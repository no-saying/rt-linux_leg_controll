#ifndef P112_DECOUPLED_CONTROL_H
#define P112_DECOUPLED_CONTROL_H

#include "axis_types.h"
#include "scheduler.h"
#include <atomic>
#include <array>
#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace p112 {

struct EsoConfig {
    std::array<double,3> bandwidth{35.,35.,25.};
    std::array<double,3> disturbance_limit{30.,30.,20.};
    double nonlinear_alpha{.5};
    double nonlinear_delta{.01};
};
struct EsoMeasurement {
    Vec3 angle{}; // roll, pitch, yaw [rad]
    Vec3 rate{};  // roll, pitch, yaw [rad/s]
    Vec3 nominal_acceleration{}; // model input equivalent [rad/s^2]
};
struct EsoState {
    Vec3 angle{};
    Vec3 rate{};
    Vec3 disturbance{}; // lumped acceleration disturbance per axis
};

// RT-Thread handoff reference: run this observer on the RT-Thread side and
// transmit its disturbance estimate with the AMP sensor snapshot. The control
// framework consumes that estimate and must not run a second ESO instance.
// One global 9-state nonlinear ESO. It estimates all three axes together;
// the axis equations are numerically independent, while their measurements
// already contain the full coupled robot dynamics.
class GlobalEso {
public:
    explicit GlobalEso(const EsoConfig& config = {});
    bool configure(const EsoConfig& config) noexcept;
    void reset(const Vec3& angle = {}, const Vec3& rate = {}) noexcept;
    bool update(const EsoMeasurement& measurement, double dt, EsoState& output) noexcept;
    const EsoState& state() const noexcept { return state_; }
private:
    EsoConfig config_{};
    EsoState state_{};
    bool initialized_{};
};

struct AxisReference4 { std::array<double,4> value{}; };
struct AxisBlockConfig {
    std::array<unsigned,2> motor_index{};
    std::array<double,2> input_gain{1.,1.};
    std::array<double,2> torque_limit{8.,8.};
    std::array<double,2> torque_rate{250.,250.};
};
struct DecoupledConfig {
    EsoConfig eso{};
    std::array<AxisBlockConfig,3> axis{};
    static DecoupledConfig defaults() noexcept;
};
using BlockNmpcCallback = int (*)(void*, DecoupledAxis, const AxisState4&,
                                  const AxisReference4&, double, double,
                                  AxisTorque2&) noexcept;
struct DecoupledInput {
    std::uint64_t sequence{}; // one sensor/ESO/control snapshot identifier
    EsoState observed_eso{};   // RT-Thread estimate received with this AMP sample
    std::array<AxisState4,3> axis_state{};
    std::array<AxisReference4,3> reference{};
    double dt{};                 // ESO/sensor sample period
    double nmpc_dt{};            // NMPC period; zero means dt
    // ESO samples arrive at 500 Hz while NMPC runs at 100 Hz. The caller sets
    // this false on the four intermediate samples and republishes the last
    // acknowledged torque; ESO itself runs outside this controller.
    bool run_nmpc{true};
};
struct DecoupledOutput {
    std::uint64_t sequence{};
    EsoState eso{};
    std::array<AxisTorque2,3> axis_torque{};
    Torque6 torque{};
};

// Three-axis nonlinear NMPC orchestration. RT-Thread owns GlobalEso and passes
// its synchronized state here; each NMPC callback uses the disturbance during
// its prediction rollout. This class does not update ESO or add post-solve ESO
// torque compensation.
class DecoupledController {
public:
    DecoupledController(void* context, BlockNmpcCallback callback,
                        const DecoupledConfig& config = DecoupledConfig::defaults());
    ~DecoupledController();
    DecoupledController(const DecoupledController&) = delete;
    DecoupledController& operator=(const DecoupledController&) = delete;
    bool configure(const DecoupledConfig& config) noexcept;
    void reset() noexcept;
    bool step(const DecoupledInput& input, DecoupledOutput& output) noexcept;
    // Creates three permanent workers. Call once during initialization, before
    // entering the real-time loop. The callback must own independent state per
    // axis; step_parallel() never calls one axis concurrently with itself.
    bool start_parallel(const std::array<int,3>& cpu_affinity = {-1,-1,-1}) noexcept;
    bool step_parallel(const DecoupledInput& input, DecoupledOutput& output) noexcept;
    bool parallel_started() const noexcept { return parallel_started_; }
    bool parallel_affinity_ok() const noexcept { return parallel_affinity_ok_.load(std::memory_order_acquire); }
    const DecoupledConfig& config() const noexcept { return config_; }
    const Torque6& previous_actual() const noexcept { return previous_actual_; }
    bool acknowledge(const Torque6& actual) noexcept;
private:
    struct ParallelJob {
        std::array<AxisState4,3> axis_state{};
        std::array<AxisReference4,3> reference{};
        std::array<double,3> disturbance{};
        double dt{};
        std::uint64_t sequence{};
        std::array<AxisTorque2,3> output{};
        std::array<int,3> status{};
    };
    void worker_loop(unsigned axis) noexcept;
    bool finish_parallel(const DecoupledInput& input, const EsoState& estimate,
                         DecoupledOutput& output) noexcept;
    void* context_{};
    BlockNmpcCallback callback_{};
    DecoupledConfig config_{};
    Torque6 previous_actual_{};
    bool initialized_{};
    ParallelJob parallel_job_{};
    std::array<std::thread,3> workers_{};
    std::atomic<std::uint64_t> parallel_generation_{0};
    std::atomic<unsigned> parallel_completed_{0};
    std::atomic<unsigned> parallel_ready_{0};
    std::atomic<bool> parallel_affinity_ok_{true};
    std::atomic<bool> parallel_stop_{false};
    std::array<int,3> cpu_affinity_{{-1,-1,-1}};
    std::mutex parallel_mutex_;
    std::condition_variable parallel_cv_;
    std::mutex parallel_start_mutex_;
    std::condition_variable parallel_start_cv_;
    bool parallel_started_{};
};
}
#endif
