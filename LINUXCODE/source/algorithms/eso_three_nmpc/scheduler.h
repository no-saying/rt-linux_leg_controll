#ifndef P112_SCHEDULER_H
#define P112_SCHEDULER_H
#include "state_reconstruction.h"
namespace p112 {
enum class SchedulerMode : unsigned char { ground_mpc, nmpc, jump_rl, locomotion_rl, spin_rl, recovery_rl, hold };
struct SchedulerRequest {
    SchedulerMode mode{SchedulerMode::hold};
    double speed{};
    double yaw_rate{};
    double confidence{1.};
    double now{};
};
using PlanSolveCallback = int (*)(void*, const State15&, const Torque6&, double, double, double, double, Torque6&) noexcept;
using ActorCallback = int (*)(void*, SchedulerMode, const std::array<float,42>&, std::array<double,6>&) noexcept;
using ResidualAllocateCallback = int (*)(void*, SchedulerMode, const State15&, const Torque6&, const Torque6&, double, Torque6&) noexcept;
struct SchedulerOutput { Torque6 requested{}; SchedulerMode mode{SchedulerMode::hold}; bool from_actor{}; };

// Mode arbitration only: controller/model ownership remains in callbacks.
// The scheduler never claims a request was physically applied; call
// acknowledge() with the actuator's confirmed command on the next sensor tick.
class AlgorithmScheduler {
public:
    AlgorithmScheduler(void* plan_context, PlanSolveCallback plan,
                       void* actor_context, ActorCallback actor,
                       ResidualAllocateCallback residual = nullptr);
    void reset() noexcept;
    bool acknowledge(const Torque6& actual) noexcept;
    bool step(const SchedulerRequest& request, const State15& state,
              const std::array<float,42>& actor_observation, double dt,
              SchedulerOutput& output) noexcept;
    const Torque6& previous_actual() const noexcept { return previous_actual_; }
private:
    void* plan_context_; PlanSolveCallback plan_;
    void* actor_context_; ActorCallback actor_;
    ResidualAllocateCallback residual_;
    Torque6 previous_actual_{};
    double last_time_{}; bool has_time_{};
};
}
#endif
