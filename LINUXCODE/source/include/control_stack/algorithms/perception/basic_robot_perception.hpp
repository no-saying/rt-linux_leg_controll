#pragma once

#include "control_stack/algorithms/perception/perception_algorithm.hpp"

namespace control_stack {

struct BasicPerceptionConfig {
  double parallel_angle_rad{0.14};
  double fallen_angle_rad{0.70};
  double centered_swing_rad{0.12};
  double centered_leg_delta_rad{0.10};
  double wheel_slip_enter_m_s{0.25};
  double wheel_slip_exit_m_s{0.12};
  double wheel_slip_torque_scale{0.5};
  double slip_confirm_s{0.020};
  double support_unload_g{0.30};
  double support_contact_g{0.75};
  double support_unload_s{0.012};
  double support_recontact_s{0.006};
  double nominal_sample_s{0.002};
};

// Lightweight perception over the synchronized AMP snapshot. Attitude and
// kinematic state are supplied by RT-Thread, so this class does not duplicate
// the IMU/ESO estimator or require the unavailable CAD Jacobian.
class BasicRobotPerception final : public IPerceptionAlgorithm {
 public:
  explicit BasicRobotPerception(
      const BasicPerceptionConfig& config = {}) noexcept;
  void reset() override;
  PerceptionState update(const SensorFrame& sensors) override;

 private:
  BasicPerceptionConfig config_;
  std::uint64_t last_timestamp_us_{};
  bool have_timestamp_{};
  double slip_dwell_s_{};
  double unload_dwell_s_{};
  double recontact_dwell_s_{};
  SupportMode support_{SupportMode::Unknown};
  std::array<bool, 2> slip_{};
  PerceptionState last_output_{};
};

}  // namespace control_stack
