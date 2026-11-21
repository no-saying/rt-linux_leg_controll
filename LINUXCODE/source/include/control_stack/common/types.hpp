#pragma once

#include <array>
#include <cstdint>

namespace control_stack {

using TorqueVector = std::array<double, 6>;
using JointVector = std::array<double, 6>;
using AxisVector = std::array<double, 3>;

enum class RecoveryStage : std::uint8_t {
  Idle, SwingLegsBackward, MoveLegsToCenter, BalanceHandoff, Failed
};

enum class SupportMode : std::uint8_t {
  Unknown, Ground, Unloading, Airborne, Recontact
};

struct SensorFrame {
  std::uint64_t timestamp_us{0};
  bool attitude_valid{false};
  bool imu_valid{false};
  bool kinematic_state_valid{false};
  bool joint_state_valid{false};
  bool wheel_speed_valid{false};
  bool body_velocity_valid{false};
  std::array<double, 15> state{};
  AxisVector gyro{};
  AxisVector specific_force{};
  JointVector joint_position{};
  JointVector joint_velocity{};
  double body_vx_m_s{0.0};
  double body_vy_m_s{0.0};
  double body_yaw_rad{0.0};
  double leg_height_m{0.0};
  bool body_parallel{false};
  bool legs_centered{false};
  bool fallen{false};
  bool support_valid{false};
  SupportMode support{SupportMode::Unknown};
  double support_confidence{0.0};
  AxisVector eso_disturbance{};
  bool eso_valid{false};
  std::array<bool, 2> wheel_slip{};
  std::array<double, 2> wheel_torque_scale{{1.0, 1.0}};
};

struct Command {
  double x_speed_m_s{0.0};
  double y_speed_m_s{0.0};
  double z_speed_m_s{0.0};
  double leg_height_m{0.0};
  std::uint32_t vehicle_state{0};
  std::uint32_t motion_state{0};
  RecoveryStage recovery_stage{};
};

struct PerceptionState {
  double confidence{1.0};
  std::array<bool, 2> wheel_slip{};
  std::array<double, 2> wheel_torque_scale{{1.0, 1.0}};
  bool attitude_valid{false};
  bool fallen{false};
  bool body_parallel{false};
  bool legs_centered{false};
  bool support_valid{false};
  SupportMode support{SupportMode::Unknown};
  double support_confidence{0.0};
  double wheel_speed_residual_m_s{0.0};
};

struct GreenWavePlan {
  Command command{};
  std::uint32_t phase{0};
  bool valid{false};
};

struct ControlOutput {
  TorqueVector torque{};
  std::uint32_t mode{0};
  bool valid{false};
  double confidence{0.0};
};

struct AmpCommand {
  Command command{};
  std::uint16_t sequence{0};
  std::uint32_t timestamp_ms{0};
  bool valid{false};
};

enum class RobotState : std::uint8_t {
  Disabled = 0,
  Standby = 1,
  Locomotion = 2,
  Spin = 3,
  Step = 4,
  Jump = 5,
  Recovery = 6,
  Fault = 7,
};

struct ApplicationStatus {
  RobotState state{RobotState::Disabled};
  RecoveryStage recovery_stage{RecoveryStage::Idle};
  double odom_x_m{0.0};
  double odom_y_m{0.0};
  double odom_yaw_rad{0.0};
  std::uint64_t last_command_us{0};
  std::uint32_t fault_code{0};
};

struct FaultEvent {
  std::uint64_t timestamp_us{0};
  std::uint32_t code{0};
  std::uint8_t severity{0};
};

}
