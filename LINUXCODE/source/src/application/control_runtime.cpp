#include "control_stack/application/control_runtime.hpp"
#include "control_stack/platform/amp/amp_protocol.hpp"
#include <cmath>

namespace control_stack {
namespace {
constexpr std::uint64_t kCommandTimeoutUs = 50000;
constexpr std::uint64_t kSensorMaxAgeUs = 5000;
constexpr std::uint64_t kRecoveryTimeoutUs = 10000000;
constexpr std::uint8_t kCommandType = 0x01;
}

bool ControlRuntime::isFresh(std::uint16_t seq, std::uint16_t previous, bool initialized) {
  return !initialized || static_cast<std::int16_t>(seq - previous) > 0;
}

AmpDecodeResult ControlRuntime::ingestAmpBytes(const std::uint8_t* bytes,
                                                std::size_t size,
                                                std::uint64_t received_us) {
  AmpFrame frame{};
  std::size_t consumed = 0;
  const auto result = AmpFrameCodec::decode(bytes, size, frame, consumed);
  if (result != AmpDecodeResult::Ok) return result;
  if (frame.type == static_cast<std::uint8_t>(AmpMessageType::SafetySnapshot)) {
    if (!isFresh(frame.sequence, last_safety_sequence_, have_safety_sequence_))
      return AmpDecodeResult::StaleSequence;
    AmpSafetySnapshot decoded{};
    const auto safety_result = AmpFrameCodec::decodeSafetySnapshot(frame, decoded);
    if (safety_result != AmpDecodeResult::Ok) return safety_result;
    rt_safety_ = decoded;
    last_safety_sequence_ = frame.sequence;
    have_safety_sequence_ = true;
    have_safety_us_ = received_us;
    SensorFrame partial{};
    partial.timestamp_us = received_us;
    partial.state[0] = decoded.pitch_rad;
    partial.state[2] = decoded.roll_rad;
    partial.attitude_valid = decoded.attitude_valid;
    partial.imu_valid = decoded.specific_force_valid;
    partial.specific_force = decoded.specific_force_m_s2;
    partial.fallen = decoded.fallen;
    partial.body_parallel = decoded.body_parallel;
    perception_ = app_.perceive(partial);
    perception_.attitude_valid = decoded.attitude_valid;
    perception_.fallen = perception_.fallen || decoded.fallen;
    perception_.body_parallel = perception_.body_parallel || decoded.body_parallel;
    fall_recovery_latched_ = fall_recovery_latched_ || decoded.fallen;
    return AmpDecodeResult::Ok;
  }
  if (frame.type == kCommandType) {
    if (!isFresh(frame.sequence, last_command_sequence_, have_command_sequence_))
      return AmpDecodeResult::StaleSequence;
    AmpCommand decoded{};
    const auto command_result = AmpFrameCodec::decodeCommand(frame, decoded);
    if (command_result != AmpDecodeResult::Ok) return command_result;
    command_ = decoded.command;
    last_command_sequence_ = decoded.sequence;
    have_command_sequence_ = true;
    status_.last_command_us = received_us;
    return AmpDecodeResult::Ok;
  }
  if (frame.type == static_cast<std::uint8_t>(AmpMessageType::SensorSnapshot)) {
    if (!isFresh(frame.sequence, last_sensor_sequence_, have_sensor_sequence_))
      return AmpDecodeResult::StaleSequence;
    SensorFrame decoded{};
    const auto sensor_result = AmpFrameCodec::decodeSensorSnapshot(frame, decoded);
    if (sensor_result != AmpDecodeResult::Ok) return sensor_result;
    decoded.timestamp_us = received_us;
    const bool safety_fresh = have_safety_sequence_ &&
                              received_us >= have_safety_us_ &&
                              received_us - have_safety_us_ <= kSensorMaxAgeUs;
    if (safety_fresh) {
      if (rt_safety_.attitude_valid) {
        decoded.state[0] = rt_safety_.pitch_rad;
        decoded.state[2] = rt_safety_.roll_rad;
        decoded.attitude_valid = true;
      }
      if (rt_safety_.specific_force_valid) {
        decoded.specific_force = rt_safety_.specific_force_m_s2;
        decoded.imu_valid = true;
      }
    }
    perception_ = app_.perceive(decoded);
    if (safety_fresh && rt_safety_.attitude_valid) {
      perception_.attitude_valid = true;
      perception_.fallen = rt_safety_.fallen;
      perception_.body_parallel = rt_safety_.body_parallel;
    }
    perception_.fallen = perception_.fallen || decoded.fallen;
    perception_.body_parallel = perception_.body_parallel || decoded.body_parallel;
    perception_.legs_centered = perception_.legs_centered || decoded.legs_centered;
    fall_recovery_latched_ = fall_recovery_latched_ || perception_.fallen;
    decoded.fallen = perception_.fallen;
    decoded.body_parallel = perception_.body_parallel;
    decoded.legs_centered = perception_.legs_centered;
    decoded.wheel_slip = perception_.wheel_slip;
    decoded.wheel_torque_scale = perception_.wheel_torque_scale;
    sensors_ = decoded;
    last_sensor_sequence_ = frame.sequence;
    have_sensor_sequence_ = true;
    have_sensor_us_ = received_us;
    return AmpDecodeResult::Ok;
  }
  return AmpDecodeResult::BadType;
}

void ControlRuntime::updateRecovery(const SensorFrame& s, Command& c) {
  if (status_.state != RobotState::Recovery) {
    status_.recovery_stage = RecoveryStage::Idle;
    recovery_start_us_ = 0;
    last_recovery_sensor_us_ = 0;
    return;
  }
  if (s.timestamp_us == last_recovery_sensor_us_) {
    c.recovery_stage = status_.recovery_stage;
    c.motion_state = status_.recovery_stage == RecoveryStage::BalanceHandoff ? 0U : 4U;
    return;
  }
  last_recovery_sensor_us_ = s.timestamp_us;
  if (recovery_start_us_ == 0) recovery_start_us_ = s.timestamp_us;
  if (s.timestamp_us - recovery_start_us_ > kRecoveryTimeoutUs) {
    status_.recovery_stage = RecoveryStage::Failed;
    status_.fault_code = 0x2001;
    status_.state = RobotState::Fault;
    fault_latched_ = true;
  } else if (status_.recovery_stage == RecoveryStage::Idle) {
    status_.recovery_stage = RecoveryStage::SwingLegsBackward;
  } else if (status_.recovery_stage == RecoveryStage::SwingLegsBackward && s.body_parallel) {
    status_.recovery_stage = RecoveryStage::MoveLegsToCenter;
  } else if (status_.recovery_stage == RecoveryStage::MoveLegsToCenter && s.legs_centered) {
    status_.recovery_stage = RecoveryStage::BalanceHandoff;
  } else if (status_.recovery_stage == RecoveryStage::BalanceHandoff && !s.fallen) {
    status_.state = RobotState::Standby;
    status_.recovery_stage = RecoveryStage::Idle;
    recovery_start_us_ = 0;
    fall_recovery_latched_ = false;
  }
  c.recovery_stage = status_.recovery_stage;
  c.motion_state = status_.recovery_stage == RecoveryStage::BalanceHandoff ? 0U : 4U;
}

void ControlRuntime::selectController() {
  ControlSelection next{};
  next.backend = ControlBackend::Mpc;
  next.profile = status_.state == RobotState::Recovery &&
                         status_.recovery_stage != RecoveryStage::BalanceHandoff
                     ? MpcProfile::Recovery
                     : (status_.state == RobotState::Locomotion ||
                        status_.state == RobotState::Spin ||
                        status_.state == RobotState::Step || status_.state == RobotState::Jump)
                           ? MpcProfile::Locomotion
                           : MpcProfile::Balance;
  if (!selection_initialized_ || next.profile != selection_.profile ||
      next.backend != selection_.backend) {
    selection_ = next;
    selection_initialized_ = true;
    app_.reset(selection_);
  }
}

ControlOutput ControlRuntime::tick(std::uint64_t now_us) {
  ControlOutput safe{};
  const auto& sensors = sensors_;
  if (fault_latched_) return safe;
  if (command_.vehicle_state == 2 || command_.vehicle_state == 3) {
    status_.state = RobotState::Fault;
    status_.fault_code = command_.vehicle_state == 2 ? 0x1001 : 0x1002;
    fault_latched_ = true;
    selectController();
    return safe;
  }
  if (!have_command_sequence_ || !have_sensor_sequence_ ||
      now_us < status_.last_command_us || now_us < have_sensor_us_ ||
      now_us - status_.last_command_us > kCommandTimeoutUs ||
      now_us - have_sensor_us_ > kSensorMaxAgeUs) {
    status_.state = RobotState::Disabled;
    status_.recovery_stage = RecoveryStage::Idle;
    selectController();
    return safe;
  }
  const double dt = last_tick_us_ != 0 && sensors.timestamp_us > last_tick_us_
                        ? double(sensors.timestamp_us - last_tick_us_) * 1e-6
                        : 0.0;
  last_tick_us_ = sensors.timestamp_us;
  status_.odom_x_m += (sensors.body_vx_m_s * std::cos(status_.odom_yaw_rad) -
                       sensors.body_vy_m_s * std::sin(status_.odom_yaw_rad)) * dt;
  status_.odom_y_m += (sensors.body_vx_m_s * std::sin(status_.odom_yaw_rad) +
                       sensors.body_vy_m_s * std::cos(status_.odom_yaw_rad)) * dt;
  status_.odom_yaw_rad = sensors.body_yaw_rad;
  if (command_.vehicle_state == 0) {
    status_.state = RobotState::Disabled;
    selectController();
    return safe;
  }
  if (!sensors.eso_valid) {
    status_.state = RobotState::Disabled;
    selectController();
    if (log_ != nullptr)
      log_->write({LogLevel::Warning, sensors.timestamp_us,
                   "eso_estimate_invalid", "control output inhibited"});
    return safe;
  }
  if (status_.state == RobotState::Recovery || fall_recovery_latched_ ||
      command_.motion_state == 4 || sensors.fallen)
    status_.state = RobotState::Recovery;
  else if (command_.motion_state == 2) status_.state = RobotState::Step;
  else if (command_.motion_state == 3) status_.state = RobotState::Jump;
  else if (std::abs(command_.z_speed_m_s) > 1.0)
    status_.state = RobotState::Spin;
  else if (std::abs(command_.x_speed_m_s) + std::abs(command_.y_speed_m_s) > 1e-3)
    status_.state = RobotState::Locomotion;
  else status_.state = RobotState::Standby;
  Command effective = command_;
  updateRecovery(sensors, effective);
  if (fault_latched_) return safe;
  selectController();
  return app_.step(sensors, effective, perception_);
}

void ControlRuntime::forceFault(std::uint32_t code, std::uint64_t now_us) {
  status_.state = RobotState::Fault;
  status_.fault_code = code;
  fault_latched_ = true;
  selectController();
  if (log_) log_->write({LogLevel::Error, now_us, "application_fault", "forced fault"});
}
void ControlRuntime::clearFault() {
  if (status_.state == RobotState::Fault) {
    status_.state = RobotState::Disabled;
    status_.fault_code = 0;
    fault_latched_ = false;
    selection_initialized_ = false;
  }
}
}  // namespace control_stack
