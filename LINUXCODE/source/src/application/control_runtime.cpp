#include "control_stack/application/control_runtime.hpp"
#include "control_stack/platform/amp/amp_protocol.hpp"
#include <cmath>

namespace control_stack {
namespace {
constexpr std::uint64_t kCommandTimeoutUs = 50000;
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
    return;
  }
  if (recovery_start_us_ == 0) recovery_start_us_ = s.timestamp_us;
  if (s.timestamp_us - recovery_start_us_ > kRecoveryTimeoutUs) {
    status_.recovery_stage = RecoveryStage::Failed;
    status_.fault_code = 0x2001;
    status_.state = RobotState::Fault;
    fault_latched_ = true;
  } else if (status_.recovery_stage == RecoveryStage::Idle) {
    status_.recovery_stage = RecoveryStage::SwingLegsBackward;
  } else if (status_.recovery_stage == RecoveryStage::SwingLegsBackward && s.body_parallel) {
    status_.recovery_stage = RecoveryStage::FoldLegs;
  } else if (status_.recovery_stage == RecoveryStage::FoldLegs && s.leg_height_m <= 0.08) {
    status_.recovery_stage = RecoveryStage::MoveLegsToCenter;
  } else if (status_.recovery_stage == RecoveryStage::MoveLegsToCenter && s.legs_centered) {
    status_.recovery_stage = RecoveryStage::BalanceHandoff;
  } else if (status_.recovery_stage == RecoveryStage::BalanceHandoff && !s.fallen) {
    status_.state = RobotState::Standby;
    status_.recovery_stage = RecoveryStage::Idle;
    recovery_start_us_ = 0;
  }
  c.recovery_stage = status_.recovery_stage;
  c.motion_state = status_.recovery_stage == RecoveryStage::BalanceHandoff ? 0U : 4U;
}

void ControlRuntime::selectController() {
  ControlSelection next{};
  next.backend = ControlBackend::Mpc;
  next.profile = status_.state == RobotState::Recovery
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

ControlOutput ControlRuntime::tick() {
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
      sensors.timestamp_us < status_.last_command_us ||
      sensors.timestamp_us - status_.last_command_us > kCommandTimeoutUs ||
      sensors.timestamp_us < have_sensor_us_ ||
      sensors.timestamp_us - have_sensor_us_ > kCommandTimeoutUs) {
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
  if (command_.motion_state == 4 || sensors.fallen) status_.state = RobotState::Recovery;
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
  return app_.step(sensors, effective);
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
