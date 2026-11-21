#pragma once

#include <cstdint>

namespace control_stack {

// Wire values are binary16 bit patterns. Conversion is a BSP/codec concern;
// control algorithms consume float/double values after validation.
struct AmpCommandWire {
  std::uint16_t x_speed_f16{};
  std::uint16_t y_speed_f16{};
  std::uint16_t z_speed_f16{};
  std::uint16_t leg_height_f16{};
  std::uint32_t vehicle_state{};
  std::uint32_t motion_state{};
};

static_assert(sizeof(AmpCommandWire) == 16, "AMP command payload must be 16 bytes");

enum class AmpMessageType : std::uint8_t {
  CommandSetpoint = 0x01,
  CommandAck = 0x02,
  TelemetryState = 0x10,
  TelemetryActuator = 0x11,
  SensorSnapshot = 0x12,
  SafetySnapshot = 0x13,
  FaultEvent = 0x20,
  Heartbeat = 0x7f,
};

enum AmpSensorFlag : std::uint16_t {
  SensorBodyParallel = 1U << 0,
  SensorLegsCentered = 1U << 1,
  SensorFallen = 1U << 2,
  SensorEsoValid = 1U << 3,
  SensorAttitudeValid = 1U << 4,
  SensorImuValid = 1U << 5,
  SensorKinematicStateValid = 1U << 6,
  SensorJointStateValid = 1U << 7,
  SensorWheelSpeedValid = 1U << 8,
  SensorBodyVelocityValid = 1U << 9,
};

}
