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
  FaultEvent = 0x20,
  Heartbeat = 0x7f,
};

}
