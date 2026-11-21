#include "control_stack/platform/amp/amp_frame.hpp"

#include <cmath>

namespace control_stack {
namespace {

constexpr std::uint8_t kSof0 = 0xA5;
constexpr std::uint8_t kSof1 = 0x5A;
constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kCommand = 0x01;
constexpr std::uint8_t kSensorSnapshot = 0x12;
constexpr std::uint16_t kCommandPayloadSize = 16;
constexpr std::uint16_t kSensorPayloadSize = 82;

void put16(std::uint8_t* p, std::uint16_t value) {
  p[0] = static_cast<std::uint8_t>(value & 0xffU);
  p[1] = static_cast<std::uint8_t>(value >> 8U);
}

void put32(std::uint8_t* p, std::uint32_t value) {
  for (int i = 0; i < 4; ++i)
    p[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xffU);
}

std::uint16_t get16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0]) |
         (static_cast<std::uint16_t>(p[1]) << 8U);
}

std::uint32_t get32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8U) |
         (static_cast<std::uint32_t>(p[2]) << 16U) |
         (static_cast<std::uint32_t>(p[3]) << 24U);
}

std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1U);
  }
  return crc;
}

std::uint16_t toHalf(float value) {
  std::uint32_t bits;
  __builtin_memcpy(&bits, &value, sizeof(bits));
  const std::uint32_t sign = (bits >> 16U) & 0x8000U;
  const std::uint32_t exponent = (bits >> 23U) & 0xffU;
  const std::uint32_t mantissa = bits & 0x7fffffU;
  if (exponent == 0xffU)
    return static_cast<std::uint16_t>(sign | 0x7c00U | (mantissa ? 0x0200U : 0U));

  const int half_exponent = static_cast<int>(exponent) - 127 + 15;
  if (half_exponent >= 31) return static_cast<std::uint16_t>(sign | 0x7c00U);
  if (half_exponent <= 0) {
    if (half_exponent < -10) return static_cast<std::uint16_t>(sign);
    const std::uint32_t shifted = (mantissa | 0x800000U) >> (1 - half_exponent);
    return static_cast<std::uint16_t>(sign | ((shifted + 0x1000U) >> 13U));
  }
  return static_cast<std::uint16_t>(sign | (static_cast<std::uint32_t>(half_exponent) << 10U) |
                                    ((mantissa + 0x1000U) >> 13U));
}

float fromHalf(std::uint16_t half) {
  const std::uint32_t sign = static_cast<std::uint32_t>(half & 0x8000U) << 16U;
  const std::uint32_t exponent = (half >> 10U) & 0x1fU;
  const std::uint32_t mantissa = half & 0x03ffU;
  std::uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;
    } else {
      std::uint32_t normalized = mantissa;
      int adjusted_exponent = -14;
      while ((normalized & 0x0400U) == 0) {
        normalized <<= 1U;
        --adjusted_exponent;
      }
      bits = sign | (static_cast<std::uint32_t>(adjusted_exponent + 127) << 23U) |
             ((normalized & 0x03ffU) << 13U);
    }
  } else if (exponent == 0x1fU) {
    bits = sign | 0x7f800000U | (mantissa << 13U);
  } else {
    bits = sign | ((exponent - 15U + 127U) << 23U) | (mantissa << 13U);
  }
  float value;
  __builtin_memcpy(&value, &bits, sizeof(value));
  return value;
}

bool putHalf(std::array<std::uint8_t, kAmpMaxPayload>& payload,
             std::size_t& offset, double value) {
  if (!std::isfinite(value) || std::abs(value) > 65504.0) return false;
  put16(payload.data() + offset, toHalf(static_cast<float>(value)));
  offset += 2;
  return true;
}

}  // namespace

bool AmpFrameCodec::encode(const AmpFrame& frame, std::uint8_t* out,
                           std::size_t capacity, std::size_t& written) {
  written = 0;
  if (out == nullptr || frame.payload_size > kAmpMaxPayload ||
      capacity < kAmpFrameOverhead + frame.payload_size)
    return false;

  out[0] = kSof0;
  out[1] = kSof1;
  out[2] = kVersion;
  out[3] = frame.type;
  put16(out + 4, frame.flags);
  put16(out + 6, frame.payload_size);
  put16(out + 8, frame.sequence);
  put32(out + 10, frame.timestamp_ms);
  for (std::size_t i = 0; i < frame.payload_size; ++i) out[14 + i] = frame.payload[i];
  put16(out + 14 + frame.payload_size, crc16(out + 2, 12 + frame.payload_size));
  written = 16 + frame.payload_size;
  return true;
}

AmpDecodeResult AmpFrameCodec::decode(const std::uint8_t* bytes, std::size_t size,
                                      AmpFrame& frame, std::size_t& consumed) {
  consumed = 0;
  if (bytes == nullptr || size < 2) return AmpDecodeResult::Incomplete;
  if (bytes[0] != kSof0 || bytes[1] != kSof1) {
    consumed = 1;
    return AmpDecodeResult::BadSync;
  }
  if (size < 14) return AmpDecodeResult::Incomplete;
  if (bytes[2] != kVersion) {
    consumed = 1;
    return AmpDecodeResult::BadVersion;
  }

  const auto payload_size = get16(bytes + 6);
  if (payload_size > kAmpMaxPayload) {
    consumed = 1;
    return AmpDecodeResult::BadLength;
  }
  const std::size_t frame_size = kAmpFrameOverhead + payload_size;
  if (size < frame_size) return AmpDecodeResult::Incomplete;
  if (get16(bytes + 14 + payload_size) != crc16(bytes + 2, 12 + payload_size)) {
    consumed = frame_size;
    return AmpDecodeResult::BadCrc;
  }

  frame.type = bytes[3];
  frame.flags = get16(bytes + 4);
  frame.payload_size = payload_size;
  frame.sequence = get16(bytes + 8);
  frame.timestamp_ms = get32(bytes + 10);
  for (std::size_t i = 0; i < payload_size; ++i) frame.payload[i] = bytes[14 + i];
  consumed = frame_size;
  return AmpDecodeResult::Ok;
}

bool AmpFrameCodec::encodeCommand(const Command& command, std::uint16_t sequence,
                                 std::uint32_t timestamp_ms, std::uint8_t* out,
                                 std::size_t capacity, std::size_t& written) {
  const double values[] = {command.x_speed_m_s, command.y_speed_m_s,
                           command.z_speed_m_s, command.leg_height_m};
  AmpFrame frame{};
  frame.type = kCommand;
  frame.sequence = sequence;
  frame.timestamp_ms = timestamp_ms;
  frame.payload_size = kCommandPayloadSize;
  std::size_t offset = 0;
  for (double value : values)
    if (!putHalf(frame.payload, offset, value)) return false;
  put32(frame.payload.data() + 8, command.vehicle_state);
  put32(frame.payload.data() + 12, command.motion_state);
  return encode(frame, out, capacity, written);
}

AmpDecodeResult AmpFrameCodec::decodeCommand(const AmpFrame& frame, AmpCommand& out) {
  if (frame.type != kCommand) return AmpDecodeResult::BadType;
  if (frame.flags != 0 || frame.payload_size != kCommandPayloadSize)
    return AmpDecodeResult::BadPayload;

  const float values[] = {fromHalf(get16(frame.payload.data())),
                          fromHalf(get16(frame.payload.data() + 2)),
                          fromHalf(get16(frame.payload.data() + 4)),
                          fromHalf(get16(frame.payload.data() + 6))};
  for (float value : values)
    if (!std::isfinite(value)) return AmpDecodeResult::BadPayload;
  const auto vehicle_state = get32(frame.payload.data() + 8);
  const auto motion_state = get32(frame.payload.data() + 12);
  if (vehicle_state > 3 || motion_state > 5) return AmpDecodeResult::BadPayload;

  out.command.x_speed_m_s = values[0];
  out.command.y_speed_m_s = values[1];
  out.command.z_speed_m_s = values[2];
  out.command.leg_height_m = values[3];
  out.command.vehicle_state = vehicle_state;
  out.command.motion_state = motion_state;
  out.sequence = frame.sequence;
  out.timestamp_ms = frame.timestamp_ms;
  out.valid = true;
  return AmpDecodeResult::Ok;
}

bool AmpFrameCodec::encodeSensorSnapshot(const SensorFrame& sensor,
                                         std::uint16_t sequence,
                                         std::uint32_t timestamp_ms,
                                         std::uint8_t* out,
                                         std::size_t capacity,
                                         std::size_t& written) {
  AmpFrame frame{};
  frame.type = kSensorSnapshot;
  frame.sequence = sequence;
  frame.timestamp_ms = timestamp_ms;
  frame.payload_size = kSensorPayloadSize;
  std::size_t offset = 0;
  for (double value : sensor.state)
    if (!putHalf(frame.payload, offset, value)) return false;
  for (double value : sensor.gyro)
    if (!putHalf(frame.payload, offset, value)) return false;
  for (double value : sensor.specific_force)
    if (!putHalf(frame.payload, offset, value)) return false;
  for (double value : sensor.joint_position)
    if (!putHalf(frame.payload, offset, value)) return false;
  for (double value : sensor.joint_velocity)
    if (!putHalf(frame.payload, offset, value)) return false;
  if (!putHalf(frame.payload, offset, sensor.body_vx_m_s) ||
      !putHalf(frame.payload, offset, sensor.body_vy_m_s) ||
      !putHalf(frame.payload, offset, sensor.body_yaw_rad) ||
      !putHalf(frame.payload, offset, sensor.leg_height_m))
    return false;
  for (double value : sensor.eso_disturbance)
    if (!putHalf(frame.payload, offset, value)) return false;

  const std::uint16_t flags = (sensor.body_parallel ? 1U : 0U) |
                              (sensor.legs_centered ? 2U : 0U) |
                              (sensor.fallen ? 4U : 0U) |
                              (sensor.eso_valid ? 8U : 0U);
  put16(frame.payload.data() + offset, flags);
  return encode(frame, out, capacity, written);
}

AmpDecodeResult AmpFrameCodec::decodeSensorSnapshot(const AmpFrame& frame,
                                                    SensorFrame& out) {
  if (frame.type != kSensorSnapshot) return AmpDecodeResult::BadType;
  if (frame.flags != 0 || frame.payload_size != kSensorPayloadSize)
    return AmpDecodeResult::BadPayload;

  SensorFrame decoded{};
  std::size_t offset = 0;
  auto read_half = [&]() {
    const float value = fromHalf(get16(frame.payload.data() + offset));
    offset += 2;
    return static_cast<double>(value);
  };
  for (double& value : decoded.state) value = read_half();
  for (double& value : decoded.gyro) value = read_half();
  for (double& value : decoded.specific_force) value = read_half();
  for (double& value : decoded.joint_position) value = read_half();
  for (double& value : decoded.joint_velocity) value = read_half();
  decoded.body_vx_m_s = read_half();
  decoded.body_vy_m_s = read_half();
  decoded.body_yaw_rad = read_half();
  decoded.leg_height_m = read_half();
  for (double& value : decoded.eso_disturbance) value = read_half();

  const auto flags = get16(frame.payload.data() + offset);
  if (flags & ~std::uint16_t(15)) return AmpDecodeResult::BadPayload;
  decoded.body_parallel = (flags & 1U) != 0;
  decoded.legs_centered = (flags & 2U) != 0;
  decoded.fallen = (flags & 4U) != 0;
  decoded.eso_valid = (flags & 8U) != 0;

  auto all_finite = [](const auto& values) {
    for (double value : values)
      if (!std::isfinite(value)) return false;
    return true;
  };
  if (!all_finite(decoded.state) || !all_finite(decoded.gyro) ||
      !all_finite(decoded.specific_force) || !all_finite(decoded.joint_position) ||
      !all_finite(decoded.joint_velocity) || !all_finite(decoded.eso_disturbance) ||
      !std::isfinite(decoded.body_vx_m_s) ||
      !std::isfinite(decoded.body_vy_m_s) || !std::isfinite(decoded.body_yaw_rad) ||
      !std::isfinite(decoded.leg_height_m))
    return AmpDecodeResult::BadPayload;

  out = decoded;
  return AmpDecodeResult::Ok;
}

}  // namespace control_stack
