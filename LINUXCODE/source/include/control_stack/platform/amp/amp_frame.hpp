#pragma once

#include "control_stack/common/types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace control_stack {

constexpr std::size_t kAmpMaxPayload = 96;
constexpr std::size_t kAmpHeaderSize = 14;
constexpr std::size_t kAmpFrameOverhead = 16;
constexpr std::size_t kAmpMaxFrameSize = kAmpHeaderSize + kAmpMaxPayload + 2;

struct AmpFrame {
  std::uint8_t type{0};
  std::uint16_t flags{0};
  std::uint16_t sequence{0};
  std::uint32_t timestamp_ms{0};
  std::uint16_t payload_size{0};
  std::array<std::uint8_t, kAmpMaxPayload> payload{};
};

enum class AmpDecodeResult : std::uint8_t {
  Ok, Incomplete, BadSync, BadVersion, BadLength, BadCrc, BadType, BadPayload,
  StaleSequence
};

class AmpFrameCodec {
 public:
  static bool encode(const AmpFrame&, std::uint8_t* out, std::size_t capacity,
                     std::size_t& written);
  static AmpDecodeResult decode(const std::uint8_t* bytes, std::size_t size,
                                AmpFrame& out, std::size_t& consumed);
  static bool encodeCommand(const Command&, std::uint16_t sequence,
                            std::uint32_t timestamp_ms, std::uint8_t* out,
                            std::size_t capacity, std::size_t& written);
  static AmpDecodeResult decodeCommand(const AmpFrame&, AmpCommand& out);
  static bool encodeSensorSnapshot(const SensorFrame&, std::uint16_t sequence,
                                   std::uint32_t timestamp_ms, std::uint8_t* out,
                                   std::size_t capacity, std::size_t& written);
  static AmpDecodeResult decodeSensorSnapshot(const AmpFrame&, SensorFrame& out);
};

}  // namespace control_stack
