#pragma once

#include "control_stack/common/fixed_ring_buffer.hpp"
#include "control_stack/platform/logging/logger.hpp"
#include <array>
#include <atomic>
#include <cstddef>

namespace control_stack {

template <std::size_t Capacity, std::size_t EventBytes = 24,
          std::size_t DetailBytes = 64>
class StaticLogBuffer final : public ILogSink {
  static_assert(EventBytes > 1 && DetailBytes > 1, "log text buffers need a terminator");
 public:
  struct Record {
    LogLevel level{LogLevel::Info};
    std::uint64_t timestamp_us{0};
    std::array<char, EventBytes> event{};
    std::array<char, DetailBytes> detail{};
  };

  void write(const LogRecord& record) override {
    Record fixed{};
    fixed.level = record.level;
    fixed.timestamp_us = record.timestamp_us;
    copy(fixed.event, record.event);
    copy(fixed.detail, record.detail);
    if (!records_.push(fixed)) dropped_.fetch_add(1, std::memory_order_relaxed);
  }
  bool pop(Record& record) { return records_.pop(record); }
  std::size_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  template <std::size_t N>
  static void copy(std::array<char, N>& dst, std::string_view src) {
    const auto count = src.size() < N - 1 ? src.size() : N - 1;
    for (std::size_t i = 0; i < count; ++i) dst[i] = src[i];
    dst[count] = '\0';
  }

  StaticRingBuffer<Record, Capacity> records_{};
  std::atomic<std::size_t> dropped_{0};
};

}  // namespace control_stack
