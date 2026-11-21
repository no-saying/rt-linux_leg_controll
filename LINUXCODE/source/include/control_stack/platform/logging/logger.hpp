#pragma once

#include <cstdint>
#include <string_view>

namespace control_stack {

enum class LogLevel : std::uint8_t { Debug, Info, Warning, Error };
struct LogRecord {
  LogLevel level{LogLevel::Info};
  std::uint64_t timestamp_us{0};
  std::string_view event{};
  std::string_view detail{};
};

class ILogSink {
 public:
  virtual ~ILogSink() = default;
  virtual void write(const LogRecord&) = 0;
};

}
