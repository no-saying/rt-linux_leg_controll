#pragma once

#include <cstdint>

namespace control_stack {

class IThreadManager {
 public:
  using Entry = void (*)(void* context);
  virtual ~IThreadManager() = default;
  virtual bool start(const char* name, Entry entry, void* context,
                     std::uint32_t stack_bytes, std::uint8_t priority) = 0;
  virtual void requestStop() = 0;
};

}
