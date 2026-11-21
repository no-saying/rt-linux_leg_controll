#pragma once

#include <cstddef>
#include <cstdint>

namespace control_stack {

class IAmpTransport {
 public:
  virtual ~IAmpTransport() = default;
  virtual bool send(const std::uint8_t* data, std::size_t size) = 0;
  virtual std::size_t receive(std::uint8_t* data, std::size_t capacity) = 0;
};

}
