#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace control_stack {

template <typename T, std::size_t Capacity>
class StaticRingBuffer {
  static_assert(Capacity > 0, "Capacity must be positive");
 public:
  bool push(const T& value) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    const auto head = head_.load(std::memory_order_acquire);
    if (tail - head == Capacity) return false;
    storage_[tail % Capacity] = value;
    tail_.store(tail + 1U, std::memory_order_release);
    return true;
  }

  bool pop(T& value) {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    if (tail == head) return false;
    value = storage_[head % Capacity];
    head_.store(head + 1U, std::memory_order_release);
    return true;
  }

  void clear() { head_.store(0, std::memory_order_relaxed); tail_.store(0, std::memory_order_relaxed); }
  std::size_t size() const {
    const auto head = head_.load(std::memory_order_acquire);
    const auto tail = tail_.load(std::memory_order_acquire);
    return tail >= head ? tail - head : 0;
  }
  constexpr std::size_t capacity() const { return Capacity; }
  bool empty() const { return size() == 0; }
  bool full() const { return size() == Capacity; }

 private:
  std::array<T, Capacity> storage_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

}
