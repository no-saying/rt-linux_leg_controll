#pragma once

#include "control_stack/application/control_application.hpp"
#include "control_stack/platform/amp/amp_frame.hpp"

namespace control_stack {

class ControlRuntime {
 public:
  explicit ControlRuntime(ControlApplication& app, ILogSink* log = nullptr)
      : app_(app), log_(log) {}

  AmpDecodeResult ingestAmpBytes(const std::uint8_t* bytes, std::size_t size,
                                 std::uint64_t received_us);
  ControlOutput tick(std::uint64_t now_us);
  const ApplicationStatus& status() const { return status_; }
  const Command& command() const { return command_; }
  void forceFault(std::uint32_t code, std::uint64_t now_us);
  void clearFault();

 private:
  static bool isFresh(std::uint16_t sequence, std::uint16_t previous, bool initialized);
  void updateRecovery(const SensorFrame& sensors, Command& command);
  void selectController();
  ControlApplication& app_;
  ILogSink* log_;
  PerceptionState perception_{};
  AmpSafetySnapshot rt_safety_{};
  ApplicationStatus status_{};
  Command command_{};
  SensorFrame sensors_{};
  bool have_command_sequence_{false};
  bool have_sensor_sequence_{false};
  bool have_safety_sequence_{false};
  bool fault_latched_{false};
  bool fall_recovery_latched_{false};
  std::uint16_t last_command_sequence_{0};
  std::uint16_t last_sensor_sequence_{0};
  std::uint16_t last_safety_sequence_{0};
  std::uint64_t recovery_start_us_{0};
  std::uint64_t last_recovery_sensor_us_{0};
  std::uint64_t last_tick_us_{0};
  std::uint64_t have_sensor_us_{0};
  std::uint64_t have_safety_us_{0};
  ControlSelection selection_{};
  bool selection_initialized_{false};
};

}  // namespace control_stack
