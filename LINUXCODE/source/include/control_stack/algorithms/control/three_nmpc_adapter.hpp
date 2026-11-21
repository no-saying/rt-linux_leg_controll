#pragma once

#include "control_stack/algorithms/control/control_algorithm.hpp"
#include "decoupled_control.h"

namespace control_stack {

using ThreeNmpcInputMapper = bool (*)(
    void*, const SensorFrame&, const Command&, ControlSelection,
    p112::DecoupledInput&) noexcept;

class ThreeNmpcControlAlgorithm final : public IControlAlgorithm {
 public:
  ThreeNmpcControlAlgorithm(p112::DecoupledController& controller,
                            void* mapper_context,
                            ThreeNmpcInputMapper mapper,
                            double nominal_period_s = 0.01) noexcept;

  void reset(ControlSelection selection) override;
  ControlOutput update(const SensorFrame& sensors,
                       const Command& command) override;
  bool acknowledgeAppliedTorque(const TorqueVector& actual) noexcept;

 private:
  p112::DecoupledController& controller_;
  void* mapper_context_;
  ThreeNmpcInputMapper mapper_;
  ControlSelection selection_{};
  double nominal_period_s_;
  std::uint64_t last_timestamp_us_{};
  bool have_timestamp_{false};
};

}  // namespace control_stack
