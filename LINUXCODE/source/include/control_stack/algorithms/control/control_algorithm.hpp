#pragma once

#include "control_stack/common/types.hpp"

namespace control_stack {

enum class MpcProfile : std::uint8_t {
  Balance = 0,
  Locomotion = 1,
  Recovery = 2,
};

enum class ControlBackend : std::uint8_t { Mpc, ReinforcementLearning, Hybrid };

struct ControlSelection {
  MpcProfile profile{MpcProfile::Balance};
  ControlBackend backend{ControlBackend::Mpc};
};

class IControlAlgorithm {
 public:
  virtual ~IControlAlgorithm() = default;
  virtual void reset(ControlSelection selection) = 0;
  virtual ControlOutput update(const SensorFrame&, const Command&) = 0;
};

}
