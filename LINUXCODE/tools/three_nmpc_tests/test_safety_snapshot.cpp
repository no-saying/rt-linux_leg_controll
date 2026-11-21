#include "control_stack/algorithms/control/control_algorithm.hpp"
#include "control_stack/algorithms/greenwave/greenwave_algorithm.hpp"
#include "control_stack/algorithms/perception/basic_robot_perception.hpp"
#include "control_stack/application/control_runtime.hpp"
#include "control_stack/platform/amp/amp_frame.hpp"

#include <cassert>
#include <cstdint>

namespace {
class TestControl final : public control_stack::IControlAlgorithm {
 public:
  void reset(control_stack::ControlSelection) override {}
  control_stack::ControlOutput update(const control_stack::SensorFrame& sensors,
                                     const control_stack::Command&) override {
    last_sensors = sensors;
    control_stack::ControlOutput output{};
    output.valid = true;
    return output;
  }
  control_stack::SensorFrame last_sensors{};
};

class TestPlan final : public control_stack::IGreenWaveAlgorithm {
 public:
  void reset() override {}
  control_stack::GreenWavePlan update(const control_stack::SensorFrame&,
                                     const control_stack::Command& command) override {
    return {command, 0, false};
  }
};
}

int main() {
  using namespace control_stack;
  TestControl control;
  TestPlan planner;
  BasicRobotPerception perception;
  ControlApplication app({control, perception, planner, nullptr});
  ControlRuntime runtime(app);
  std::uint8_t wire[kAmpMaxFrameSize]{};
  std::size_t wire_size = 0;

  Command command{};
  command.vehicle_state = 1;
  assert(AmpFrameCodec::encodeCommand(command, 1, 100, wire, sizeof(wire),
                                      wire_size));
  assert(runtime.ingestAmpBytes(wire, wire_size, 100000) == AmpDecodeResult::Ok);

  AmpSafetySnapshot safety{};
  safety.pitch_rad = 0.9;
  safety.attitude_valid = true;
  safety.specific_force_valid = true;
  safety.specific_force_m_s2 = {0.0, 0.0, 9.80665};
  safety.fallen = true;
  assert(AmpFrameCodec::encodeSafetySnapshot(safety, 1, 100, wire,
                                             sizeof(wire), wire_size));
  assert(runtime.ingestAmpBytes(wire, wire_size, 100100) == AmpDecodeResult::Ok);
  safety = {};
  safety.attitude_valid = true;
  safety.body_parallel = true;
  assert(AmpFrameCodec::encodeSafetySnapshot(safety, 2, 101, wire,
                                             sizeof(wire), wire_size));
  assert(runtime.ingestAmpBytes(wire, wire_size, 101000) == AmpDecodeResult::Ok);

  SensorFrame sensors{};
  sensors.eso_valid = true;
  assert(AmpFrameCodec::encodeSensorSnapshot(sensors, 1, 101, wire,
                                             sizeof(wire), wire_size));
  AmpFrame sensor_frame{};
  std::size_t consumed = 0;
  assert(AmpFrameCodec::decode(wire, wire_size, sensor_frame, consumed) ==
         AmpDecodeResult::Ok);
  SensorFrame decoded_sensor{};
  assert(AmpFrameCodec::decodeSensorSnapshot(sensor_frame, decoded_sensor) ==
         AmpDecodeResult::Ok);
  assert(!decoded_sensor.attitude_valid && !decoded_sensor.kinematic_state_valid &&
         !decoded_sensor.wheel_speed_valid && !decoded_sensor.body_velocity_valid);
  sensors.attitude_valid = true;
  sensors.imu_valid = true;
  sensors.kinematic_state_valid = true;
  sensors.joint_state_valid = true;
  sensors.wheel_speed_valid = true;
  sensors.body_velocity_valid = true;
  assert(AmpFrameCodec::encodeSensorSnapshot(sensors, 2, 101, wire,
                                             sizeof(wire), wire_size));
  assert(AmpFrameCodec::decode(wire, wire_size, sensor_frame, consumed) ==
         AmpDecodeResult::Ok);
  assert(AmpFrameCodec::decodeSensorSnapshot(sensor_frame, decoded_sensor) ==
         AmpDecodeResult::Ok);
  assert(decoded_sensor.attitude_valid && decoded_sensor.imu_valid &&
         decoded_sensor.kinematic_state_valid && decoded_sensor.joint_state_valid &&
         decoded_sensor.wheel_speed_valid && decoded_sensor.body_velocity_valid);
  sensors = {};
  sensors.eso_valid = true;
  assert(AmpFrameCodec::encodeSensorSnapshot(sensors, 1, 101, wire,
                                             sizeof(wire), wire_size));
  assert(runtime.ingestAmpBytes(wire, wire_size, 101100) == AmpDecodeResult::Ok);
  assert(runtime.tick(101100).valid);
  assert(control.last_sensors.attitude_valid);
  assert(control.last_sensors.body_parallel);
  assert(runtime.status().state == RobotState::Recovery);
  assert(runtime.status().recovery_stage == RecoveryStage::SwingLegsBackward);
  assert(runtime.tick(101200).valid);
  assert(runtime.status().recovery_stage == RecoveryStage::SwingLegsBackward);
}
