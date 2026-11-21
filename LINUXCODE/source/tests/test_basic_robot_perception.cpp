#include "control_stack/algorithms/perception/basic_robot_perception.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace control_stack;
  BasicRobotPerception perception;
  SensorFrame sensors{};
  sensors.attitude_valid = true;
  sensors.imu_valid = true;
  sensors.kinematic_state_valid = true;
  sensors.wheel_speed_valid = true;
  sensors.body_velocity_valid = true;
  sensors.timestamp_us = 1000;
  sensors.specific_force = {0.0, 0.0, 9.80665};
  auto result = perception.update(sensors);
  assert(result.attitude_valid && result.body_parallel && result.legs_centered);
  assert(!result.fallen && result.support_valid &&
         result.support == SupportMode::Ground);

  sensors.state[2] = 0.9;
  sensors.timestamp_us += 2000;
  result = perception.update(sensors);
  assert(result.fallen && !result.body_parallel);

  sensors.state[2] = 0.0;
  sensors.state[10] = 1.0;
  sensors.body_vx_m_s = 0.0;
  for (int sample = 0; sample < 12; ++sample) {
    sensors.timestamp_us += 2000;
    result = perception.update(sensors);
  }
  assert(result.wheel_slip[0] && result.wheel_slip[1]);
  assert(result.wheel_torque_scale[0] < 1.0 &&
         result.wheel_torque_scale[1] < 1.0);

  sensors.state[10] = 0.2;
  sensors.timestamp_us += 2000;
  const auto held = perception.update(sensors);
  assert(held.wheel_slip[0] && held.wheel_slip[1]);
  sensors.state[10] = 0.0;
  sensors.timestamp_us += 2000;
  result = perception.update(sensors);
  assert(!result.wheel_slip[0] && !result.wheel_slip[1]);

  sensors.imu_valid = false;
  sensors.timestamp_us += 30000;
  result = perception.update(sensors);
  assert(!result.support_valid && result.support == SupportMode::Ground);
  sensors.imu_valid = true;
  sensors.specific_force = {};
  for (int sample = 0; sample < 8; ++sample) {
    sensors.timestamp_us += 2000;
    result = perception.update(sensors);
  }
  assert(result.support == SupportMode::Airborne);
  const auto duplicate = perception.update(sensors);
  assert(duplicate.support == SupportMode::Airborne);
}
