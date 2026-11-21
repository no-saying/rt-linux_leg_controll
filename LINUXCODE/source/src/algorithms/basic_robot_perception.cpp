#include "control_stack/algorithms/perception/basic_robot_perception.hpp"

#include <algorithm>
#include <cmath>

namespace control_stack {
namespace {
constexpr double kGravity = 9.80665;
}

BasicRobotPerception::BasicRobotPerception(
    const BasicPerceptionConfig& config) noexcept
    : config_(config) {}

void BasicRobotPerception::reset() {
  last_timestamp_us_ = 0;
  have_timestamp_ = false;
  slip_dwell_s_ = 0.0;
  unload_dwell_s_ = 0.0;
  recontact_dwell_s_ = 0.0;
  support_ = SupportMode::Unknown;
  slip_ = {};
  last_output_ = {};
}

PerceptionState BasicRobotPerception::update(const SensorFrame& sensors) {
  if (have_timestamp_ && sensors.timestamp_us <= last_timestamp_us_)
    return last_output_;
  PerceptionState result{};
  const double pitch = sensors.state[0];
  const double roll = sensors.state[2];
  result.attitude_valid = sensors.attitude_valid && std::isfinite(pitch) &&
                          std::isfinite(roll);
  if (result.attitude_valid) {
    result.fallen = sensors.fallen || std::max(std::abs(roll), std::abs(pitch)) >=
                                          config_.fallen_angle_rad;
    result.body_parallel = sensors.body_parallel ||
                           (std::abs(roll) <= config_.parallel_angle_rad &&
                            std::abs(pitch) <= config_.parallel_angle_rad);
  } else {
    result.fallen = sensors.fallen;
    result.body_parallel = sensors.body_parallel;
  }

  const double swing = sensors.state[4];
  const double leg_delta = sensors.state[8];
  result.legs_centered = sensors.legs_centered ||
                         (sensors.kinematic_state_valid && std::isfinite(swing) &&
                          std::isfinite(leg_delta) &&
                          std::abs(swing) <= config_.centered_swing_rad &&
                          std::abs(leg_delta) <= config_.centered_leg_delta_rad);

  double dt = config_.nominal_sample_s;
  if (have_timestamp_ && sensors.timestamp_us > last_timestamp_us_) {
    dt = static_cast<double>(sensors.timestamp_us - last_timestamp_us_) * 1e-6;
  }
  last_timestamp_us_ = sensors.timestamp_us;
  have_timestamp_ = true;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.020) {
    slip_dwell_s_ = 0.0;
    unload_dwell_s_ = 0.0;
    recontact_dwell_s_ = 0.0;
    dt = 0.0;
  }

  // The AMP snapshot exposes only mean wheel speed and body forward speed, so
  // this residual is a vehicle-level slip indicator applied to both wheels.
  const double wheel_speed = sensors.state[10];
  if (sensors.wheel_speed_valid && sensors.body_velocity_valid &&
      std::isfinite(wheel_speed) && std::isfinite(sensors.body_vx_m_s)) {
    result.wheel_speed_residual_m_s = wheel_speed - sensors.body_vx_m_s;
    const double residual = std::abs(result.wheel_speed_residual_m_s);
    if (residual >= config_.wheel_slip_enter_m_s)
      slip_dwell_s_ += dt;
    else
      slip_dwell_s_ = 0.0;
    if (slip_dwell_s_ >= config_.slip_confirm_s)
      slip_.fill(true);
    if (residual <= config_.wheel_slip_exit_m_s)
      slip_.fill(false);
  } else {
    slip_dwell_s_ = 0.0;
  }
  result.wheel_slip = slip_;
  for (std::size_t side = 0; side < slip_.size(); ++side)
    result.wheel_torque_scale[side] = slip_[side]
                                          ? config_.wheel_slip_torque_scale
                                          : 1.0;

  const double ax = sensors.specific_force[0];
  const double ay = sensors.specific_force[1];
  const double az = sensors.specific_force[2];
  if (sensors.imu_valid && std::isfinite(ax) && std::isfinite(ay) &&
      std::isfinite(az)) {
    result.support_valid = true;
    const double specific_force = std::hypot(std::hypot(ax, ay), az);
    if (specific_force < config_.support_unload_g * kGravity)
      unload_dwell_s_ += dt;
    else
      unload_dwell_s_ = 0.0;
    if (specific_force > config_.support_contact_g * kGravity)
      recontact_dwell_s_ += dt;
    else
      recontact_dwell_s_ = 0.0;

    if ((support_ == SupportMode::Unknown || support_ == SupportMode::Ground) &&
        specific_force < config_.support_unload_g * kGravity) {
      support_ = SupportMode::Unloading;
    }
    if (support_ == SupportMode::Unloading &&
        unload_dwell_s_ >= config_.support_unload_s) {
      support_ = SupportMode::Airborne;
    } else if (support_ == SupportMode::Unloading &&
               specific_force >= config_.support_contact_g * kGravity) {
      support_ = SupportMode::Ground;
    } else if (support_ == SupportMode::Airborne &&
               recontact_dwell_s_ >= config_.support_recontact_s) {
      support_ = SupportMode::Recontact;
    } else if (support_ == SupportMode::Recontact &&
               specific_force >= config_.support_contact_g * kGravity) {
      support_ = SupportMode::Ground;
    } else if (support_ == SupportMode::Recontact &&
               specific_force < config_.support_unload_g * kGravity) {
      support_ = SupportMode::Airborne;
    } else if (support_ == SupportMode::Unknown) {
      support_ = SupportMode::Ground;
    }
    result.support_confidence =
        std::clamp(std::abs(specific_force / kGravity - 0.5) * 2.0, 0.0, 1.0);
  } else {
    unload_dwell_s_ = 0.0;
    recontact_dwell_s_ = 0.0;
  }
  result.support = support_;
  result.confidence = result.attitude_valid ? 1.0 : 0.0;
  last_output_ = result;
  return result;
}

}  // namespace control_stack
