#ifndef AMP_MOTOR_CONFIG_H
#define AMP_MOTOR_CONFIG_H
#include <stdint.h>
struct amp_motor_config {
    uint32_t number, family, model, driver_model, bus, can_id, feedback_id;
    uint32_t offline_ms, feedback_period_ms, mit_period_ms, native_period_ms;
    float pmax, vmax, tmax, max_velocity, max_torque;
    float speed_kp, speed_ki, speed_kd, speed_output;
    float position_kp, position_output;
};
/* Immutable generated table from project/config/motors.json. */
extern const struct amp_motor_config amp_motor_config[];
extern const uint32_t amp_motor_config_count, amp_motor_config_hash;
extern const uint32_t amp_motor_status_hz, amp_motor_feedback_hz;
int AmpMotorRegisterConfigured(void);
#endif
