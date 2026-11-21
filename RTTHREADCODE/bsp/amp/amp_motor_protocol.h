#ifndef AMP_MOTOR_PROTOCOL_H
#define AMP_MOTOR_PROTOCOL_H
#include <stdint.h>
/* Fixed-width little-endian ABI, IEEE754 float32, no pointers or native enums.
 * All memory is Normal NC on CPU3 and O_SYNC/no-map on Linux. */
#define AMP_MOTOR_BASE 0x07870000U
#define AMP_MOTOR_BYTES 65536U
#define AMP_MOTOR_MAGIC 0x4d415001U
#define AMP_MOTOR_VERSION 1U
#define AMP_MOTOR_MAX 16U
#define AMP_MOTOR_QUEUE 16U
#define AMP_MOTOR_BATCH 16U
#define AMP_MOTOR_TIMEOUT_MS 6000U
#define AMP_MOTOR_DM 1U
#define AMP_MOTOR_DJI 2U
#define AMP_MOTOR_AXIS_OUTPUT 1U
#define AMP_MOTOR_AXIS_ROTOR 2U

enum amp_motor_op {
    AMP_OP_ENABLE=1, AMP_OP_DISABLE, AMP_OP_VELOCITY, AMP_OP_POSITION,
    AMP_OP_TORQUE, AMP_OP_MIT, AMP_OP_MODE, AMP_OP_CLEAR_ERROR, AMP_OP_ZERO,
    AMP_OP_READ_REGISTER, AMP_OP_WRITE_REGISTER, AMP_OP_CURRENT, AMP_OP_PID,
    AMP_OP_RESET_ESTOP
};
#define AMP_CAP(op) (1U << (op))
enum amp_motor_result {
    AMP_PENDING=0, AMP_OK=1, AMP_ERR_ARGUMENT=-1, AMP_ERR_NOT_FOUND=-2,
    AMP_ERR_UNSUPPORTED=-3, AMP_ERR_OFFLINE=-4, AMP_ERR_ESTOP=-5,
    AMP_ERR_BUSY=-6, AMP_ERR_TIMEOUT=-7, AMP_ERR_CANCELED=-8,
    AMP_ERR_SESSION=-9, AMP_ERR_STALE=-10, AMP_ERR_FULL=-11,
    AMP_ERR_DISABLED_REQUIRED=-12, AMP_ERR_DRIVE=-13, AMP_ERR_IO=-14,
    AMP_ERR_CONFIG=-15, AMP_ERR_RT_OFFLINE=-16
};
struct amp_motor_command {
    uint32_t motor, op;
    /* velocity:[rad/s]; position:[rad,velocity_limit]; torque:[Nm];
     * MIT:[rad,rad/s,Nm,Kp,Kd]; current:[DJI raw counts];
     * PID:[Kp,Ki,Kd,integral_limit,output_limit], arg=loop 1/2/4.
     * MODE/READ/WRITE: arg=mode or RID; flags=raw uint32 register data. */
    float value[6];
    uint32_t arg, flags;
};
struct amp_motor_slot {
    uint32_t published_seq, session, generation, count;
    struct amp_motor_command command[AMP_MOTOR_BATCH];
    uint32_t accepted_seq, completed_seq;
    int32_t result;
    uint32_t applied_ms;
    int32_t item_result[AMP_MOTOR_BATCH];
    uint32_t reply[AMP_MOTOR_BATCH];
};
/* Only fixed inventory confirmation; no encoder payload in this channel. */
struct amp_motor_status {
    uint32_t motor, model, bus, error, offline;
};
struct amp_motor_feedback {
    uint32_t motor, sample_ms, drive_state, mode, confirmed, enabled;
    uint32_t feedback_count, age_ms, control_tx_count, active_op;
    int32_t last_result;
    /* torque_nm is output-shaft torque; for DJI it is estimated from the raw
     * C620 current feedback and the motor's calibrated torque constant. */
    float position_rad, position_total_rad, velocity_rad_s, torque_nm, current_raw, temperature_c;
    float reference, max_velocity_rad_s, max_torque_nm, pmax, vmax, tmax;
};
struct amp_motor_shared {
    uint32_t magic, version, bytes, session;
    uint32_t producer, consumer; /* monotonic uint32; modulo only for slot index */
    /* Independent priority lanes. Normal queue saturation cannot block them. */
    uint32_t estop_request, estop_ack, disable_request, disable_ack;
    uint32_t generation, estop_latched;
    uint32_t status_seq, now_ms, motor_count, control_tick;
    uint32_t feedback_seq, status_hz, feedback_hz, config_hash;
    uint32_t status_ms, feedback_ms, reserved[2];
    struct amp_motor_status motor[AMP_MOTOR_MAX];
    struct amp_motor_feedback feedback[AMP_MOTOR_MAX];
    struct amp_motor_slot slot[AMP_MOTOR_QUEUE];
};
_Static_assert(sizeof(struct amp_motor_command)==40,"command ABI");
_Static_assert(sizeof(struct amp_motor_status)==20,"status ABI");
_Static_assert(sizeof(struct amp_motor_feedback)==92,"feedback ABI");
_Static_assert(sizeof(struct amp_motor_slot)==800,"slot ABI");
_Static_assert(sizeof(struct amp_motor_shared)<=AMP_MOTOR_BYTES,"shared reservation");
#endif
