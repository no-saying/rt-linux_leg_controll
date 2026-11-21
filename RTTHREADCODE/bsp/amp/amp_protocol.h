#ifndef AMP_PROTOCOL_H
#define AMP_PROTOCOL_H

#include <stdint.h>

/* AMP v0.1 wire framing shared with the Linux control stack. */
#define AMP_SOF0                 0xA5U
#define AMP_SOF1                 0x5AU
#define AMP_PROTOCOL_VERSION     1U
#define AMP_FRAME_HEADER_BYTES   14U /* SOF is not included in the CRC span. */
#define AMP_FRAME_OVERHEAD       16U /* SOF(2) + header(12) + CRC(2). */
#define AMP_MAX_PAYLOAD_BYTES    256U
#define AMP_COMMAND_PAYLOAD_BYTES 16U
#define AMP_ACK_PAYLOAD_BYTES    12U

enum amp_message_type {
    AMP_MSG_COMMAND_SETPOINT = 0x01,
    AMP_MSG_COMMAND_ACK      = 0x02,
    AMP_MSG_TELEMETRY_STATE  = 0x10,
    AMP_MSG_TELEMETRY_ACTUATOR = 0x11,
    AMP_MSG_SAFETY_SNAPSHOT  = 0x13,
    AMP_MSG_FAULT_EVENT      = 0x20,
    AMP_MSG_HEARTBEAT        = 0x7f,
};

enum amp_vehicle_state {
    AMP_VEHICLE_DISABLED = 0,
    AMP_VEHICLE_ENABLED  = 1,
    AMP_VEHICLE_ESTOP    = 2,
    AMP_VEHICLE_FAULT    = 3,
};

enum amp_motion_state {
    AMP_MOTION_STOP       = 0,
    AMP_MOTION_LOCOMOTION = 1,
    AMP_MOTION_STEP_UP    = 2,
    AMP_MOTION_JUMP       = 3,
    AMP_MOTION_RECOVERY   = 4,
    AMP_MOTION_STEP_DOWN  = 5,
};

enum amp_sensor_flag {
    AMP_SENSOR_BODY_PARALLEL       = 1U << 0,
    AMP_SENSOR_LEGS_CENTERED       = 1U << 1,
    AMP_SENSOR_FALLEN              = 1U << 2,
    AMP_SENSOR_ESO_VALID           = 1U << 3,
    AMP_SENSOR_ATTITUDE_VALID      = 1U << 4,
    AMP_SENSOR_IMU_VALID           = 1U << 5,
    AMP_SENSOR_KINEMATIC_VALID     = 1U << 6,
    AMP_SENSOR_JOINT_STATE_VALID   = 1U << 7,
    AMP_SENSOR_WHEEL_SPEED_VALID   = 1U << 8,
    AMP_SENSOR_BODY_VELOCITY_VALID = 1U << 9,
};

/* ACK result/reason values are part of the RT application contract. */
enum amp_ack_result {
    AMP_ACK_ACCEPTED = 0,
    AMP_ACK_REJECTED = 1,
};

enum amp_ack_reason {
    AMP_ACK_REASON_NONE = 0,
    AMP_ACK_REASON_VERSION,
    AMP_ACK_REASON_TYPE,
    AMP_ACK_REASON_FLAGS,
    AMP_ACK_REASON_LENGTH,
    AMP_ACK_REASON_CRC,
    AMP_ACK_REASON_SEQUENCE,
    AMP_ACK_REASON_VALUE,
    AMP_ACK_REASON_STATE,
    AMP_ACK_REASON_TIMEOUT,
};

#endif
