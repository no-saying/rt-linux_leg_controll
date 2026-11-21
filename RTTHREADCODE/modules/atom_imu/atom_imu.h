#ifndef ATOM_IMU_H
#define ATOM_IMU_H

/*
 * XRobot ATOM-IMU V5.3 (classic CAN, 1 Mbps) decoder for RK3568 CPU3.
 *
 * The module is a CAN consumer: it registers one CANInstance per output frame
 * ID on the selected bus. Decode callbacks run in the motor thread (via
 * CANServicePoll) and update the latest decoded sample; the periph thread
 * snapshots it for reporting. Slow readers can skip intermediate samples;
 * receive capacity depends on the CAN FIFO and the shared polling budget.
 *
 * Frame IDs are <base> + type. base defaults to 48 / 0x30 (see the IMU's
 * `set_can_id`):
 *   accl  base+0   3x 21-bit  accel   [-24g, +24g]     -> g
 *   gyro  base+1   3x 21-bit  gyro    [-2000dps, +2000]-> rad/s
 *   (base+2 reserved)
 *   eulr  base+3   3x 21-bit  euler   [-pi, +pi]       -> rad (pitch,roll,yaw)
 *   quat  base+4   4x int16   quat    normalized       -> w,x,y,z
 *
 * `atom_imu_init` must run before MotorServiceInit(): it mutates the shared
 * CAN instance table without a lock, exactly like the DM motor registration.
 */
#include <stdint.h>

#define ATOM_IMU_BASE_ID   48U

#define ATOM_IMU_ID_ACCL  (ATOM_IMU_BASE_ID + 0U)
#define ATOM_IMU_ID_GYRO  (ATOM_IMU_BASE_ID + 1U)
#define ATOM_IMU_ID_EULR  (ATOM_IMU_BASE_ID + 3U)
#define ATOM_IMU_ID_QUAT  (ATOM_IMU_BASE_ID + 4U)

/* Frame-type enable bits for atom_imu_init(). Register only what the IMU is
 * configured to send, e.g. ATOM_IMU_F_EULR for a `data:eulr` configuration. */
#define ATOM_IMU_F_ACCL (1u << 0)
#define ATOM_IMU_F_GYRO (1u << 1)
#define ATOM_IMU_F_EULR (1u << 2)
#define ATOM_IMU_F_QUAT (1u << 3)
#define ATOM_IMU_F_ALL  (ATOM_IMU_F_ACCL | ATOM_IMU_F_GYRO | \
                         ATOM_IMU_F_EULR | ATOM_IMU_F_QUAT)

struct atom_imu_sample {
    float accel[3];   /* g, x/y/z */
    float gyro[3];    /* rad/s, x/y/z */
    float eulr[3];    /* rad, pitch/roll/yaw */
    float quat[4];    /* w/x/y/z, normalized */
    uint32_t accel_count;
    uint32_t gyro_count;
    uint32_t eulr_count;
    uint32_t quat_count;
    uint32_t last_ms;     /* BSPTimeMillis() of the newest decoded frame */
    uint32_t error_count; /* rejected frames: DLC != 8 or unexpected type id */
    uint8_t  online;      /* 1 after a valid frame, until daemon expiry */
    uint32_t lost_count;  /* online -> offline transitions, excluding startup */
};

/* Register ATOM-IMU receive callbacks on CAN bus 1 or 2 for the frame types
 * enabled in `frames` (bitmask of ATOM_IMU_F_*). Returns RT_EOK or a negative
 * error. Safe to call once; repeat calls are no-ops. */
int atom_imu_init(int bus, uint32_t frames);

/* Consistent snapshot of the latest decoded values. Returns RT_EOK or -EINVAL. */
int atom_imu_read(struct atom_imu_sample *sample);

/* Matches sample.online: 0 until the first valid frame and after daemon expiry.
 * Rejected frames do not update values, counters, timestamps or the watchdog;
 * only error_count increases. */
uint8_t atom_imu_is_online(void);

#endif
