/*
 * XRobot ATOM-IMU V5.3 decoder (classic CAN, 1 Mbps).
 *
 * Consumes ATOM-IMU CAN frames by registering one CANInstance per frame ID on
 * the chosen bus. Decode runs in the motor thread's CANServicePoll() path and
 * updates the latest decoded sample; the periph thread snapshots it for
 * reporting. Slower readers can skip intermediate samples; CAN FIFO capacity
 * and the shared receive budget still limit the sustainable input rate.
 *
 * Frame layout (from the ATOM-IMU SDK README, ProcessClassicCanPacket):
 *   accl / gyro / eulr: 3x 21-bit unsigned fields packed into 8 bytes, LSB
 *     first (bitfields data1:bits[0..20], data2:bits[21..41],
 *     data3:bits[42..62], res:bit[63]).
 *   quat: 4x int16 (little-endian), each normalized by 32767.
 */
#include "atom_imu.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "daemon.h"
#include <rtthread.h>
#include <string.h>

#define ATOM_21_MAX     0x1FFFFFU
#define ATOM_INT16_MAX  32767.0f
#define ATOM_PI         3.14159265358979f

#define ATOM_ACCL_MIN  (-24.0f)
#define ATOM_ACCL_MAX  ( 24.0f)
#define ATOM_GYRO_MIN  (-2000.0f * ATOM_PI / 180.0f)
#define ATOM_GYRO_MAX  ( 2000.0f * ATOM_PI / 180.0f)
#define ATOM_EULR_MIN  (-ATOM_PI)
#define ATOM_EULR_MAX  ( ATOM_PI)

/* DaemonTask() ticks every 10 ms (motor thread), so 50 ticks == ~500 ms of
 * silence before the IMU is declared offline. */
#define ATOM_IMU_ONLINE_RELOAD 50U

static struct atom_imu_sample sample;
static CANInstance *atom_inst[4];
static DaemonInstance *atom_daemon;
static int atom_registered;

static float atom_decode21(uint32_t v, float min, float max)
{
    float norm = (float)(v & ATOM_21_MAX) / (float)ATOM_21_MAX;
    return min + norm * (max - min);
}

/* 21-bit fields, LSB-first bitfields (see header comment). */
static uint32_t atom_field0(const uint8_t d[8])
{
    return (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
           ((uint32_t)(d[2] & 0x1FU) << 16);
}

static uint32_t atom_field1(const uint8_t d[8])
{
    return ((uint32_t)d[2] >> 5) | ((uint32_t)d[3] << 3) |
           ((uint32_t)d[4] << 11) | ((uint32_t)(d[5] & 0x03U) << 19);
}

static uint32_t atom_field2(const uint8_t d[8])
{
    return ((uint32_t)d[5] >> 2) | ((uint32_t)d[6] << 6) |
           ((uint32_t)(d[7] & 0x7FU) << 14);
}

static float atom_int16_norm(int16_t v)
{
    return (float)v / ATOM_INT16_MAX;
}

static int16_t atom_s16(const uint8_t *d, unsigned int i)
{
    return (int16_t)((uint16_t)d[i] | ((uint16_t)d[i + 1] << 8));
}

static void atom_imu_cb(CANInstance *inst)
{
    const uint8_t *d = inst->rx_buff;
    uint32_t type = inst->rx_id - ATOM_IMU_BASE_ID;

    rt_enter_critical();
    if (inst->rx_len != 8U) {
        sample.error_count++;
        rt_exit_critical();
        return;
    }
    switch (type) {
    case 0: /* accl: x, y, z in g */
        sample.accel[0] = atom_decode21(atom_field0(d), ATOM_ACCL_MIN, ATOM_ACCL_MAX);
        sample.accel[1] = atom_decode21(atom_field1(d), ATOM_ACCL_MIN, ATOM_ACCL_MAX);
        sample.accel[2] = atom_decode21(atom_field2(d), ATOM_ACCL_MIN, ATOM_ACCL_MAX);
        sample.accel_count++;
        sample.accel_ms = BSPTimeMillis();
        sample.accel_us = DWT_GetTimeline_us();
        break;
    case 1: /* gyro: x, y, z in rad/s */
        sample.gyro[0] = atom_decode21(atom_field0(d), ATOM_GYRO_MIN, ATOM_GYRO_MAX);
        sample.gyro[1] = atom_decode21(atom_field1(d), ATOM_GYRO_MIN, ATOM_GYRO_MAX);
        sample.gyro[2] = atom_decode21(atom_field2(d), ATOM_GYRO_MIN, ATOM_GYRO_MAX);
        sample.gyro_count++;
        sample.gyro_ms = BSPTimeMillis();
        sample.gyro_us = DWT_GetTimeline_us();
        break;
    case 3: /* eulr: pitch, roll, yaw in rad */
        sample.eulr[0] = atom_decode21(atom_field0(d), ATOM_EULR_MIN, ATOM_EULR_MAX);
        sample.eulr[1] = atom_decode21(atom_field1(d), ATOM_EULR_MIN, ATOM_EULR_MAX);
        sample.eulr[2] = atom_decode21(atom_field2(d), ATOM_EULR_MIN, ATOM_EULR_MAX);
        sample.eulr_count++;
        sample.eulr_ms = BSPTimeMillis();
        sample.eulr_us = DWT_GetTimeline_us();
        break;
    case 4: /* quat: w, x, y, z */
        sample.quat[0] = atom_int16_norm(atom_s16(d, 0));
        sample.quat[1] = atom_int16_norm(atom_s16(d, 2));
        sample.quat[2] = atom_int16_norm(atom_s16(d, 4));
        sample.quat[3] = atom_int16_norm(atom_s16(d, 6));
        sample.quat_count++;
        sample.quat_ms = BSPTimeMillis();
        sample.quat_us = DWT_GetTimeline_us();
        break;
    default:
        sample.error_count++;
        rt_exit_critical();
        return;
    }
    sample.online = 1;
    sample.last_ms = BSPTimeMillis();
    if (atom_daemon)
        DaemonReload(atom_daemon);
    rt_exit_critical();
}

/* Daemon fires when no frame has arrived for ATOM_IMU_ONLINE_RELOAD ticks. */
static void atom_imu_lost_cb(void *arg)
{
    (void)arg;
    rt_enter_critical();
    /* The daemon starts with a grace period even before the first frame.
     * Count only an actual online -> offline transition. */
    if (sample.online) {
        sample.online = 0;
        sample.lost_count++;
    }
    rt_exit_critical();
}

static void atom_imu_unregister_all(void)
{
    unsigned int j;
    for (j = 0; j < 4; j++) {
        if (atom_inst[j]) {
            CANUnregister(atom_inst[j]);
            atom_inst[j] = RT_NULL;
        }
    }
}

int atom_imu_init(int bus, uint32_t frames)
{
    BSP_CAN_Handle *handle;
    static const uint32_t ids[4] = {
        ATOM_IMU_ID_ACCL, ATOM_IMU_ID_GYRO, ATOM_IMU_ID_EULR, ATOM_IMU_ID_QUAT,
    };
    static const uint32_t masks[4] = {
        ATOM_IMU_F_ACCL, ATOM_IMU_F_GYRO, ATOM_IMU_F_EULR, ATOM_IMU_F_QUAT,
    };
    unsigned int i;

    if (bus == 1)
        handle = &bsp_can1;
    else if (bus == 2)
        handle = &bsp_can2;
    else
        return -RT_EINVAL;

    if (atom_registered)
        return RT_EOK;

    memset(&sample, 0, sizeof(sample));
    for (i = 0; i < 4; i++) {
        CAN_Init_Config_s cfg;
        if (!(frames & masks[i]))
            continue;
        memset(&cfg, 0, sizeof(cfg));
        cfg.can_handle = handle;
        cfg.tx_id = 0; /* receive-only */
        cfg.rx_id = ids[i];
        cfg.can_module_callback = atom_imu_cb;
        cfg.id = RT_NULL;
        atom_inst[i] = CANRegister(&cfg);
        if (atom_inst[i] == RT_NULL) {
            atom_imu_unregister_all();
            return -RT_ENOMEM;
        }
    }
    if (atom_daemon == RT_NULL) {
        Daemon_Init_Config_s dconf;
        memset(&dconf, 0, sizeof(dconf));
        dconf.callback = atom_imu_lost_cb;
        dconf.owner_id = RT_NULL;
        dconf.reload_count = ATOM_IMU_ONLINE_RELOAD;
        atom_daemon = DaemonRegister(&dconf);
        if (atom_daemon == RT_NULL) {
            atom_imu_unregister_all();
            return -RT_ENOMEM;
        }
    }
    atom_registered = 1;
    return RT_EOK;
}

uint8_t atom_imu_is_online(void)
{
    uint8_t online;
    /* Use the same state as atom_imu_read(): a daemon grace period alone is
     * not evidence that any valid IMU frame has been received. */
    rt_enter_critical();
    online = sample.online;
    rt_exit_critical();
    return online;
}

int atom_imu_read(struct atom_imu_sample *out)
{
    if (out == RT_NULL)
        return -RT_EINVAL;
    rt_enter_critical();
    *out = sample;
    rt_exit_critical();
    return RT_EOK;
}
