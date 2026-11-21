/*
 * RT-Thread application for RK3568 AMP CPU3
 *
 * Board init (board.c → rt_hw_board_init) already calls:
 *   1. rt_hw_interrupt_init()  — GICv3
 *   2. rt_hw_uart_init()       — UART4 RS485 data port
 *   3. rt_system_heap_init()   — heap
 *   4. _timer_init()           — ARM generic timer tick
 *   5. amp_ipc_init()          — shared memory + mailbox IPC
 *
 * So here we just start application threads.
 */
#include <rtthread.h>
#include <stdint.h>
#include <string.h>
#include "amp_ipc.h"
#include "amp_periph.h"
#include "board.h"
#include "bsp_can.h"
#include "motor_task.h"
#include "dmmotor.h"
#include "amp_motor_service.h"
#include "amp_motor_config.h"
#include "dm_motion_protocol.h"
#include "bsp_dwt.h"
#include "amp_diag.h"
#include "amp_dma.h"
#include "amp_uart4.h"
#include "amp_bench_protocol.h"
#include "atom_imu.h"
#include "amp_application.h"
#include "global_eso.h"

#define DM_APP_CAN_ID 0x06U
#define DM_APP_MASTER_ID 0x16U

static DMMotorInstance *dm_motor;
static struct amp_application amp_application;
static struct global_eso global_eso;
static struct global_eso_state global_eso_output;
static uint32_t global_eso_eulr_count;
static uint32_t global_eso_gyro_count;
static uint64_t global_eso_last_update_us;
static uint8_t global_eso_output_valid;

static void amp_update_global_eso(void)
{
    struct atom_imu_sample sample;
    struct global_eso_measurement measurement;
    uint64_t now_us = DWT_GetTimeline_us();
    double dt;

    if (atom_imu_read(&sample) != RT_EOK || !sample.online ||
        sample.eulr_count == 0U || sample.gyro_count == 0U) {
        global_eso_reset(&global_eso, RT_NULL, RT_NULL);
        global_eso_last_update_us = 0U;
        global_eso_output_valid = 0U;
        return;
    }
    if (sample.eulr_count == global_eso_eulr_count ||
        sample.gyro_count == global_eso_gyro_count) {
        if (global_eso_last_update_us != 0U &&
            now_us - global_eso_last_update_us > 20000U) {
            global_eso_reset(&global_eso, RT_NULL, RT_NULL);
            global_eso_last_update_us = 0U;
            global_eso_output_valid = 0U;
        }
        return;
    }

    global_eso_eulr_count = sample.eulr_count;
    global_eso_gyro_count = sample.gyro_count;
    dt = global_eso_last_update_us == 0U
             ? 0.005
             : (double)(now_us - global_eso_last_update_us) / 1000000.0;
    global_eso_last_update_us = now_us;

    /* ATOM reports Euler angles as pitch, roll, yaw; the observer uses
     * roll, pitch, yaw. Its model acceleration is zero until a nominal
     * rigid-body model is available in the RT application. */
    measurement.angle[0] = sample.eulr[1];
    measurement.angle[1] = sample.eulr[0];
    measurement.angle[2] = sample.eulr[2];
    measurement.rate[0] = sample.gyro[0];
    measurement.rate[1] = sample.gyro[1];
    measurement.rate[2] = sample.gyro[2];
    measurement.nominal_acceleration[0] = 0.0;
    measurement.nominal_acceleration[1] = 0.0;
    measurement.nominal_acceleration[2] = 0.0;

    if (global_eso_update(&global_eso, &measurement, dt,
                          &global_eso_output))
        global_eso_output_valid = 1U;
    else {
        global_eso_reset(&global_eso, RT_NULL, RT_NULL);
        global_eso_last_update_us = 0U;
        global_eso_output_valid = 0U;
    }
}

void MotorApplicationInit(void)
{
    int rc=AmpMotorRegisterConfigured();
    dm_motor=DMMotorAt(0);
    /* Retire the temporary single-motor test ABI. Old tools fail closed. */
    *(volatile uint32_t *)DM_MOTION_BASE=0;
    AmpMotorInit((volatile struct amp_motor_shared *)AMP_MOTOR_BASE);
    rt_kprintf("[MOTOR AMP] protocol v1 fixed=%u init=%d disabled boot status=%uHz feedback=%uHz\n",
        amp_motor_config_count,rc,amp_motor_status_hz,amp_motor_feedback_hz);
}

void MotorApplicationUpdate(void)
{
    uint32_t vehicle_state;

    /* The line protocol owns the high-level safety gate. The existing shared
     * motor ABI remains below this gate and cannot produce output while Linux
     * has not sent a valid ENABLED setpoint. */
    amp_application_get_state(&amp_application, &vehicle_state, RT_NULL, RT_NULL);
    MotorSetOutputInhibit(vehicle_state != AMP_VEHICLE_ENABLED);
    AmpMotorUpdate();
}

static void amp_send_text(const char *text)
{
    char line[256];
    int n;

    if (!text)
        return;
    n = rt_snprintf(line, sizeof(line), "%s\n", text);
    if (n > 0)
        amp_ipc_send(line, n);
}

static void amp_send_can_frame(const char *tag, const struct amp_can_frame *frame)
{
    char msg[128];
    char *p = msg;
    int left = sizeof(msg);
    int n;
    uint8_t i;

    n = rt_snprintf(p, left, "%s id=0x%x ide=%u rtr=%u len=%u data=",
                    tag, frame->id, frame->ide, frame->rtr, frame->len);
    if (n < 0 || n >= left)
        return;
    p += n;
    left -= n;

    for (i = 0; i < frame->len && left > 3; i++) {
        n = rt_snprintf(p, left, "%02x", frame->data[i]);
        if (n < 0 || n >= left)
            break;
        p += n;
        left -= n;
    }

    amp_send_text(msg);
}

static void amp_send_rs485_data(const uint8_t *data, int len)
{
    char msg[160];
    char *p = msg;
    int left = sizeof(msg);
    int n;
    int i;

    n = rt_snprintf(p, left, "RS485 len=%d data=", len);
    if (n < 0 || n >= left)
        return;
    p += n;
    left -= n;

    for (i = 0; i < len && left > 3; i++) {
        n = rt_snprintf(p, left, "%02x", data[i]);
        if (n < 0 || n >= left)
            break;
        p += n;
        left -= n;
    }

    amp_send_text(msg);
}

static int fixed1000(float v)
{
    if (v >= 0.0f)
        return (int)(v * 1000.0f + 0.5f);
    return (int)(v * 1000.0f - 0.5f);
}

static int fixed100(float v)
{
    if (v >= 0.0f)
        return (int)(v * 100.0f + 0.5f);
    return (int)(v * 100.0f - 0.5f);
}

static void amp_send_dm_feedback(void)
{
    DM_Motor_Measure_s measure;
    struct amp_can_frame last_rx = {0};
    uint32_t count, age_ms, reply_count, reply_value;
    float torque_output;
    unsigned int disabled, confirmed, reported, pending;
    char msg[256];

    if (!dm_motor)
        return;

    /* CAN decoding runs in the motor thread; copy one consistent sample. */
    rt_enter_critical();
    measure = dm_motor->measure;
    count = dm_motor->feedback_count;
    age_ms = BSPTimeMillis() - dm_motor->last_feedback_ms;
    disabled = dm_motor->stop_flag == MOTOR_STOP;
    confirmed = dm_motor->control_mode_confirmed;
    reported = dm_motor->reported_control_mode;
    pending = dm_motor->register_pending_command;
    reply_count = dm_motor->register_reply_count;
    reply_value = dm_motor->register_reply_value;
    torque_output = dm_motor->torque_output;
    last_rx.id = dm_motor->motor_can_instace->rx_id;
    last_rx.len = dm_motor->motor_can_instace->rx_len;
    memcpy(last_rx.data, dm_motor->motor_can_instace->rx_buff, sizeof(last_rx.data));
    rt_exit_critical();

    rt_snprintf(msg, sizeof(msg),
                "DM can=1 id=0x6 rx=0x16 disabled=%u online=%u count=%u age_ms=%u "
                "state=0x%x mode=%u confirmed=%u reg_count=%u reg_value=0x%x pending=0x%x torque_cmd_mNm=%d",
                disabled, count != 0U && age_ms <= DM_MOTOR_FEEDBACK_TIMEOUT_MS,
                count, age_ms, measure.state, reported, confirmed,
                reply_count, reply_value, pending, fixed1000(torque_output));
    amp_send_text(msg);

    /* Actual P_MAX/V_MAX/T_MAX are configurable on the drive. Preserve raw
     * replies until those settings have been verified for this geared motor. */
    if (last_rx.len != 0U)
        amp_send_can_frame("DM_RX_LAST", &last_rx);
}

static void amp_send_can_diagnostic(int bus)
{
    struct amp_can_diagnostic diag;
    CAN_Diagnostic_s software = {0};
    char msg[256];

    if (amp_can_get_diagnostic(bus, &diag) != RT_EOK)
        return;

    rt_snprintf(msg, sizeof(msg),
                "CAN_DIAG v=1 bus=%d ms=%u mode=0x%x cmd=0x%x state=0x%x "
                "irq=0x%x err=0x%x rec=%u tec=%u nbtp=0x%x id=0x%x fic=0x%x",
                bus, diag.sample_ms, diag.mode, diag.cmd, diag.state,
                diag.irq, diag.error, diag.rx_error, diag.tx_error,
                diag.nbtp, diag.tx_id, diag.tx_fic);
    amp_send_text(msg);
    rt_snprintf(msg, sizeof(msg),
                "CAN_COUNT v=1 bus=%d ms=%u submit=%u complete=%u busy=%u "
                "err_events=%u busoff=%u recover=%u",
                bus, diag.sample_ms, diag.tx_submitted, diag.tx_completed,
                diag.tx_busy, diag.error_events, diag.bus_off_events,
                diag.recovery_attempts);
    amp_send_text(msg);
    rt_snprintf(msg, sizeof(msg),
                "CAN_FAULT v=1 bus=%d ms=%u last_err=0x%x last_state=0x%x "
                "last_rec=%u last_tec=%u",
                bus, diag.sample_ms, diag.last_error, diag.last_state,
                diag.last_rx_error, diag.last_tx_error);
    amp_send_text(msg);
    CANGetDiagnostic(bus == 1 ? &bsp_can1 : &bsp_can2, &software);
    rt_snprintf(msg, sizeof(msg),
                "CAN_RX v=1 bus=%d ms=%u total=%u unmatched_drop=%u "
                "tx_accepted=%u tx_rejected=%u",
                bus, diag.sample_ms, software.rx_count, software.unmatched_dropped,
                software.tx_success_count, software.tx_failure_count);
    amp_send_text(msg);
}

static void amp_send_atom_imu(void)
{
    struct atom_imu_sample s;
    char msg[256];

    if (atom_imu_read(&s) != RT_EOK || s.last_ms == 0)
        return;

    rt_snprintf(msg, sizeof(msg),
                "ATOM_IMU accl_mg=%d,%d,%d gyro_mrads=%d,%d,%d pitch_mrad=%d roll_mrad=%d yaw_mrad=%d n=%u,%u,%u,%u age_ms=%u online=%u lost=%u err=%u",
                fixed1000(s.accel[0]), fixed1000(s.accel[1]), fixed1000(s.accel[2]),
                fixed1000(s.gyro[0]), fixed1000(s.gyro[1]), fixed1000(s.gyro[2]),
                fixed1000(s.eulr[0]), fixed1000(s.eulr[1]), fixed1000(s.eulr[2]),
                s.accel_count, s.gyro_count, s.eulr_count, s.quat_count,
                (uint32_t)(BSPTimeMillis() - s.last_ms), s.online, s.lost_count,
                s.error_count);
    amp_send_text(msg);
}

/* Diagnostic echo is handled before the ordinary text receive log. */
static void ipc_rx_thread(void *param)
{
    struct amp_bench_parser parser = {0};
    uint8_t amp_reply[AMP_FRAME_OVERHEAD + AMP_ACK_PAYLOAD_BYTES];
    uint32_t amp_reply_len;

    rt_kprintf("[IPC] RX thread started, AMP RTT echo v1 (64 bytes)\n");

    amp_ipc_send("HELLO_FROM_CPU3\n", 16);

    while (1) {
        uint8_t buf[256];
        int len = amp_ipc_recv(buf, sizeof(buf) - 1);

        if (len > 0) {
            int i;
            int benchmark = parser.used != 0;
            int amp_frame = 0;

            for (i = 0; i < len; i++) {
                int amp_event = amp_application_feed(
                    &amp_application, buf[i], BSPTimeMillis(), amp_reply,
                    sizeof(amp_reply), &amp_reply_len);
                if (amp_event == AMP_APP_EVENT_PARTIAL) {
                    amp_frame = 1;
                    continue;
                }
                if (amp_event == AMP_APP_EVENT_ACK_READY) {
                    if (amp_reply_len != 0U)
                        amp_ipc_send(amp_reply, amp_reply_len);
                    amp_frame = 0;
                    continue;
                }
                if (amp_frame) {
                    /* The parser consumed a malformed partial frame. Do not
                     * reinterpret its tail as a diagnostic text command. */
                    amp_frame = 0;
                    continue;
                }
                if (amp_bench_feed(&parser, buf[i], AMP_BENCH_REQUEST)) {
                    parser.frame[AMP_BENCH_MAGIC_SIZE - 1] = '!';
                    amp_ipc_send(parser.frame, AMP_BENCH_SIZE);
                    benchmark = 1;
                }
                if (parser.used)
                    benchmark = 1;
            }
            if (benchmark)
                continue;
            buf[len] = '\0';
            rt_kprintf("[IPC RX] '%s' (%d bytes)\n", buf, len);
        }
    }
}

/* ─── Peripheral framework thread ─── */
#ifdef AMP_CAN_LOOPBACK_TEST
static void amp_run_can_loopback_tests(void)
{
    struct amp_can_loopback_result result;
    char msg[256];
    int bus, rc;

    for (bus = 1; bus <= 2; bus++) {
        rc = amp_can_loopback_test(bus, &result);
        rt_snprintf(msg, sizeof(msg),
                    "CAN_LOOPBACK bus=%d result=%s sent=%u received=%u complete=%u "
                    "mismatch=%u timeout=%u err_events=%u busoff=%u",
                    bus, rc == RT_EOK ? "PASS" : "FAIL", result.sent,
                    result.received, result.during.tx_completed,
                    result.mismatched, result.timeouts,
                    result.during.error_events, result.during.bus_off_events);
        amp_send_text(msg);
        rt_snprintf(msg, sizeof(msg),
                    "CAN_LOOPBACK_REG bus=%d mode=0x%x state=0x%x err=0x%x "
                    "rec=%u tec=%u nbtp=0x%x rxfc=0x%x restored_mode=0x%x",
                    bus, result.during.mode, result.during.state,
                    result.during.error, result.during.rx_error,
                    result.during.tx_error, result.during.nbtp,
                    result.rxfc, result.normal_mode);
        amp_send_text(msg);
    }
}
#endif

static void periph_thread(void *param)
{
    struct amp_can_frame frame;
    struct amp_imu_sample imu;
    uint8_t rs485_buf[64];
    uint32_t tick = 0;
    uint32_t can1_rx = 0;
        uint32_t can2_rx = 0;
        uint32_t rs485_rx = 0;
        int imu_ok = 0;
        uint8_t acc_id = 0xff, gyro_id = 0xff;
        uint32_t spi_ok = 0, spi_to = 0;

        amp_bmi088_raw_ids(&acc_id, &gyro_id);
        amp_spi3_stats(&spi_ok, &spi_to);

    rt_kprintf("[PERIPH] init start\n");
    amp_periph_init();
    global_eso_init(&global_eso, RT_NULL);
    if (atom_imu_init(1, ATOM_IMU_F_EULR | ATOM_IMU_F_GYRO) != RT_EOK)
        rt_kprintf("[ATOM-IMU] CAN1 EULR/GYRO register failed\n");
#ifdef AMP_CAN_LOOPBACK_TEST
    amp_run_can_loopback_tests();
#endif
    MotorApplicationInit();
    if (MotorServiceInit() != RT_EOK)
        rt_kprintf("[MOTOR] service startup failed\n");
    rt_kprintf("[PERIPH] init done, bmi088_err=0x%02x\n", amp_bmi088_last_error());

    while (1) {
        int rs485_len;
        amp_application_tick(&amp_application, BSPTimeMillis());
        AMP_DIAG_SET(AMP_DIAG_PERIPH_STEP, 1);

        while (CANReceiveUnmatched(1, &frame) > 0) {
            can1_rx++;
            amp_send_can_frame("CAN1", &frame);
        }
        while (CANReceiveUnmatched(2, &frame) > 0) {
            can2_rx++;
            amp_send_can_frame("CAN2", &frame);
        }

        AMP_DIAG_SET(AMP_DIAG_PERIPH_STEP, 2);
        rs485_len = rt_hw_rs485_recv(rs485_buf, sizeof(rs485_buf));
        if (rs485_len > 0) {
            rs485_rx += rs485_len;
            amp_send_rs485_data(rs485_buf, rs485_len);
        }

        AMP_DIAG_SET(AMP_DIAG_PERIPH_STEP, 3);
        imu_ok = (amp_bmi088_read(&imu) == RT_EOK);
        amp_update_global_eso();

        tick++;
        AMP_DIAG_SET(AMP_DIAG_PERIPH, tick);
        AMP_DIAG_SET(AMP_DIAG_PERIPH_STEP, 4);
        if ((tick % 100) == 0)
            amp_send_dm_feedback();
        if ((tick % 200) == 0) {
            char msg[256];
            struct MotorTiming timing;
            struct amp_dma_stats dma;
            struct amp_uart4_stats uart;
            struct amp_spi3_diagnostic spi;

            if (global_eso_output_valid) {
                rt_snprintf(msg, sizeof(msg),
                            "ESO valid=1 d_mrad_s2=%d,%d,%d",
                            (int)(global_eso_output.disturbance[0] * 1000.0),
                            (int)(global_eso_output.disturbance[1] * 1000.0),
                            (int)(global_eso_output.disturbance[2] * 1000.0));
            } else {
                rt_snprintf(msg, sizeof(msg), "ESO valid=0");
            }
            amp_send_text(msg);

            /* Linux may prepare SPI clocks after the first RT boot attempt,
             * or the sensor may be powered later. Retry SPI only, never CAN. */
            if (amp_bmi088_last_error()) {
                amp_pwm14_set(0);
                if (amp_spi3_init() == RT_EOK) amp_pwm14_set(300);
            }

            amp_dma_get_stats(&dma);
            amp_uart4_get_stats(&uart);
            amp_spi3_get_diagnostic(&spi);
            amp_bmi088_raw_ids(&acc_id, &gyro_id);
            amp_spi3_stats(&spi_ok, &spi_to);
            rt_snprintf(msg, sizeof(msg),
                        "RT_DMA ready=%u spi_tx=%u spi_rx=%u uart_tx=%u uart_rx=%u "
                        "timeout=%u fault=%u stop_fail=%u",
                        dma.granted, dma.completed[0], dma.completed[1],
                        dma.completed[2], dma.completed[3],
                        dma.timeouts, dma.faults, dma.stop_failures);
            amp_send_text(msg);
            rt_snprintf(msg, sizeof(msg),
                        "RT_IO spi_dma=%u spi_pio=%u spi_err=%u spi_max_ticks=%u "
                        "uart_tx_dma=%u uart_rx_dma=%u uart_drop=%u uart_err=%u",
                        spi.dma, spi.pio, spi.errors, spi.max_ticks,
                        uart.tx_dma, uart.rx_dma, uart.rx_dropped, uart.line_errors);
            amp_send_text(msg);

            MotorControlTiming(&timing);
            rt_snprintf(msg, sizeof(msg),
                        "MOTOR_TIMING hz=24000000 count=%u period_min=%u period_max=%u "
                        "exec_max=%u period_gt1500us=%u exec_gt1000us=%u",
                        timing.samples, timing.period_min_ticks, timing.period_max_ticks,
                        timing.execution_max_ticks, timing.period_over_1500us,
                        timing.execution_over_1000us);
            amp_send_text(msg);

            amp_send_can_diagnostic(1);
            amp_send_can_diagnostic(2);
            amp_send_atom_imu();

            if (imu_ok) {
                rt_snprintf(msg, sizeof(msg),
                            "STAT tick=%u bmi088=ok ax_mg=%d ay_mg=%d az_mg=%d gx_mrads=%d gy_mrads=%d gz_mrads=%d temp_centi=%d can1_rx=%u can2_rx=%u rs485_rx=%u",
                            tick,
                            fixed1000(imu.accel[0]), fixed1000(imu.accel[1]), fixed1000(imu.accel[2]),
                            fixed1000(imu.gyro[0]), fixed1000(imu.gyro[1]), fixed1000(imu.gyro[2]),
                            fixed100(imu.temperature),
                            can1_rx, can2_rx, rs485_rx);
            } else {
                rt_snprintf(msg, sizeof(msg),
                            "STAT tick=%u bmi088=err err=0x%02x acc_id=0x%02x gyro_id=0x%02x spi_ok=%u spi_to=%u can1_rx=%u can2_rx=%u rs485_rx=%u",
                            tick, amp_bmi088_last_error(), acc_id, gyro_id,
                            spi_ok, spi_to, can1_rx, can2_rx, rs485_rx);
            }

            amp_send_text(msg);
        }

        AMP_DIAG_SET(AMP_DIAG_PERIPH_STEP, 0);
        rt_thread_mdelay(5);
    }
}

/* ─── Heartbeat thread ─── */
static void heartbeat_thread(void *param)
{
    uint32_t count = 0;
    while (1) {
        char msg[48];

        rt_thread_mdelay(5000);
        count++;
        AMP_DIAG_SET(AMP_DIAG_HEARTBEAT, count);
        rt_snprintf(msg, sizeof(msg), "HEARTBEAT cpu3=%u", count);
        amp_send_text(msg);
        rt_kprintf("[CPU3] alive: %u\n", count);
    }
}

/* ─── Main ─── */
int main(void)
{
    amp_application_init(&amp_application);
    rt_kprintf("\n");
    rt_kprintf("==========================================\n");
    rt_kprintf("  RT-Thread on RK3568 CPU3 (AMP)\n");
    rt_kprintf("  IPC: shared memory + mailbox, debug over AMP\n");
    rt_kprintf("  RT owns CAN1/CAN2/UART4-RS485/SPI3/PWM14\n");
    rt_kprintf("  APP=motor-can1-recheck-20261102 build=%s %s\n", __DATE__, __TIME__);
    rt_kprintf("==========================================\n\n");

    rt_thread_t periph = rt_thread_create(
        "periph",
        periph_thread, RT_NULL,
        16384,
        10,
        20
    );
    if (periph)
        rt_thread_startup(periph);

    rt_thread_t ipc = rt_thread_create(
        "ipc_rx",
        ipc_rx_thread, RT_NULL,
        16384, /* stack */
        9,     /* below motor (8), above ordinary peripherals (10) */
        20     /* timeslice */
    );
    if (ipc)
        rt_thread_startup(ipc);

    /* Start heartbeat thread */
    rt_thread_t hb = rt_thread_create(
        "heartbeat",
        heartbeat_thread, RT_NULL,
        8192,
        20,
        20
    );
    if (hb)
        rt_thread_startup(hb);

    /* Never return: if main() exits, the thread is queued to the defunct
     * list and the cleanup path crashes on this board. Keep the main thread
     * alive forever instead. */
    while (1)
        rt_thread_mdelay(1000);
}
