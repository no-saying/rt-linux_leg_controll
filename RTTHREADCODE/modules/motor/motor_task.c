#include "motor_task.h"
#include "amp_motor_service.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "daemon.h"
#include <rtthread.h>
#include <rthw.h>
#include "amp_diag.h"

static volatile int output_inhibit;
int MotorOutputsInhibited(void) { return output_inhibit; }
void MotorSetOutputInhibit(int inhibit) { output_inhibit = inhibit != 0; }

static rt_thread_t motor_thread;
static uint32_t control_ticks;
static uint32_t daemon_last_ms;
static struct MotorTiming motor_timing;
static uint64_t motor_previous;

void MotorControlTiming(struct MotorTiming *timing)
{
    rt_base_t level;
    if (!timing)
        return;
    level = rt_hw_interrupt_disable();
    *timing = motor_timing;
    rt_hw_interrupt_enable(level);
}

rt_weak void MotorApplicationInit(void) { }
rt_weak void MotorApplicationUpdate(void) { }

void MotorControlTask(void)
{
    uint32_t now;
    uint64_t start = BSPTimeCounter();
    uint64_t elapsed;
    if (motor_timing.samples) {
        elapsed = start - motor_previous;
        if (!motor_timing.period_min_ticks || elapsed < motor_timing.period_min_ticks)
            motor_timing.period_min_ticks = (uint32_t)elapsed;
        if (elapsed > motor_timing.period_max_ticks)
            motor_timing.period_max_ticks = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
        if (elapsed > 36000U)
            motor_timing.period_over_1500us++;
    }
    motor_previous = start;
    AMP_DIAG_SET(AMP_DIAG_MOTOR_STEP, 1);
    AmpMotorBegin();
    CANServicePoll();
    AMP_DIAG_SET(AMP_DIAG_MOTOR_STEP, 2);
    MotorApplicationUpdate();
    AmpMotorBegin();
    DJIMotorControl();
    AmpMotorBegin();
    DMMotorControl();
    AmpMotorFinish();
    now = BSPTimeMillis();
    if ((uint32_t)(now - daemon_last_ms) >= 10U) {
        daemon_last_ms = now;
        DaemonTask();
    }
    control_ticks++;
    AMP_DIAG_SET(AMP_DIAG_MOTOR, control_ticks);
    AMP_DIAG_SET(AMP_DIAG_MOTOR_STEP, 0);
    elapsed = BSPTimeCounter() - start;
    if (elapsed > motor_timing.execution_max_ticks)
        motor_timing.execution_max_ticks = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
    if (elapsed > 24000U)
        motor_timing.execution_over_1000us++;
    motor_timing.samples++;
}

uint32_t MotorControlTaskTicks(void)
{
    return control_ticks;
}

static void motor_entry(void *argument)
{
    (void)argument;
    while (1) {
        rt_tick_t start = rt_tick_get();
        MotorControlTask();
        if (rt_tick_get() == start)
            rt_thread_delay_until(&start, 1);
        else
            rt_thread_mdelay(1);
    }
}

int MotorServiceInit(void)
{
    if (motor_thread)
        return RT_EOK;
    daemon_last_ms = BSPTimeMillis();
    motor_thread = rt_thread_create("motor", motor_entry, RT_NULL, 8192, 8, 1);
    if (!motor_thread)
        return -RT_ENOMEM;
    return rt_thread_startup(motor_thread);
}
