/* Linux-commanded output-shaft velocity. Linux owns run duration and STOP.
 * Native velocity is rad/s, DLC=4; no motor parameters are saved to flash. */
#include "dm_motion.h"
#include "bsp_dwt.h"
#include <math.h>
#include <string.h>

static DMMotorInstance *motor;
static volatile struct dm_motion_page *page;
static uint32_t state, error, accepted, seen, phase_ms, run_start, run_ms;
static uint32_t stop_ms, reply_count, parameter_ms;
static unsigned parameter, waiting, started;
static float speed;
static double origin;
static DM_Motor_Limit_s limits;

static int fresh(uint32_t now)
{
    return motor->feedback_online && motor->feedback_count &&
           now - motor->last_feedback_ms <= 100U;
}
static void fail(uint32_t reason, uint32_t now)
{
    if (state == DM_RUNNING) {
        run_ms = now - run_start;
        stop_ms = now;
    }
    DMMotorSetRef(motor, 0);
    DMMotorDisable(motor);
    state = DM_FAILED;
    error = reason;
}
void DMMotionInit(DMMotorInstance *m, volatile struct dm_motion_page *p)
{
    uint32_t epoch = p->magic == DM_MOTION_MAGIC ? p->epoch + 1U : 1U;
    p->magic = 0;
    __sync_synchronize();
    memset((void *)p, 0, sizeof(*p));
    motor = m; page = p;
    state = DM_IDLE; error = accepted = seen = run_ms = run_start = stop_ms = 0;
    parameter = waiting = started = 0;
    speed = 0; origin = m->measure.total_position;
    p->epoch = epoch ? epoch : 1U;
    __sync_synchronize();
    p->magic = DM_MOTION_MAGIC;
}
static void publish(uint32_t now)
{
    page->status_seq++;
    __sync_synchronize();
    page->accepted_seq = accepted;
    page->state = state; page->error = error;
    page->now_ms = now;
    page->run_ms = state == DM_RUNNING ? now - run_start : run_ms;
    page->feedback_age_ms = now - motor->last_feedback_ms;
    page->drive_state = motor->measure.state;
    page->mode = motor->reported_control_mode;
    page->confirmed = motor->control_mode_confirmed;
    page->feedback_count = motor->feedback_count;
    page->disabled = motor->stop_flag == MOTOR_STOP;
    page->command_speed = state == DM_RUNNING ? speed : 0;
    page->measured_speed = motor->measure.velocity;
    page->displacement = started ?
        (float)(motor->measure.total_position - origin) : 0;
    page->p_max = motor->limits.p_max;
    page->v_max = motor->limits.v_max;
    page->t_max = motor->limits.t_max;
    page->start_ms = run_start; page->stop_ms = stop_ms;
    __sync_synchronize();
    page->status_seq++;
}
void DMMotionPoll(void)
{
    uint32_t now, seq;
    if (!motor || !page) return;
    now = BSPTimeMillis();
    seq = page->request_seq;
    __sync_synchronize();
    if (seq != seen) {
        uint32_t epoch = page->request_epoch, command = page->command;
        float requested_speed = page->requested_speed;
        seen = seq;
        /* Old-boot commands can never start or change an active test. */
        if (epoch == page->epoch) {
            if (command == DM_MOTION_STOP) {
                accepted = seq;
                if (state == DM_RUNNING) {
                    run_ms = now - run_start; stop_ms = now;
                }
                DMMotorSetRef(motor, 0); DMMotorDisable(motor);
                DMMotorSetControlMode(motor, DM_CTRL_MIT);
                state = DM_RESTORE; phase_ms = now; error = DM_OK;
            } else if (command == DM_MOTION_START && (state == DM_IDLE || state == DM_DONE)) {
                accepted = seq; error = DM_OK; run_ms = run_start = stop_ms = 0; started = 0;
                if (!isfinite(requested_speed) || fabsf(requested_speed) > DM_MOTION_MAX_SPEED ||
                    requested_speed == 0)
                    fail(DM_BAD_REQUEST, now);
                else if (!fresh(now)) fail(DM_OFFLINE, now);
                else if (motor->measure.state > 1U) fail(DM_DRIVE_FAULT, now);
                else {
                    speed = requested_speed;
                    DMMotorSetRef(motor, 0); DMMotorDisable(motor);
                    state = DM_READ_LIMITS; phase_ms = now;
                    parameter = waiting = 0; limits = motor->limits;
                }
            }
        }
    }
    /* The drive pauses motion feedback during mode management (~240 ms).
     * Setup remains disabled/zero-speed with its own handshake timeout.
     * Apply the tight feedback-loss check only while reading or running. */
    if (state >= DM_READ_LIMITS && state <= DM_RESTORE) {
        if ((state == DM_READ_LIMITS || state == DM_RUNNING) && !fresh(now))
            fail(DM_OFFLINE, now);
        else if (motor->measure.state > 1U) fail(DM_DRIVE_FAULT, now);
    }
    switch (state) {
    case DM_READ_LIMITS:
        if (now - phase_ms > 5000U) { fail(DM_SETUP_TIMEOUT, now); break; }
        if (!motor->control_mode_confirmed || motor->measure.state != 0U) break;
        /* DM parameter map: RID21/22/23 = PMAX/VMAX/TMAX, IEEE float.
         * Serialize with driver mode negotiation; never write these ranges. */
        if (!waiting) {
            reply_count = motor->register_reply_count;
            DMMotorReadRegister(motor, (uint8_t)(21U + parameter));
            parameter_ms = now; waiting = 1;
        } else if (motor->register_reply_count != reply_count) {
            uint32_t bits = motor->register_reply_value;
            float value; memcpy(&value, &bits, sizeof(value));
            if (!isfinite(value) || value <= 0 || value > 10000) {
                fail(DM_PARAMETER_ERROR, now); break;
            }
            if (parameter == 0) limits.p_max = value;
            if (parameter == 1) limits.v_max = value;
            if (parameter == 2) limits.t_max = value;
            waiting = 0;
            if (++parameter == 3) {
                if (fabsf(speed) > limits.v_max) { fail(DM_PARAMETER_ERROR, now); break; }
                DMMotorSetLimits(motor, &limits);
                DMMotorSetControlMode(motor, DM_CTRL_VEL);
                phase_ms = now; state = DM_SET_MODE;
            }
        } else if (now - parameter_ms > 250U) fail(DM_PARAMETER_ERROR, now);
        break;
    case DM_SET_MODE:
        if (now - phase_ms > 5000U) { fail(DM_SETUP_TIMEOUT, now); break; }
        if (motor->control_mode_confirmed && motor->reported_control_mode == DM_CTRL_VEL &&
            motor->measure.state == 0U && fresh(now)) {
            DMMotorEnable(motor); phase_ms = now; state = DM_ARMING;
        }
        break;
    case DM_ARMING:
        if (now - phase_ms > 2000U) { fail(DM_SETUP_TIMEOUT, now); break; }
        if (motor->control_mode_confirmed && motor->reported_control_mode == DM_CTRL_VEL &&
            motor->measure.state == 1U && fresh(now) && now - phase_ms >= 100U) {
            origin = motor->measure.total_position;
            run_start = now; state = DM_RUNNING; started = 1;
            DMMotorSetRef(motor, speed);
        }
        break;
    case DM_RUNNING:
        if (!motor->control_mode_confirmed || motor->reported_control_mode != DM_CTRL_VEL ||
            motor->measure.state != 1U) { fail(DM_DRIVE_FAULT, now); break; }
        /* No RT run timer: motion continues until Linux sends STOP. */
        break;
    case DM_RESTORE:
        if (now - phase_ms > 5000U) { fail(DM_SETUP_TIMEOUT, now); break; }
        if (motor->control_mode_confirmed && motor->reported_control_mode == DM_CTRL_MIT &&
            motor->measure.state == 0U && fresh(now)) state = DM_DONE;
        break;
    default: break;
    }
    publish(now);
}
