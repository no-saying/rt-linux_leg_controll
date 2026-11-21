#include "dmmotor.h"
#include <string.h>
#include <rtthread.h>
#include "motor_task.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "controller.h"
#include "motor_def.h"
#include <math.h>

/* Protocol mapping ranges (p_max, v_max, t_max). */
const DM_Motor_Limit_s dm_motor_limits[DM_MOTOR_TYPE_NUM] = {
    {12.5f, 30.0f,  10.0f},   // DM4310
    {12.5f, 50.0f,  10.0f},   // DM4310_48V
    {12.5f, 10.0f,  28.0f},   // DM4340
    {12.5f, 10.0f,  28.0f},   // DM4340_48V
    {12.5f, 45.0f,  12.0f},   // DM6006
    {12.5f, 45.0f,  20.0f},   // DM8006
    {12.5f, 45.0f,  54.0f},   // DM8009
    {12.5f, 25.0f, 200.0f},   // DM10010L
    {12.5f, 20.0f, 200.0f},   // DM10010
    {12.5f, 280.0f,  1.0f},   // DMH3510
    {12.5f, 45.0f,  10.0f},   // DMH6215
    {12.5f, 45.0f,  10.0f},   // DMG6220
    /* Manual defaults; the chassis overrides these with measured settings. */
    {DM2325_PMAX_RAD, DM2325_VMAX_RADPS, DM2325_TMAX_NM} // DM2325
};

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static uint32_t next_control_ms[DM_MOTOR_CNT];
static uint8_t control_scheduled[DM_MOTOR_CNT];
unsigned int DMMotorCount(void) { return idx; }
DMMotorInstance *DMMotorAt(unsigned int index) { return index < idx ? dm_motor_instance[index] : NULL; }
void DMMotorForceDisable(DMMotorInstance *motor)
{
    if (!motor) return;
    DMMotorSetRef(motor, 0); DMMotorDisable(motor); DMMotorResetControl(motor);
    motor->urgent_disable=1;
    for (unsigned int i=0; i<idx; i++)
        if (dm_motor_instance[i]==motor) control_scheduled[i]=0;
}
int DMMotorSetNativeMIT(DMMotorInstance *m, float p, float v, float t, float kp, float kd)
{
    if (!m || !isfinite(p) || !isfinite(v) || !isfinite(t) || !isfinite(kp) || !isfinite(kd) ||
        fabsf(p)>m->limits.p_max || fabsf(v)>m->limits.v_max || fabsf(t)>m->limits.t_max ||
        kp<0 || kp>500 || kd<0 || kd>5) return 0;
    m->native_position=p; m->native_velocity=v; m->native_torque=t;
    m->native_kp=kp; m->native_kd=kd; m->native_mit_active=1;
    return 1;
}
int DMMotorSetNativePosition(DMMotorInstance *m, float p, float vmax)
{
    if (!m || !isfinite(p) || !isfinite(vmax) || fabsf(p)>m->limits.p_max ||
        vmax<=0 || vmax>m->limits.v_max) return 0;
    DMMotorSetRef(m,p); m->native_velocity=vmax; m->native_position_active=1;
    return 1;
}
volatile uint32_t dm_decode_count;
volatile uint32_t dm_decode_reject_count;

static float DMMotorPhysicalTorqueLimit(const DMMotorInstance *motor)
{
    return motor->dm_motor_type == DM2325
        ? fminf(motor->limits.t_max, DM2325_PEAK_TORQUE_NM)
        : motor->limits.t_max;
}

static float DMMotorPhysicalSpeedLimit(const DMMotorInstance *motor)
{
    return motor->dm_motor_type == DM2325
        ? fminf(motor->limits.v_max, DM2325_MECHANICAL_SPEED_RADPS)
        : motor->limits.v_max;
}

/* float → uint 线性映射 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* uint → float 线性映射 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static uint16_t clamp_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    if (x <= x_min)
        return 0;
    if (x >= x_max)
        return (uint16_t)((1u << bits) - 1u);
    return float_to_uint(x, x_min, x_max, bits);
}

static uint32_t DMMotorControlFrameId(const DMMotorInstance *motor,
                                    DM_Control_Mode_e mode)
{
    if (motor == NULL)
        return 0u;

    /* DM-S2325 command frames use a mode-dependent ID offset.  This also
       applies to enable/disable/clear/zero commands (manual V1.1). */
    switch (mode)
    {
    case DM_CTRL_POS_VEL:
        return motor->can_id + 0x100u;
    case DM_CTRL_VEL:
        return motor->can_id + 0x200u;
    case DM_CTRL_POS_FORCE:
        return motor->can_id + 0x300u;
    case DM_CTRL_MIT:
    default:
        return motor->can_id;
    }
}

static uint8_t DMMotorFeedbackFresh(const DMMotorInstance *motor)
{
    return motor != NULL && motor->feedback_online != 0U &&
           motor->feedback_count != 0U &&
           BSPTimeMillis() - motor->last_feedback_ms <=
               DM_MOTOR_FEEDBACK_TIMEOUT_MS;
}

static void DMMotorSetModeForControl(DM_Motor_Cmd_e cmd, DMMotorInstance *motor,
                                    DM_Control_Mode_e mode)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return;
    CANInstance command = *motor->motor_can_instace;
    uint32_t command_id = DMMotorControlFrameId(motor, mode);
    command.tx_id = command_id;
    command.tx_len = 8;
    memset(command.tx_buff, 0xff, 7);
    command.tx_buff[7] = (uint8_t)cmd;
    if (CANTransmit(&command, 1)) motor->control_tx_count++;
}

void DMMotorSetMode(DM_Motor_Cmd_e cmd, DMMotorInstance *motor)
{
    if (motor != NULL)
        DMMotorSetModeForControl(cmd, motor, motor->control_mode);
}

void DMMotorSetDMType(DMMotorInstance *motor, DM_Motor_Type_e type)
{
    if (motor == NULL)
        return;
    if ((unsigned int)type >= DM_MOTOR_TYPE_NUM)
        type = DM4310;
    rt_enter_critical();
    motor->dm_motor_type = type;
    motor->limits = dm_motor_limits[type];
    rt_exit_critical();
}

void DMMotorSetLimits(DMMotorInstance *motor, const DM_Motor_Limit_s *limits)
{
    if (motor != NULL && limits != NULL &&
        isfinite(limits->p_max) && isfinite(limits->v_max) && isfinite(limits->t_max) &&
        limits->p_max > 0.0f && limits->v_max > 0.0f && limits->t_max > 0.0f)
    {
        rt_enter_critical();
        motor->limits = *limits;
        rt_exit_critical();
    }
}

/* 解析 MIT 模式反馈帧 (bytes 1-5: position/velocity/torque) */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);
    DM_Motor_Limit_s *lim = &(motor->limits);

    /* Parameter echoes share MST_ID with motion feedback. Match a pending
     * transaction (full ID, opcode, RID and deadline), not byte 2 alone: a
     * valid position byte may also equal 0x33/0x55/0xAA. */
    if (motor->register_pending_command != 0U &&
        BSPTimeMillis() - motor->register_request_ms > DM_REGISTER_REPLY_TIMEOUT_MS)
        motor->register_pending_command = 0U;
    if (motor_can->rx_len == 8U && motor->register_pending_command != 0U &&
        ((uint32_t)rxbuff[0] | ((uint32_t)rxbuff[1] << 8)) == motor->can_id &&
        rxbuff[2] == motor->register_pending_command &&
        rxbuff[3] == motor->register_pending_rid)
    {
        uint32_t value;
        memcpy(&value, rxbuff + 4, sizeof(value));
        motor->register_reply_value = value;
        motor->register_reply_count++;
        motor->register_pending_command = 0U;
        if (rxbuff[3] == 10U && (rxbuff[2] == 0x33U || rxbuff[2] == 0x55U))
        {
            motor->reported_control_mode = value >= DM_CTRL_MIT && value <= DM_CTRL_POS_FORCE
                ? (uint8_t)value : 0U;
            motor->control_mode_confirmed = value == (uint32_t)motor->control_mode;
            motor->mode_confirmed_ms = BSPTimeMillis();
        }
        return;
    }

    if (motor_can->rx_len < 8u || (rxbuff[0] & 0x0fu) != (motor->can_id & 0x0fu))
    {
        dm_decode_reject_count++;
        return;
    }

    DaemonReload(motor->motor_daemon);

    measure->id = (uint8_t)(rxbuff[0] & 0x0fu);
    measure->state = (uint8_t)(rxbuff[0] >> 4);
    motor->feedback_online = 1u;

    measure->last_position = measure->position;

    /* 位置 16-bit (bytes 1-2) */
    uint16_t tmp16 = (uint16_t)(rxbuff[1] << 8) | rxbuff[2];
    measure->position = uint_to_float(tmp16, -lim->p_max, lim->p_max, 16);
    if (motor->feedback_count != 0u)
    {
        float delta = measure->position - measure->last_position;
        if (delta > lim->p_max)
        {
            delta -= 2.0f * lim->p_max;
            measure->total_round--;
        }
        else if (delta < -lim->p_max)
        {
            delta += 2.0f * lim->p_max;
            measure->total_round++;
        }
        measure->total_position += delta;
    }

    /* 速度 12-bit (byte 3 全8位 + byte 4 高4位) */
    tmp16 = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp16, -lim->v_max, lim->v_max, 12);

    /* 扭矩 12-bit (byte 4 低4位 + byte 5 全8位) */
    tmp16 = (uint16_t)((rxbuff[4] & 0x0f) << 8) | rxbuff[5];
    measure->torque = uint_to_float(tmp16, -lim->t_max, lim->t_max, 12);

    /* bytes 6-7: 温度, 部分固件版本可能为保留字节 */
    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    motor->last_feedback_ms = BSPTimeMillis();
    motor->feedback_count++;
    dm_decode_count++;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    if (motor != NULL)
    {
        if (motor->feedback_online)
        {
            motor->feedback_online = 0u;
            motor->control_mode_confirmed = 0U;
            motor->reported_control_mode = 0U;
        }
        motor->lost_cnt++;
    }
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    rt_thread_mdelay(100);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    if (config == NULL || idx >= DM_MOTOR_CNT ||
        config->can_init_config.tx_id == 0U || config->can_init_config.tx_id > 0x7ffU)
        return NULL;

    for (unsigned int i = 0; i < idx; i++) {
        if (dm_motor_instance[i]->motor_can_instace->can_handle == config->can_init_config.can_handle &&
            (dm_motor_instance[i]->can_id == config->can_init_config.tx_id ||
             (dm_motor_instance[i]->motor_can_instace->rx_id == config->can_init_config.rx_id &&
              (dm_motor_instance[i]->can_id & 0xfU) == (config->can_init_config.tx_id & 0xfU))))
            return NULL;
    }

    DMMotorInstance *motor = (DMMotorInstance *)rt_calloc(1, sizeof(DMMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    if (motor->motor_can_instace == NULL)
    {
        rt_free(motor);
        return NULL;
    }
    motor->can_id = config->can_init_config.tx_id;

    /* Default to the DM-2325 used by this project. */
    DMMotorSetDMType(motor, DM2325);
    motor->control_mode = DM_CTRL_MIT;
    motor->torque_config = (DMMotor_Torque_Config_s){
        .torque_limit = DMMotorPhysicalTorqueLimit(motor),
        .stopped_speed = 0.05f,
    };

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        /* Daemon runs every 10 ms. Frame freshness is checked separately;
         * paired-wheel interlocks belong to the application. */
        .reload_count = 50,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    if (motor->motor_daemon == NULL) {
        CANUnregister(motor->motor_can_instace);
        rt_free(motor);
        return NULL;
    }

    motor->stop_flag = MOTOR_STOP;
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor != NULL) {
        motor->pid_ref = isfinite(ref) ? ref : 0.0f;
        motor->native_mit_active=0; motor->native_position_active=0;
    }
}

void DMMotorEnable(DMMotorInstance *motor)
{
    if (motor != NULL && !MotorOutputsInhibited())
        motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorDisable(DMMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_STOP;
}

void DMMotorStop(DMMotorInstance *motor)
{
    DMMotorDisable(motor);
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    if (motor != NULL)
        motor->motor_settings.outer_loop_type = type;
}

void DMMotorSetTorqueConfig(DMMotorInstance *motor,
                            const DMMotor_Torque_Config_s *config)
{
    if (motor == NULL || config == NULL ||
        !isfinite(config->torque_limit) ||
        !isfinite(config->stopped_speed) ||
        config->torque_limit <= 0.0f ||
        config->torque_limit > DMMotorPhysicalTorqueLimit(motor) || config->stopped_speed < 0.0f)
        return;
    motor->torque_config = *config;
}

void DMMotorSetControlInhibit(DMMotorInstance *motor, uint8_t inhibit)
{
    if (motor != NULL)
        motor->control_inhibit = inhibit != 0U;
}

void DMMotorResetControl(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;
    PIDReset(&motor->angle_PID);
    PIDReset(&motor->speed_PID);
    PIDReset(&motor->current_PID);
    motor->last_ref_direction = 0;
    motor->speed_ref = 0.0f;
    motor->speed_torque_ref = 0.0f;
    motor->torque_ref = 0.0f;
    motor->torque_output = 0.0f;
    motor->speed_loop_last_ms = 0U;
    motor->speed_loop_initialized = 0U;
}

static float DMMotorClampTorque(float torque, float limit)
{
    return fmaxf(-limit, fminf(limit, torque));
}

static float DMMotorPIDCalculate(PIDInstance *pid, float measure, float ref,
                                  float dt)
{
    return isfinite(dt) ? PIDCalculateWithDt(pid, measure, ref, dt)
                        : PIDCalculate(pid, measure, ref);
}

/* Calculate one inner-loop sample.  The speed/angle cascade is updated only
 * when speed_update is set; its torque command is held for the intervening
 * 1 kHz torque-loop samples. */
static float DMMotorCalculateTorqueTimed(DMMotorInstance *motor,
                                         uint8_t speed_update,
                                         float speed_dt, float torque_dt)
{
    float ref;
    float angle;
    float speed;
    float torque;
    float limit;
    uint8_t speed_loop;
    Motor_Control_Setting_s *setting;
    DMMotor_Torque_Config_s *config;

    if (motor == NULL)
        return 0.0f;
    if (motor->stop_flag == MOTOR_STOP || motor->control_inhibit ||
        motor->control_mode != DM_CTRL_MIT || !DMMotorFeedbackFresh(motor))
        goto reset;

    /* total_position is a double updated by the CAN ISR on a 32-bit MCU. */
    rt_enter_critical();
    angle = (float)motor->measure.total_position;
    speed = motor->measure.velocity;
    torque = motor->measure.torque;
    rt_exit_critical();
    ref = motor->pid_ref;
    setting = &motor->motor_settings;
    config = &motor->torque_config;
    if (!isfinite(ref) || !isfinite(angle) || !isfinite(speed) ||
        !isfinite(torque) || !isfinite(config->torque_limit) ||
        config->torque_limit <= 0.0f)
        goto reset;
    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        ref = -ref;

    /* Same cascade selection as DJIMotorControl(). With SPEED_LOOP outermost,
     * position PID is bypassed even when ALL_THREE_LOOP is configured. */
    if (setting->outer_loop_type == ANGLE_LOOP && speed_update)
    {
        if (!(setting->close_loop_type & ANGLE_LOOP))
            goto reset;
        if (setting->angle_feedback_source == OTHER_FEED)
        {
            if (motor->other_angle_feedback_ptr == NULL)
                goto reset;
            angle = *motor->other_angle_feedback_ptr;
        }
        if (!isfinite(angle))
            goto reset;
        ref = DMMotorPIDCalculate(&motor->angle_PID, angle, ref, speed_dt);
    }
    speed_loop = (setting->close_loop_type & SPEED_LOOP) &&
        (setting->outer_loop_type == ANGLE_LOOP ||
         setting->outer_loop_type == SPEED_LOOP);
    if ((setting->outer_loop_type == SPEED_LOOP ||
         setting->outer_loop_type == ANGLE_LOOP) && !speed_loop)
        goto reset;

    limit = fminf(config->torque_limit, DMMotorPhysicalTorqueLimit(motor));
    if (speed_loop && speed_update)
    {
        if (setting->feedforward_flag & SPEED_FEEDFORWARD)
        {
            if (motor->speed_feedforward_ptr == NULL)
                goto reset;
            ref += *motor->speed_feedforward_ptr;
        }
        if (setting->speed_feedback_source == OTHER_FEED)
        {
            if (motor->other_speed_feedback_ptr == NULL)
                goto reset;
            speed = *motor->other_speed_feedback_ptr;
        }
        if (!isfinite(ref) || !isfinite(speed))
            goto reset;
        float speed_limit = DMMotorPhysicalSpeedLimit(motor);
        ref = fmaxf(-speed_limit, fminf(speed_limit, ref));
        motor->speed_ref = ref;
        int8_t direction = ref > 1.0e-5f ? 1 : ref < -1.0e-5f ? -1 : 0;
        if (direction != motor->last_ref_direction)
        {
            PIDReset(&motor->speed_PID);
            PIDReset(&motor->current_PID);
            motor->last_ref_direction = direction;
        }
        if (direction == 0 && fabsf(speed) <= config->stopped_speed)
            goto reset;

        motor->speed_PID.MaxOut = limit;
        motor->speed_PID.MaxOut_ = -limit;
        motor->speed_torque_ref = DMMotorPIDCalculate(&motor->speed_PID,
                                                       speed, ref, speed_dt);
    }
    else if (!speed_loop)
    {
        motor->speed_ref = 0.0f;
    }

    if (speed_loop)
        ref = motor->speed_torque_ref;

    if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
    {
        if (motor->current_feedforward_ptr == NULL)
            goto reset;
        ref += *motor->current_feedforward_ptr; /* Nm, not amperes. */
    }
    if (!isfinite(ref))
        goto reset;
    ref = DMMotorClampTorque(ref, limit);
    motor->torque_ref = ref;
    if (setting->close_loop_type & CURRENT_LOOP)
    {
        /* The existing CURRENT_LOOP slot closes on reported torque (Nm).
         * Target feedforward + PI correction preserves the Nm command scale;
         * this is NOT an independent phase-current/FOC loop on the MCU. */
        motor->current_PID.MaxOut = limit - ref;
        motor->current_PID.MaxOut_ = -limit - ref;
        ref += DMMotorPIDCalculate(&motor->current_PID, torque, ref, torque_dt);
    }
    if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        ref = -ref;
    if (!isfinite(ref))
        goto reset;
    motor->torque_output = DMMotorClampTorque(ref, limit);
    if (speed_update)
        motor->speed_loop_count++;
    motor->torque_loop_count++;
    return motor->torque_output;

reset:
    DMMotorResetControl(motor);
    return 0.0f;
}

float DMMotorCalculateTorque(DMMotorInstance *motor)
{
    /* Compatibility entry point: callers that invoke this function directly
     * expect both cascaded loops to run on each call and use measured timing. */
    return DMMotorCalculateTorqueTimed(motor, 1U, NAN, NAN);
}

_Static_assert(sizeof(DMMotor_Tuning_s) == 48, "J-Link tuning ABI changed");

void DMMotorAttachTuning(DMMotorInstance *motor, volatile DMMotor_Tuning_s *tuning)
{
    if (motor == NULL || tuning == NULL)
        return;
    *tuning = (DMMotor_Tuning_s){
        .version = DM_TUNING_VERSION,
        .speed_kp = motor->speed_PID.Kp,
        .speed_ki = motor->speed_PID.Ki,
        .speed_i_limit = motor->speed_PID.IntegralLimit,
        .torque_kp = motor->current_PID.Kp,
        .torque_ki = motor->current_PID.Ki,
        .torque_i_limit = motor->current_PID.IntegralLimit,
        .torque_limit = motor->torque_config.torque_limit,
        .cascade = (motor->motor_settings.close_loop_type & CURRENT_LOOP) != 0,
    };
    motor->tuning = tuning;
}

void DMMotorApplyTuning(DMMotorInstance *motor)
{
    if (motor == NULL || motor->tuning == NULL)
        return;
    DMMotor_Tuning_s request = *motor->tuning;
    if (request.request_seq == request.applied_seq)
        return;
    uint32_t status = 2U;
    if (request.version == DM_TUNING_VERSION &&
        request.request_seq != 0U && request.cascade <= 1U &&
        isfinite(request.speed_kp) && request.speed_kp >= 0.0f && request.speed_kp <= 5.0f &&
        isfinite(request.speed_ki) && request.speed_ki >= 0.0f && request.speed_ki <= 20.0f &&
        isfinite(request.torque_kp) && request.torque_kp >= 0.0f && request.torque_kp <= 1.0f &&
        isfinite(request.torque_ki) && request.torque_ki >= 0.0f && request.torque_ki <= 20.0f &&
        isfinite(request.torque_limit) && request.torque_limit > 0.0f &&
        request.torque_limit <= DMMotorPhysicalTorqueLimit(motor) &&
        isfinite(request.speed_i_limit) && request.speed_i_limit >= 0.0f &&
        request.speed_i_limit <= request.torque_limit &&
        isfinite(request.torque_i_limit) && request.torque_i_limit >= 0.0f &&
        request.torque_i_limit <= request.torque_limit)
    {
        status = 3U;
        rt_enter_critical();
        if (motor->stop_flag == MOTOR_STOP)
        {
            motor->speed_PID.Kp = request.speed_kp;
            motor->speed_PID.Ki = request.speed_ki;
            motor->speed_PID.IntegralLimit = request.speed_i_limit;
            motor->current_PID.Kp = request.torque_kp;
            motor->current_PID.Ki = request.torque_ki;
            motor->current_PID.IntegralLimit = request.torque_i_limit;
            motor->torque_config.torque_limit = request.torque_limit;
            if (request.cascade)
                motor->motor_settings.close_loop_type |= CURRENT_LOOP;
            else
                motor->motor_settings.close_loop_type &= ~CURRENT_LOOP;
            DMMotorResetControl(motor);
            status = 1U;
        }
        rt_exit_critical();
    }
    /* Readback always represents the active settings, also after rejection. */
    motor->tuning->version = DM_TUNING_VERSION;
    motor->tuning->speed_kp = motor->speed_PID.Kp;
    motor->tuning->speed_ki = motor->speed_PID.Ki;
    motor->tuning->speed_i_limit = motor->speed_PID.IntegralLimit;
    motor->tuning->torque_kp = motor->current_PID.Kp;
    motor->tuning->torque_ki = motor->current_PID.Ki;
    motor->tuning->torque_i_limit = motor->current_PID.IntegralLimit;
    motor->tuning->torque_limit = motor->torque_config.torque_limit;
    motor->tuning->cascade = (motor->motor_settings.close_loop_type & CURRENT_LOOP) != 0;
    motor->tuning->status = status;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    motor->tuning->applied_seq = request.request_seq;
}

/* The motor task owns CAN management frames and confirms the actual mode. */
void DMMotorSetControlMode(DMMotorInstance *motor, DM_Control_Mode_e mode)
{
    if (motor == NULL || mode < DM_CTRL_MIT || mode > DM_CTRL_POS_FORCE ||
        DMMotorControlFrameId(motor, mode) > 0x7ffU)
        return;
    rt_enter_critical();
    motor->control_mode = mode;
    motor->control_mode_confirmed = 0U;
    motor->reported_control_mode = 0U;
    motor->mode_request_sent = 0U;
    rt_exit_critical();
}

/* 读电机寄存器 (CAN ID 0x7FF, 命令 0x33) */
float DMMotorReadRegister(DMMotorInstance *motor, uint8_t rid)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return 0.0f;
    CANInstance message = *motor->motor_can_instace;
    CANInstance *can = &message;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0x33;
    tx[3] = rid;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    rt_enter_critical();
    motor->register_request_ms = BSPTimeMillis();
    motor->register_pending_rid = rid;
    motor->register_pending_command = 0x33U;
    rt_exit_critical();

    /* 通过 ID 0x7FF 发送寄存器读命令 */
    can->tx_id = 0x7FF;
    can->tx_len = 8;
    CANTransmit(can, 1);
    return 0.0f;  // Legacy return; asynchronous result is in register_reply_value.
}

/* 写电机寄存器 (CAN ID 0x7FF, 命令 0x55) */
void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t rid, const uint8_t data[4])
{
    if (motor == NULL || motor->motor_can_instace == NULL || data == NULL)
        return;
    CANInstance message = *motor->motor_can_instace;
    CANInstance *can = &message;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0x55;
    tx[3] = rid;
    tx[4] = data[0];
    tx[5] = data[1];
    tx[6] = data[2];
    tx[7] = data[3];
    rt_enter_critical();
    motor->register_request_ms = BSPTimeMillis();
    motor->register_pending_rid = rid;
    motor->register_pending_command = 0x55U;
    rt_exit_critical();

    can->tx_id = 0x7FF;
    can->tx_len = 8;
    CANTransmit(can, 1);
}

/* 保存所有参数到 Flash (CAN ID 0x7FF, 命令 0xAA, RID=0x01) */
void DMMotorSaveParams(DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return;
    DMMotorDisable(motor);
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);  // 先停电机
    rt_thread_mdelay(10);

    CANInstance message = *motor->motor_can_instace;
    CANInstance *can = &message;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0xAA;
    tx[3] = 0x01;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    rt_enter_critical();
    motor->register_request_ms = BSPTimeMillis();
    motor->register_pending_rid = 0x01U;
    motor->register_pending_command = 0xAAU;
    rt_exit_critical();

    can->tx_id = 0x7FF;
    can->tx_len = 8;
    CANTransmit(can, 1);

    rt_thread_mdelay(100);  // 等待写入完成
}

static uint8_t DMMotorPrepareControlMode(DMMotorInstance *motor)
{
    uint32_t now = BSPTimeMillis();
    /* A mode reply is not motion feedback. Allow the first disabled/zero
     * frame after confirmation to obtain motion feedback on a late power-up. */
    if (!DMMotorFeedbackFresh(motor) && motor->control_mode_confirmed &&
        now - motor->mode_confirmed_ms > DM_MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        motor->control_mode_confirmed = 0U;
        DMMotorSetModeForControl(DM_CMD_RESET_MODE, motor,
            motor->reported_control_mode != 0U
                ? (DM_Control_Mode_e)motor->reported_control_mode : motor->control_mode);
        motor->reported_control_mode = 0U;
        motor->mode_request_sent = 0U;
        DMMotorResetControl(motor);
        return 0U;
    }
    if (motor->control_mode_confirmed)
        return 1U;

    DMMotorResetControl(motor);
    /* Keep disable and management requests in separate iterations: the
     * installed drive drops a register request immediately following disable. */
    if (motor->reported_control_mode != 0U && motor->measure.state != 0U)
    {
        DMMotorSetModeForControl(DM_CMD_RESET_MODE, motor,
            (DM_Control_Mode_e)motor->reported_control_mode);
        return 0U;
    }
    if (motor->mode_request_sent &&
        now - motor->mode_request_ms <= DM_REGISTER_REPLY_TIMEOUT_MS)
        return 0U;
    motor->mode_request_sent = 1U;
    motor->mode_request_ms = now;
    if (motor->reported_control_mode != 0U && motor->measure.state == 0U)
    {
        uint8_t data[4] = {(uint8_t)motor->control_mode, 0, 0, 0};
        DMMotorWriteRegister(motor, 10U, data);
    }
    else
        DMMotorReadRegister(motor, 10U);
    return 0U;
}

static void DMMotorTransmitTorque(DMMotorInstance *motor, float torque)
{
    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;
    if (!isfinite(torque) || motor->stop_flag == MOTOR_STOP || motor->control_inhibit)
        torque = 0.0f;
    torque = DMMotorClampTorque(torque,
        fminf(motor->torque_config.torque_limit, DMMotorPhysicalTorqueLimit(motor)));
    uint16_t position = clamp_float_to_uint(0.0f, -motor->limits.p_max, motor->limits.p_max, 16);
    uint16_t velocity = clamp_float_to_uint(0.0f, -motor->limits.v_max, motor->limits.v_max, 12);
    uint16_t command = clamp_float_to_uint(torque, -motor->limits.t_max, motor->limits.t_max, 12);
    /* Native MIT Kp/Kd stay zero: every active loop is calculated on the MCU. */
    tx[0] = (uint8_t)(position >> 8);
    tx[1] = (uint8_t)position;
    tx[2] = (uint8_t)(velocity >> 4);
    tx[3] = (uint8_t)((velocity & 0x0fU) << 4);
    tx[4] = 0U;
    tx[5] = 0U;
    tx[6] = (uint8_t)((command >> 8) & 0x0fU);
    tx[7] = (uint8_t)command;
    can->tx_id = motor->can_id;
    can->tx_len = 8;
    motor->torque_output = torque;
    if (CANTransmit(can, 1)) motor->control_tx_count++;
}

uint32_t DMMotorControlStep(DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return 20U;
    /* Emergency bypasses asynchronous mode negotiation. Unknown mode: disable
     * every supported command ID, never send an enable while inhibited. */
    if (MotorOutputsInhibited() || motor->urgent_disable) {
        motor->urgent_disable=0;
        DMMotorSetRef(motor,0); DMMotorDisable(motor); DMMotorResetControl(motor);
        if (motor->reported_control_mode)
            DMMotorSetModeForControl(DM_CMD_RESET_MODE,motor,
                (DM_Control_Mode_e)motor->reported_control_mode);
        else for (unsigned int mode=1; mode<=3; mode++)
            if (DMMotorControlFrameId(motor,(DM_Control_Mode_e)mode)<=0x7ff)
                DMMotorSetModeForControl(DM_CMD_RESET_MODE,motor,(DM_Control_Mode_e)mode);
        return 20U;
    }
    DMMotorApplyTuning(motor);
    if (!DMMotorPrepareControlMode(motor))
        return 20U;
    if (motor->stop_flag == MOTOR_STOP)
    {
        DMMotorResetControl(motor);
        DMMotorSetMode(DM_CMD_RESET_MODE, motor);
        return 20U;
    }
    if (!DMMotorFeedbackFresh(motor) || motor->control_inhibit)
    {
        DMMotorResetControl(motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        /* A zero-speed PI is a brake, not zero torque. Bypass the controller
         * during feedback recovery, and overwrite any previous torque frame. */
        if (motor->control_mode == DM_CTRL_MIT)
            DMMotorTransmitTorque(motor, 0.0f);
        return 20U;
    }
    if (motor->measure.state != 1U)
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    if (motor->control_mode == DM_CTRL_MIT)
    {
        if (motor->native_mit_active) {
            CANInstance *can=motor->motor_can_instace;
            uint16_t p=clamp_float_to_uint(motor->native_position,-motor->limits.p_max,motor->limits.p_max,16);
            uint16_t v=clamp_float_to_uint(motor->native_velocity,-motor->limits.v_max,motor->limits.v_max,12);
            uint16_t t=clamp_float_to_uint(motor->native_torque,-motor->limits.t_max,motor->limits.t_max,12);
            uint16_t kp=clamp_float_to_uint(motor->native_kp,0,500,12);
            uint16_t kd=clamp_float_to_uint(motor->native_kd,0,5,12);
            can->tx_buff[0]=p>>8; can->tx_buff[1]=p;
            can->tx_buff[2]=v>>4; can->tx_buff[3]=(v<<4)|(kp>>8);
            can->tx_buff[4]=kp; can->tx_buff[5]=kd>>4;
            can->tx_buff[6]=(kd<<4)|(t>>8); can->tx_buff[7]=t;
            can->tx_id=motor->can_id; can->tx_len=8;
            motor->torque_output=motor->native_torque;
            if (CANTransmit(can,1)) motor->control_tx_count++;
            return motor->mit_period_ms ? motor->mit_period_ms : DM_TORQUE_LOOP_PERIOD_MS;
        }
        uint32_t now = BSPTimeMillis();
        uint8_t speed_update = !motor->speed_loop_initialized ||
            (uint32_t)(now - motor->speed_loop_last_ms) >= DM_SPEED_LOOP_PERIOD_MS;
        if (speed_update)
        {
            motor->speed_loop_initialized = 1U;
            motor->speed_loop_last_ms = now;
        }
        DMMotorTransmitTorque(motor,
                              DMMotorCalculateTorqueTimed(motor, speed_update,
                                                           DM_SPEED_LOOP_PERIOD_MS * 0.001f,
                                                           DM_TORQUE_LOOP_PERIOD_MS * 0.001f));
        return motor->mit_period_ms ? motor->mit_period_ms : DM_TORQUE_LOOP_PERIOD_MS;
    }

    /* Retain the other native DM modes for callers that explicitly use them. */
    float ref = motor->pid_ref;
    CANInstance *can = motor->motor_can_instace;
    if (!isfinite(ref))
        ref = 0.0f;
    if (motor->control_mode == DM_CTRL_VEL)
    {
        if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            ref = -ref;
        float speed_limit = DMMotorPhysicalSpeedLimit(motor);
        LIMIT_MIN_MAX(ref, -speed_limit, speed_limit);
        memcpy(can->tx_buff, &ref, sizeof(float));
        memset(can->tx_buff + 4, 0, 4);
        can->tx_len = 4;
    }
    else
    {
        float limit = motor->native_position_active ? motor->native_velocity : motor->limits.v_max;
        if (motor->control_mode == DM_CTRL_POS_FORCE)
        {
            limit = motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE
                ? -ref : ref;
            limit = DMMotorClampTorque(limit, motor->limits.t_max);
        }
        memcpy(can->tx_buff, &ref, sizeof(float));
        memcpy(can->tx_buff + 4, &limit, sizeof(float));
        can->tx_len = 8;
    }
    can->tx_id = DMMotorControlFrameId(motor, motor->control_mode);
    if (CANTransmit(can, 1)) motor->control_tx_count++;
    return motor->native_period_ms ? motor->native_period_ms : 2U;
}

void DMMotorControl(void)
{
    uint32_t now = BSPTimeMillis();
    for (unsigned int i = 0; i < idx; i++) {
        if (!control_scheduled[i] || (int32_t)(now - next_control_ms[i]) >= 0) {
            uint32_t delay = DMMotorControlStep(dm_motor_instance[i]);
            next_control_ms[i] = BSPTimeMillis() + delay;
            control_scheduled[i] = 1U;
        }
    }
}

void DMMotorControlInit(void)
{
    (void)MotorServiceInit();
}
