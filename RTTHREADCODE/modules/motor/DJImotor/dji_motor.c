/* Ported from cup_car / HNU YueLu EC (neozng, 2022).
 * See modules/LICENSE.cup_car. DJI angles remain degrees, speeds degrees/s. */
#include "dji_motor.h"
#include "motor_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <math.h>
#include <rtthread.h>
#include <string.h>

static unsigned int idx;
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT];
unsigned int DJIMotorCount(void) { return idx; }
DJIMotorInstance *DJIMotorAt(unsigned int index) { return index < idx ? dji_motor_instance[index] : NULL; }
static CANInstance sender_assignment[6] = {
    {.can_handle = &bsp_can1, .tx_id = 0x1ff, .tx_len = 8},
    {.can_handle = &bsp_can1, .tx_id = 0x200, .tx_len = 8},
    {.can_handle = &bsp_can1, .tx_id = 0x2ff, .tx_len = 8},
    {.can_handle = &bsp_can2, .tx_id = 0x1ff, .tx_len = 8},
    {.can_handle = &bsp_can2, .tx_id = 0x200, .tx_len = 8},
    {.can_handle = &bsp_can2, .tx_id = 0x2ff, .tx_len = 8},
};
static uint8_t sender_enable_flag[6];
static uint32_t group_period_ms[6], group_last_ms[6];
static uint8_t group_scheduled[6];

float DJIMotorOutputReductionRatio(Motor_Type_e type)
{
    return type == M3508 ? DJI_M3508_REDUCTION_RATIO : 1.0f;
}

float DJIMotorMaxTorqueNm(const DJIMotorInstance *motor)
{
    if (!motor)
        return 0.0f;
    return motor->motor_type == M3508 ? DJI_M3508_MAX_TORQUE_NM : 0.0f;
}

float DJIMotorCurrentCommandToRotorTorqueNm(Motor_Type_e type, float command)
{
    if (type != M3508 || !isfinite(command))
        return 0.0f;
    return command * (DJI_M3508_CURRENT_MAX_A /
                      DJI_M3508_CURRENT_COMMAND_MAX) *
           DJI_M3508_ROTOR_TORQUE_CONSTANT_NM_PER_A;
}

float DJIMotorCurrentCommandToOutputTorqueNm(Motor_Type_e type, float command)
{
    if (type != M3508 || !isfinite(command))
        return 0.0f;
    return command * (DJI_M3508_CURRENT_MAX_A /
                      DJI_M3508_CURRENT_COMMAND_MAX) *
           DJI_M3508_OUTPUT_TORQUE_CONSTANT_NM_PER_A;
}

float DJIMotorOutputTorqueToCurrentCommand(Motor_Type_e type, float torque_nm)
{
    if (type != M3508 || !isfinite(torque_nm))
        return 0.0f;
    return torque_nm / DJI_M3508_OUTPUT_TORQUE_CONSTANT_NM_PER_A *
           (DJI_M3508_CURRENT_COMMAND_MAX / DJI_M3508_CURRENT_MAX_A);
}

void DJIMotorSetPeriod(DJIMotorInstance *m, uint32_t ms)
{
    if (m && ms && ms<=1000) group_period_ms[m->sender_group]=ms;
}

static void reset_controller(DJIMotorInstance *motor)
{
    PIDReset(&motor->motor_controller.angle_PID);
    PIDReset(&motor->motor_controller.speed_PID);
    PIDReset(&motor->motor_controller.current_PID);
}

static void DecodeDJIMotor(CANInstance *can)
{
    DJIMotorInstance *motor = can->id;
    DJI_Motor_Measure_s *measure = &motor->measure;
    const uint8_t *data = can->rx_buff;
    float reduction_ratio;
    if (can->rx_len != 8)
        return;
    uint16_t ecd = ((uint16_t)data[0] << 8) | data[1];
    if (ecd > 8191U)
        return;
    measure->last_ecd = motor->feedback_count ? measure->ecd : ecd;
    measure->ecd = ecd;
    reduction_ratio = DJIMotorOutputReductionRatio(motor->motor_type);
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * measure->ecd /
                                  reduction_ratio;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
        RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF *
        (int16_t)((data[2] << 8) | data[3]) / reduction_ratio;
    measure->real_current = (int16_t)((1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
        CURRENT_SMOOTH_COEF * (int16_t)((data[4] << 8) | data[5]));
    measure->temperature = data[6];
    int delta = (int)measure->ecd - measure->last_ecd;
    if (delta > 4096)
        measure->total_round--;
    else if (delta < -4096)
        measure->total_round++;
    measure->total_angle = (measure->total_round * 360.0f +
                            ECD_ANGLE_COEF_DJI * measure->ecd) /
                           reduction_ratio;
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    motor->last_feedback_ms = BSPTimeMillis();
    motor->feedback_count++;
    DaemonReload(motor->daemon);
}

static void DJIMotorLostCallback(void *owner)
{
    DJIMotorInstance *motor = owner;
    DJIMotorStop(motor);
    LOGWARNING("[dji_motor] CAN%u feedback 0x%x offline",
               motor->motor_can_instance->can_handle->bus, motor->motor_can_instance->rx_id);
}

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    CAN_Init_Config_s can;
    unsigned int offset, slot, group, i, limit;
    if (!config || idx >= DJI_MOTOR_CNT)
        return RT_NULL;
    can = config->can_init_config;
    if (can.can_handle == &bsp_can1)
        offset = 0;
    else if (can.can_handle == &bsp_can2)
        offset = 3;
    else
        return RT_NULL;
    if (config->motor_type != GM6020 && config->motor_type != M3508 && config->motor_type != M2006)
        return RT_NULL;
    limit = config->motor_type == GM6020 ? 7 : 8;
    if (can.tx_id < 1 || can.tx_id > limit)
        return RT_NULL;
    slot = (can.tx_id - 1U) % 4U;
    if (config->motor_type == GM6020) {
        group = offset + (can.tx_id <= 4 ? 0 : 2);
        can.rx_id = 0x204U + can.tx_id;
    } else {
        group = offset + (can.tx_id <= 4 ? 1 : 0);
        can.rx_id = 0x200U + can.tx_id;
    }
    for (i = 0; i < idx; i++) {
        CANInstance *other = dji_motor_instance[i]->motor_can_instance;
        if (other->can_handle == can.can_handle && other->rx_id == can.rx_id)
            return RT_NULL;
    }
    DJIMotorInstance *motor = rt_calloc(1, sizeof(*motor));
    if (!motor)
        return RT_NULL;
    motor->motor_type = config->motor_type;
    motor->motor_settings = config->controller_setting_init_config;
    Motor_Controller_s *controller = &motor->motor_controller;
    Motor_Controller_Init_s *init = &config->controller_param_init_config;
    PIDInit(&controller->current_PID, &init->current_PID);
    PIDInit(&controller->speed_PID, &init->speed_PID);
    PIDInit(&controller->angle_PID, &init->angle_PID);
    controller->other_angle_feedback_ptr = init->other_angle_feedback_ptr;
    controller->other_speed_feedback_ptr = init->other_speed_feedback_ptr;
    controller->speed_feedforward_ptr = init->speed_feedforward_ptr;
    controller->current_feedforward_ptr = init->current_feedforward_ptr;
    motor->sender_group = group;
    motor->message_num = slot;
    can.can_module_callback = DecodeDJIMotor;
    can.id = motor;
    motor->motor_can_instance = CANRegister(&can);
    if (!motor->motor_can_instance) {
        rt_free(motor);
        return RT_NULL;
    }
    Daemon_Init_Config_s daemon = {
        .callback = DJIMotorLostCallback, .owner_id = motor, .reload_count = 2,
    };
    motor->daemon = DaemonRegister(&daemon);
    if (!motor->daemon) {
        CANUnregister(motor->motor_can_instance);
        rt_free(motor);
        return RT_NULL;
    }
    motor->stop_flag = MOTOR_STOP;
    sender_enable_flag[group] = 1;
    dji_motor_instance[idx++] = motor;
    return motor;
}

void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if (motor)
        motor->motor_controller.pid_ref = isfinite(ref) ? ref : 0.0f;
}

void DJIMotorSetValue(DJIMotorInstance *motor, float value)
{
    if (motor)
        motor->set_value = isfinite(value) ? (int16_t)fmaxf(-32767.0f, fminf(32767.0f, value)) : 0;
}

void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (!motor)
        return;
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    if (motor) {
        motor->stop_flag = MOTOR_STOP;
        motor->set_value = 0;
        reset_controller(motor);
    }
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    if (motor && !MotorOutputsInhibited())
        motor->stop_flag = MOTOR_ENALBED;
}

void DJIMotorEnablePower(DJIMotorInstance *motor)
{
    if (motor && !MotorOutputsInhibited())
        motor->stop_flag = MOTOR_POWER;
}

void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    if (motor)
        motor->motor_settings.outer_loop_type = outer_loop;
}

static int16_t calculate_output(DJIMotorInstance *motor)
{
    Motor_Controller_s *controller = &motor->motor_controller;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    float ref = controller->pid_ref, measure;
    float limit = motor->motor_type == M2006 ? 10000.0f :
        motor->motor_type == M3508 ? 16384.0f : 30000.0f;
    if (MotorOutputsInhibited() || motor->stop_flag == MOTOR_STOP || !motor->feedback_count ||
        BSPTimeMillis() - motor->last_feedback_ms >= DJI_MOTOR_FEEDBACK_TIMEOUT_MS)
        goto stopped;
    if (motor->stop_flag == MOTOR_POWER) {
        ref = motor->set_value;
        goto output;
    }
    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        ref = -ref;
    if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP) {
        if (setting->angle_feedback_source == OTHER_FEED && !controller->other_angle_feedback_ptr)
            goto stopped;
        measure = setting->angle_feedback_source == OTHER_FEED
            ? *controller->other_angle_feedback_ptr : motor->measure.total_angle;
        if (!isfinite(measure) || !isfinite(ref))
            goto stopped;
        ref = PIDCalculate(&controller->angle_PID, measure, ref);
    }
    if ((setting->close_loop_type & SPEED_LOOP) &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
        if (setting->feedforward_flag & SPEED_FEEDFORWARD) {
            if (!controller->speed_feedforward_ptr)
                goto stopped;
            ref += *controller->speed_feedforward_ptr;
        }
        if (setting->speed_feedback_source == OTHER_FEED && !controller->other_speed_feedback_ptr)
            goto stopped;
        measure = setting->speed_feedback_source == OTHER_FEED
            ? *controller->other_speed_feedback_ptr : motor->measure.speed_aps;
        if (!isfinite(measure) || !isfinite(ref))
            goto stopped;
        ref = PIDCalculate(&controller->speed_PID, measure, ref);
    }
    if (setting->feedforward_flag & CURRENT_FEEDFORWARD) {
        if (!controller->current_feedforward_ptr)
            goto stopped;
        ref += *controller->current_feedforward_ptr;
    }
    if (!isfinite(ref))
        goto stopped;
    if (setting->close_loop_type & CURRENT_LOOP)
        ref = PIDCalculate(&controller->current_PID, motor->measure.real_current, ref);
    else if (setting->close_loop_type & SPEED_LOOP)
        LIMIT_MIN_MAX(ref, controller->speed_PID.MaxOut_, controller->speed_PID.MaxOut);
    if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        ref = -ref;
output:
    if (isfinite(ref))
        return (int16_t)fmaxf(-limit, fminf(limit, ref));
stopped:
    reset_controller(motor);
    return 0;
}

void DJIMotorControl(void)
{
    unsigned int i;
    for (i = 0; i < 6; i++)
        memset(sender_assignment[i].tx_buff, 0, 8);
    for (i = 0; i < idx; i++) {
        DJIMotorInstance *motor = dji_motor_instance[i];
        uint16_t value = (uint16_t)calculate_output(motor);
        uint8_t *data = sender_assignment[motor->sender_group].tx_buff + motor->message_num * 2;
        data[0] = (uint8_t)(value >> 8);
        data[1] = (uint8_t)value;
        motor->set_value = (int16_t)value;
    }
    for (i = 0; i < 6; i++) {
        uint32_t now=BSPTimeMillis(), period=group_period_ms[i] ? group_period_ms[i] : 1U;
        if (sender_enable_flag[i] && (!group_period_ms[i] || !group_scheduled[i] || MotorOutputsInhibited() ||
            now-group_last_ms[i]>=period) && CANTransmit(&sender_assignment[i], 0.1f))
        {
            group_last_ms[i]=now; group_scheduled[i]=1;
            for (unsigned int j=0; j<idx; j++)
                if (dji_motor_instance[j]->sender_group==i)
                    dji_motor_instance[j]->control_tx_count++;
        }
    }
}
