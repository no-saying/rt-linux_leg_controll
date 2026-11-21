#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4
#define DM_MOTOR_FEEDBACK_TIMEOUT_MS 200U
#define DM_REGISTER_REPLY_TIMEOUT_MS 200U
#define DM_TORQUE_LOOP_PERIOD_MS 1U
#define DM_SPEED_LOOP_PERIOD_MS 10U
/* DM-S2325 manual V1.1: protocol mapping, independent of mechanical limits. */
#define DM2325_PMAX_RAD 12.5f
#define DM2325_VMAX_RADPS 200.0f
#define DM2325_TMAX_NM 20.0f
#define DM2325_MECHANICAL_SPEED_RADPS (560.0f * 2.0f * 3.14159265359f / 60.0f)
#define DM2325_PEAK_TORQUE_NM 5.0f

/* DM 电机型号 */
typedef enum
{
    DM4310 = 0,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DM2325,
    DM_MOTOR_TYPE_NUM
} DM_Motor_Type_e;

/* 电机控制模式 (寄存器 RID=10) */
typedef enum
{
    DM_CTRL_MIT       = 1,  // MIT 模式: 位置+速度+扭矩+Kp+Kd
    DM_CTRL_POS_VEL   = 2,  // 位置-速度模式: float pos + float vel, CAN ID += 0x100
    DM_CTRL_VEL       = 3,  // 速度模式: float vel, CAN ID += 0x200
    DM_CTRL_POS_FORCE = 4   // 位置-力模式, CAN ID += 0x300
} DM_Control_Mode_e;

/* 电机状态命令 (使能/停止/归零/清除错误)。
   命令帧 ID 与当前控制模式一致：MIT=CAN_ID，位置速度=CAN_ID+0x100，
   速度=CAN_ID+0x200。 */
typedef enum
{
    DM_CMD_CLEAR_ERROR  = 0xfb,
    DM_CMD_MOTOR_MODE   = 0xfc,  // 使能
    DM_CMD_RESET_MODE   = 0xfd,  // 停止
    DM_CMD_ZERO_POSITION = 0xfe  // 编码器归零
} DM_Motor_Cmd_e;

/* Protocol mapping ranges; not mechanical speed/torque ratings. */
typedef struct
{
    float p_max;   // 位置最大值 (rad)
    float v_max;   // 速度最大值 (rad/s)
    float t_max;   // 扭矩最大值 (Nm)
} DM_Motor_Limit_s;

/* 电机测量值 */
typedef struct
{
    uint8_t id;
    volatile uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;      // MOS 温度, 部分固件版本可能为保留字节
    float T_Rotor;    // 转子温度, 部分固件版本可能为保留字节
    int32_t total_round;
    double total_position; // Unwrapped output-shaft displacement in rad, relative to first feedback.
} DM_Motor_Measure_s;

/* MIT 模式控制报文 */
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/* All references use output-shaft SI units: rad, rad/s and Nm.  The legacy
 * CURRENT_LOOP/current_PID slots implement a torque-feedback correction;
 * DM feedback does not expose phase current. The drive retains its FOC. */
typedef struct
{
    float torque_limit;
    float stopped_speed;
} DMMotor_Torque_Config_s;

/* Stable J-Link ABI. The host writes request_seq last. Values are copied and
 * validated by the motor task only while disabled; no motion is requested. */
#define DM_TUNING_VERSION 0x444D0001U
typedef struct
{
    uint32_t version;
    uint32_t request_seq;
    uint32_t applied_seq;
    uint32_t status; /* 0=idle, 1=applied, 2=invalid, 3=motor not disabled. */
    float speed_kp;
    float speed_ki;
    float speed_i_limit;
    float torque_kp;
    float torque_ki;
    float torque_i_limit;
    float torque_limit;
    uint32_t cascade; /* 0=speed PI; 1=speed PI + torque PI correction. */
} DMMotor_Tuning_s;

/* 电机实例 */
typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    volatile Motor_Working_Type_e stop_flag; // Requested state, independent of feedback.
    volatile uint8_t feedback_online;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    DM_Motor_Type_e dm_motor_type;
    DM_Motor_Limit_s limits;
    DM_Control_Mode_e control_mode;
    uint32_t can_id;
    uint32_t lost_cnt;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_ms;
    DMMotor_Torque_Config_s torque_config;
    volatile uint32_t control_tx_count;
    uint8_t native_mit_active, native_position_active, urgent_disable;
    uint32_t mit_period_ms, native_period_ms;
    float native_position, native_velocity, native_torque, native_kp, native_kd;
    volatile uint8_t control_inhibit;
    int8_t last_ref_direction;
    volatile float speed_ref;
    /* Outer-loop command held between 100 Hz speed-loop updates. */
    volatile float speed_torque_ref;
    volatile float torque_ref;
    volatile float torque_output;
    volatile uint32_t speed_loop_last_ms;
    volatile uint32_t speed_loop_count;
    volatile uint32_t torque_loop_count;
    volatile uint8_t speed_loop_initialized;
    volatile DMMotor_Tuning_s *tuning;
    /* One outstanding parameter operation. Management replies never refresh
     * motion feedback. Raw data preserves both uint32 and float registers. */
    volatile uint8_t register_pending_command;
    volatile uint8_t register_pending_rid;
    volatile uint32_t register_request_ms;
    volatile uint32_t register_reply_value;
    volatile uint32_t register_reply_count;
    volatile uint8_t control_mode_confirmed;
    volatile uint8_t reported_control_mode;
    uint8_t mode_request_sent;
    uint32_t mode_request_ms;
    volatile uint32_t mode_confirmed_ms;
} DMMotorInstance;

extern const DM_Motor_Limit_s dm_motor_limits[DM_MOTOR_TYPE_NUM];
extern volatile uint32_t dm_decode_count;
extern volatile uint32_t dm_decode_reject_count;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
unsigned int DMMotorCount(void);
DMMotorInstance *DMMotorAt(unsigned int index);
/* Nonblocking native setpoints. Caller confirms mode and validates app policy. */
int DMMotorSetNativeMIT(DMMotorInstance *, float p, float v, float t, float kp, float kd);
int DMMotorSetNativePosition(DMMotorInstance *, float p, float vmax);
void DMMotorForceDisable(DMMotorInstance *);

void DMMotorSetDMType(DMMotorInstance *motor, DM_Motor_Type_e type);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

void DMMotorSetTorqueConfig(DMMotorInstance *motor,
                            const DMMotor_Torque_Config_s *config);
/* Inhibit keeps feedback polling alive but sends strictly zero torque. */
void DMMotorSetControlInhibit(DMMotorInstance *motor, uint8_t inhibit);
void DMMotorResetControl(DMMotorInstance *motor);
float DMMotorCalculateTorque(DMMotorInstance *motor);
void DMMotorAttachTuning(DMMotorInstance *motor, volatile DMMotor_Tuning_s *tuning);
void DMMotorApplyTuning(DMMotorInstance *motor);
/* Execute one control/send iteration; return the next task delay in ms. */
uint32_t DMMotorControlStep(DMMotorInstance *motor);

/* Requests are applied asynchronously by the motor task; init defaults to disabled. */
void DMMotorEnable(DMMotorInstance *motor);

void DMMotorDisable(DMMotorInstance *motor);

/* Compatibility alias for DMMotorDisable(). */
void DMMotorStop(DMMotorInstance *motor);

void DMMotorCaliEncoder(DMMotorInstance *motor);

void DMMotorControlInit(void);
void DMMotorControl(void);

void DMMotorSetMode(DM_Motor_Cmd_e cmd, DMMotorInstance *motor);

void DMMotorSetControlMode(DMMotorInstance *motor, DM_Control_Mode_e mode);

float DMMotorReadRegister(DMMotorInstance *motor, uint8_t rid);
/* Read requests are asynchronous. Check reply_count for completion and
 * interpret register_reply_value using the register's documented type. */

void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t rid, const uint8_t data[4]);

void DMMotorSaveParams(DMMotorInstance *motor);

/* Override limits when the motor firmware/configuration differs from defaults. */
void DMMotorSetLimits(DMMotorInstance *motor, const DM_Motor_Limit_s *limits);

#endif // !DMMOTOR_H
