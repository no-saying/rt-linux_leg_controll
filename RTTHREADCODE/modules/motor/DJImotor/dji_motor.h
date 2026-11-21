/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DJI_MOTOR_CNT 12
#define DJI_MOTOR_FEEDBACK_TIMEOUT_MS 20U

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/* C620/M3508 current and torque constants.
 *
 * The C620 command range is +/-16384 counts = +/-20 A torque current. The
 * 0.3 Nm/A value is the gearbox output-shaft torque constant. For power
 * calculations using the motor-side RPM feedback, divide it by the actual
 * gearbox ratio to obtain the rotor-side constant. */
#define DJI_M3508_REDUCTION_RATIO 20.3175f
#define DJI_M3508_CURRENT_MAX_A   20.0f
#define DJI_M3508_CURRENT_COMMAND_MAX 16384.0f
#define DJI_M3508_OUTPUT_TORQUE_CONSTANT_NM_PER_A 0.3f
#define DJI_M3508_ROTOR_TORQUE_CONSTANT_NM_PER_A \
    (DJI_M3508_OUTPUT_TORQUE_CONSTANT_NM_PER_A / DJI_M3508_REDUCTION_RATIO)
#define DJI_M3508_MAX_TORQUE_NM   3.0f

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 输出轴单圈角度，单位：度
    float speed_aps;          // 输出轴角速度，单位：度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 输出轴总角度，单位：度
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    int16_t set_value; // 发送的设定值,注意是电流值,不是PWM值

    Motor_Type_e motor_type;        // 电机类型
    volatile Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    volatile uint32_t control_tx_count;
    uint32_t feed_cnt;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_ms;
    float dt;
} DJIMotorInstance;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            ID冲突、配置无效或内存不足时返回NULL,应用必须检查返回值.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
unsigned int DJIMotorCount(void);
DJIMotorInstance *DJIMotorAt(unsigned int index);
void DJIMotorSetPeriod(DJIMotorInstance *, uint32_t ms);
float DJIMotorOutputReductionRatio(Motor_Type_e type);
float DJIMotorMaxTorqueNm(const DJIMotorInstance *motor);
/* Convert the C620 raw current command/feedback to torque. The rotor-side
 * value is for power calculations with rotor RPM; output-side torque is the
 * mechanical torque after the gearbox and is used by AMP feedback. */
float DJIMotorCurrentCommandToRotorTorqueNm(Motor_Type_e type, float command);
float DJIMotorCurrentCommandToOutputTorqueNm(Motor_Type_e type, float command);
float DJIMotorOutputTorqueToCurrentCommand(Motor_Type_e type, float torque_nm);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);


/// @brief 
/// @param motor 
/// @param ref 
void DJIMotorSetValue(DJIMotorInstance *motor, float ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 由motor_task统一调度,不应再由其他线程重复调用.
 */
void DJIMotorControl(void);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化默认停止,应用确认配置和反馈后显式调用此函数.
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        使用DJIMotorSetValue设置直接输出,仍受反馈超时和输出限幅保护.
 *
 */
void DJIMotorEnablePower(DJIMotorInstance *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

#endif // !DJI_MOTOR_H
