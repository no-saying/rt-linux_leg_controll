/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include <stddef.h>

_Static_assert(offsetof(PIDInstance, Measure) == sizeof(PID_Init_Config_s),
               "PID configuration prefix layout changed");

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}



static void f_Integral_Limit(PIDInstance *pid)
{
    float limit = fabsf(pid->IntegralLimit);
    float candidate_i;
    float delta_i;
    float candidate_output;

    /* Also handle a limit reduced while running. Check the proposed output
     * against BOTH signed limits, including reserved feedforward headroom. */
    pid->Iout = fmaxf(-limit, fminf(limit, pid->Iout));
    candidate_i = fmaxf(-limit, fminf(limit, pid->Iout + pid->ITerm));
    delta_i = candidate_i - pid->Iout;
    candidate_output = pid->Pout + candidate_i + pid->Dout;
    if ((candidate_output > pid->MaxOut && delta_i > 0.0f) ||
        (candidate_output < pid->MaxOut_ && delta_i < 0.0f))
    {
        /* A large reverse error may saturate the opposite bound while it
         * unwinds old integral. Allow that unwinding, but not accumulation
         * beyond zero into the new saturated direction. */
        if (pid->Iout * delta_i < 0.0f)
        {
            if (pid->Iout * candidate_i < 0.0f)
                delta_i = -pid->Iout;
        }
        else
            delta_i = 0.0f;
    }
    pid->ITerm = delta_i;
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    // pid->Dout = pid->Kd * (pid->Last_Output - pid->Output) / pid->dt;
    pid->Dout = pid->Kd != 0.0f
        ? pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt
        : 0.0f;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    float denominator = pid->Derivative_LPF_RC + pid->dt;

    if (denominator <= 0.0f || !isfinite(denominator))
        return;

    pid->Dout = pid->Dout * pid->dt / denominator +
                pid->Last_Dout * pid->Derivative_LPF_RC / denominator;
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    float denominator = pid->Output_LPF_RC + pid->dt;

    if (denominator <= 0.0f || !isfinite(denominator))
        return;

    pid->Output = pid->Output * pid->dt / denominator +
                  pid->Last_Output * pid->Output_LPF_RC / denominator;
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < (pid->MaxOut_))
    {
        pid->Output = (pid->MaxOut_);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 750)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
    // 后续修改为逐个赋值
    // @todo motor->controler->PIDInstance改成指针
    if (pid == NULL || config == NULL)
        return;
    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // set rest of memory to 0
    DWT_GetDeltaT(&pid->DWT_CNT);
}

void PIDReset(PIDInstance *pid)
{
    PID_Init_Config_s config;
    if (pid == NULL)
        return;
    memcpy(&config, pid, sizeof(config));
    PIDInit(pid, &config);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculateWithDt(PIDInstance *pid, float measure, float ref, float dt)
{
    if (pid == NULL)
        return 0.0f;
    if (!isfinite(measure) || !isfinite(ref)) {
        PIDReset(pid);
        return 0.0f;
    }
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle && !pid->ERRORHandler.ERRORType)
        f_PID_ErrorHandle(pid);

    pid->dt = dt;
    if (pid->dt <= 0.0f || pid->dt > 0.1f || !isfinite(pid->dt))
        pid->dt = 0.001f;

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd != 0.0f
            ? pid->Kd * (pid->Err - pid->Last_Err) / pid->dt
            : 0.0f;
        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        if (!isfinite(pid->Iout))
            pid->Iout = 0.0f;
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 已堵转
        if(pid->ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR)
        {
            pid->Output = 0.0f;
            pid->Iout = 0.0f;
        }

        // 输出限幅
        f_Output_Limit(pid);
        if (!isfinite(pid->Output))
        {
            pid->Output = 0.0f;
            pid->Iout = 0.0f;
            pid->Dout = 0.0f;
        }
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    if (pid == NULL)
        return 0.0f;
    float dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次PID计算的时间间隔
    /* A debugger halt or scheduling stall must not be integrated as one
     * giant control interval. */
    if (dt <= 0.0f || dt > 0.01f || !isfinite(dt))
        dt = 0.001f;
    return PIDCalculateWithDt(pid, measure, ref, dt);
}
