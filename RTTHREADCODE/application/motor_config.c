#include "amp_motor_config.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "motor_config.generated.h"

int AmpMotorRegisterConfigured(void)
{
    int failures=0;
    for (uint32_t i=0;i<amp_motor_config_count;i++) {
        const struct amp_motor_config *c=&amp_motor_config[i];
        Motor_Init_Config_s init={.can_init_config={
            .can_handle=c->bus==1 ? &bsp_can1 : &bsp_can2,.tx_id=c->can_id,.rx_id=c->feedback_id}};
        if (c->family==1) {
            DMMotorInstance *m=DMMotorInit(&init);
            if (!m) { failures++;continue; }
            DMMotorSetDMType(m,(DM_Motor_Type_e)c->driver_model);
            DM_Motor_Limit_s limits={c->pmax,c->vmax,c->tmax};DMMotorSetLimits(m,&limits);
            DMMotor_Torque_Config_s torque={c->max_torque,.05f};DMMotorSetTorqueConfig(m,&torque);
            m->mit_period_ms=c->mit_period_ms;m->native_period_ms=c->native_period_ms;
            DMMotorDisable(m);
        } else {
            init.motor_type=(Motor_Type_e)c->driver_model;
            init.controller_setting_init_config.outer_loop_type=SPEED_LOOP;
            init.controller_setting_init_config.close_loop_type=(c->speed_output>0 ? SPEED_LOOP : 0)|
                (c->position_output>0 ? ANGLE_LOOP : 0);
            init.controller_param_init_config.speed_PID=(PID_Init_Config_s){
                .Kp=c->speed_kp,.Ki=c->speed_ki,.Kd=c->speed_kd,
                .MaxOut=c->speed_output,.MaxOut_=-c->speed_output};
            init.controller_param_init_config.angle_PID=(PID_Init_Config_s){
                .Kp=c->position_kp,.MaxOut=c->position_output,.MaxOut_=-c->position_output};
            DJIMotorInstance *m=DJIMotorInit(&init);
            if (!m) { failures++;continue; }
            DJIMotorSetPeriod(m,c->mit_period_ms);DJIMotorStop(m);
        }
    }
    return failures ? -failures : 0;
}
