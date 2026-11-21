/* One motor-task owner. No allocation, sleeps, printf, or Linux-held locks. */
#include "amp_motor_service.h"
#include "amp_motor_config.h"
#include "motor_task.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "bsp_dwt.h"
#include <math.h>
#include <string.h>
#define DEG_PER_RAD 57.29577951308232f
#define NO_JOB AMP_MOTOR_QUEUE
#define ACK_CONTROLLER_ACCEPTED 1U
#define ACK_DRIVE_CONFIRMED 2U
struct motor_entry {
    const struct amp_motor_config *cfg;
    DMMotorInstance *dm;
    DJIMotorInstance *dji;
    struct amp_motor_command command;
    uint32_t slot, item, phase, start_ms, tx_before, feedback_before, reply_before;
    int32_t last_result;
};
static struct motor_entry motor_entries[AMP_MOTOR_MAX];
static volatile struct amp_motor_shared *shared;
static uint32_t count, generation, estop_seen, disable_seen, tick_count;
static uint32_t status_last, feedback_last;
static unsigned force_status;
static uint32_t motor_feedback_last[AMP_MOTOR_MAX];

static int find(uint32_t number)
{
    for (uint32_t i=0;i<count;i++) if (motor_entries[i].cfg->number==number) return (int)i;
    return -1;
}
static uint32_t feedback_count(const struct motor_entry *e)
{ return e->dm ? e->dm->feedback_count : e->dji ? e->dji->feedback_count : 0; }
static uint32_t age(const struct motor_entry *e, uint32_t now)
{ return e->dm ? now-e->dm->last_feedback_ms : e->dji ? now-e->dji->last_feedback_ms : UINT32_MAX; }
static int online(const struct motor_entry *e, uint32_t now)
{ return feedback_count(e) && age(e,now)<e->cfg->offline_ms; }
static uint32_t tx_count(const struct motor_entry *e)
{ return e->dm ? e->dm->control_tx_count : e->dji ? e->dji->control_tx_count : 0; }
static void disable(struct motor_entry *e)
{
    if (e->dm) DMMotorForceDisable(e->dm);
    if (e->dji) { DJIMotorSetRef(e->dji,0); DJIMotorStop(e->dji); }
}
static void disable_all(void)
{ for (uint32_t i=0;i<count;i++) disable(&motor_entries[i]); }
static void resolve_slots(void)
{
    for (uint32_t s=0;s<AMP_MOTOR_QUEUE;s++) {
        volatile struct amp_motor_slot *p=&shared->slot[s];
        uint32_t seq=p->published_seq;
        if (p->accepted_seq!=seq || p->completed_seq==seq) continue;
        uint32_t n=p->count;
        if (!n || n>AMP_MOTOR_BATCH) continue;
        int32_t result=AMP_OK; unsigned pending=0;
        for (uint32_t j=0;j<n;j++) {
            int32_t r=p->item_result[j];
            if (r==AMP_PENDING) pending=1;
            else if (r<0 && result==AMP_OK) result=r;
        }
        if (!pending) {
            p->result=result;
            __sync_synchronize(); p->completed_seq=seq;
        }
    }
}
static void finish_job(struct motor_entry *e, int32_t result, uint32_t reply)
{
    if (e->slot>=AMP_MOTOR_QUEUE) return;
    volatile struct amp_motor_slot *p=&shared->slot[e->slot];
    p->reply[e->item]=reply; p->item_result[e->item]=result;
    e->slot=NO_JOB; e->last_result=result;
    if (result<0) disable(e);

}
static void cancel_all(void)
{
    disable_all();
    for (uint32_t i=0;i<count;i++) {
        if (motor_entries[i].slot<AMP_MOTOR_QUEUE) finish_job(&motor_entries[i],AMP_ERR_CANCELED,0);
    }
    /* Cancel only slots actually published by Linux (never touch unpublished
     * producer memory). Commands racing this snapshot fail generation later. */
    uint32_t consumer=shared->consumer, producer=shared->producer;
    __sync_synchronize();
    if (producer-consumer<=AMP_MOTOR_QUEUE) {
        while (consumer!=producer) {
            volatile struct amp_motor_slot *p=&shared->slot[consumer%AMP_MOTOR_QUEUE];
            p->result=AMP_ERR_CANCELED;
            uint32_t n=p->count>AMP_MOTOR_BATCH ? AMP_MOTOR_BATCH : p->count;
            for (uint32_t j=0;j<n;j++) p->item_result[j]=AMP_ERR_CANCELED;
            p->accepted_seq=p->published_seq;
            __sync_synchronize();p->completed_seq=p->published_seq;
            consumer++;
        }
        __sync_synchronize(); shared->consumer=consumer;
    }
    resolve_slots();
}
void AmpMotorInit(volatile struct amp_motor_shared *p)
{
    uint32_t session=p->magic==AMP_MOTOR_MAGIC ? p->session+1U : 1U;
    p->magic=0; __sync_synchronize();
    memset((void *)p,0,sizeof(*p)); memset(motor_entries,0,sizeof(motor_entries));
    memset(motor_feedback_last,0,sizeof(motor_feedback_last));
    shared=p; count=amp_motor_config_count;
    if (count>AMP_MOTOR_MAX) count=AMP_MOTOR_MAX;
    generation=1; estop_seen=disable_seen=tick_count=status_last=feedback_last=0;
    force_status=1; MotorSetOutputInhibit(0);
    for (uint32_t i=0;i<count;i++) {
        struct motor_entry *e=&motor_entries[i]; e->cfg=&amp_motor_config[i];e->slot=NO_JOB;
        if (e->cfg->family==AMP_MOTOR_DM) {
            for (unsigned j=0;j<DMMotorCount();j++) {
                DMMotorInstance *m=DMMotorAt(j);
                if (m->can_id==e->cfg->can_id && m->motor_can_instace->can_handle->bus==e->cfg->bus)
                    e->dm=m;
            }
        } else {
            for (unsigned j=0;j<DJIMotorCount();j++) {
                DJIMotorInstance *m=DJIMotorAt(j);
                if (m->motor_can_instance->rx_id==e->cfg->feedback_id &&
                    m->motor_can_instance->can_handle->bus==e->cfg->bus) e->dji=m;
            }
        }
        e->last_result=(e->dm || e->dji) ? AMP_OK : AMP_ERR_NOT_FOUND;
    }
    disable_all();
    p->version=AMP_MOTOR_VERSION;p->bytes=sizeof(*p);p->session=session ? session : 1U;
    p->generation=generation;p->motor_count=count;p->status_hz=amp_motor_status_hz;
    p->feedback_hz=amp_motor_feedback_hz;p->config_hash=amp_motor_config_hash;
    __sync_synchronize(); p->magic=AMP_MOTOR_MAGIC;
}
void AmpMotorBegin(void)
{
    if (!shared) return;
    uint32_t estop=shared->estop_request, all=shared->disable_request;
    __sync_synchronize();
    if (estop!=estop_seen || all!=disable_seen) {
        if (estop!=estop_seen) MotorSetOutputInhibit(1);
        generation++; shared->generation=generation;
        cancel_all();
        shared->estop_latched=MotorOutputsInhibited();
        __sync_synchronize();
        shared->estop_ack=estop;shared->disable_ack=all;
        estop_seen=estop;disable_seen=all;
    }
}
static PIDInstance *pid_for(struct motor_entry *e, uint32_t loop)
{
    if (!e->dji) return NULL;
    if (loop==CURRENT_LOOP) return &e->dji->motor_controller.current_PID;
    if (loop==SPEED_LOOP) return &e->dji->motor_controller.speed_PID;
    if (loop==ANGLE_LOOP) return &e->dji->motor_controller.angle_PID;
    return NULL;
}
static int32_t validate(struct motor_entry *e, const struct amp_motor_command *c, uint32_t now)
{
    if (!e->dm && !e->dji) return AMP_ERR_NOT_FOUND;
    if (c->op<AMP_OP_ENABLE || c->op>AMP_OP_PID) return AMP_ERR_UNSUPPORTED;
    for (unsigned j=0;j<6;j++) if (!isfinite(c->value[j])) return AMP_ERR_ARGUMENT;
    if (e->slot!=NO_JOB) return AMP_ERR_BUSY;
    if (MotorOutputsInhibited() && c->op!=AMP_OP_DISABLE) return AMP_ERR_ESTOP;
    if (c->op==AMP_OP_DISABLE) return AMP_OK;
    if (!online(e,now)) return AMP_ERR_OFFLINE;
    if (e->dm && e->dm->measure.state>1U && c->op!=AMP_OP_CLEAR_ERROR) return AMP_ERR_DRIVE;
    switch (c->op) {
    case AMP_OP_ENABLE: return AMP_OK;
    case AMP_OP_VELOCITY:
        if (fabsf(c->value[0])>e->cfg->max_velocity) return AMP_ERR_ARGUMENT;
        if (e->dji && (!(e->dji->motor_settings.close_loop_type&SPEED_LOOP) ||
            e->dji->motor_controller.speed_PID.MaxOut<=0)) return AMP_ERR_UNSUPPORTED;
        return AMP_OK;
    case AMP_OP_POSITION:
        if (e->dm && (fabsf(c->value[0])>e->dm->limits.p_max ||
            c->value[1]<=0 || c->value[1]>e->cfg->max_velocity)) return AMP_ERR_ARGUMENT;
        if (e->dji && (!(e->dji->motor_settings.close_loop_type&ANGLE_LOOP) ||
            e->dji->motor_controller.angle_PID.MaxOut<=0)) return AMP_ERR_UNSUPPORTED;
        return AMP_OK;
    case AMP_OP_TORQUE:
        if (e->dm)
            return fabsf(c->value[0])>e->cfg->max_torque ? AMP_ERR_ARGUMENT : AMP_OK;
        if (e->dji) {
            float limit=e->cfg->max_torque>0.0f ? e->cfg->max_torque :
                DJIMotorMaxTorqueNm(e->dji);
            return limit<=0.0f || fabsf(c->value[0])>limit ? AMP_ERR_ARGUMENT : AMP_OK;
        }
        return AMP_ERR_UNSUPPORTED;
    case AMP_OP_MIT:
        if (!e->dm) return AMP_ERR_UNSUPPORTED;
        if (fabsf(c->value[0])>e->dm->limits.p_max || fabsf(c->value[1])>e->cfg->max_velocity ||
            fabsf(c->value[2])>e->cfg->max_torque || c->value[3]<0 || c->value[3]>500 ||
            c->value[4]<0 || c->value[4]>5) return AMP_ERR_ARGUMENT;
        return AMP_OK;
    case AMP_OP_CURRENT:
        if (!e->dji) return AMP_ERR_UNSUPPORTED;
        return fabsf(c->value[0])>(e->dji->motor_type==M2006 ? 10000 :
            e->dji->motor_type==M3508 ? 16384 : 30000) ? AMP_ERR_ARGUMENT : AMP_OK;
    case AMP_OP_PID:
        if (!pid_for(e,c->arg)) return AMP_ERR_UNSUPPORTED;
        if (e->dji->stop_flag!=MOTOR_STOP) return AMP_ERR_DISABLED_REQUIRED;
        for (unsigned j=0;j<5;j++) if (c->value[j]<0 || c->value[j]>1e6f) return AMP_ERR_ARGUMENT;
        return c->value[4]>0 ? AMP_OK : AMP_ERR_ARGUMENT;
    default:
        if (!e->dm) return AMP_ERR_UNSUPPORTED;
        if (e->dm->stop_flag!=MOTOR_STOP) return AMP_ERR_DISABLED_REQUIRED;
        if (!e->dm->control_mode_confirmed || e->dm->register_pending_command) return AMP_ERR_BUSY;
        if (c->op==AMP_OP_MODE) return c->arg>=1 && c->arg<=3 ? AMP_OK : AMP_ERR_UNSUPPORTED;
        if ((c->op==AMP_OP_READ_REGISTER || c->op==AMP_OP_WRITE_REGISTER) && c->arg>255) return AMP_ERR_ARGUMENT;
        /* Routing, bitrate and mode are configured/negotiated, never silently
         * changed behind the fixed inventory. Read remains available. */
        if (c->op==AMP_OP_WRITE_REGISTER && c->arg>=7 && c->arg<=10) return AMP_ERR_UNSUPPORTED;
        return AMP_OK;
    }
}
static void start_job(struct motor_entry *e, const struct amp_motor_command *c,
                      uint32_t slot, uint32_t item, uint32_t now)
{
    e->command=*c;e->slot=slot;e->item=item;e->phase=0;e->start_ms=now;
    e->tx_before=tx_count(e);e->feedback_before=feedback_count(e);
    if (c->op==AMP_OP_DISABLE) { disable(e);e->phase=2;return; }
    if (c->op==AMP_OP_PID) {
        PID_Init_Config_s cfg={.Kp=c->value[0],.Ki=c->value[1],.Kd=c->value[2],
            .IntegralLimit=c->value[3],.MaxOut=c->value[4],.MaxOut_=-c->value[4],.Improve=PID_Integral_Limit};
        PIDInit(pid_for(e,c->arg),&cfg);finish_job(e,AMP_OK,0);return;
    }
    if (e->dji) {
        DJIMotorInstance *m=e->dji;
        if (c->op==AMP_OP_ENABLE) {
            DJIMotorSetRef(m,m->motor_settings.outer_loop_type==ANGLE_LOOP ? m->measure.total_angle : 0);
            DJIMotorSetValue(m,0);DJIMotorEnable(m);
        }
        if (c->op==AMP_OP_VELOCITY || c->op==AMP_OP_POSITION) {
            DJIMotorOuterLoop(m,c->op==AMP_OP_VELOCITY ? SPEED_LOOP : ANGLE_LOOP);
            DJIMotorSetRef(m,c->value[0]*DEG_PER_RAD);DJIMotorEnable(m);
        }
        if (c->op==AMP_OP_CURRENT) { DJIMotorSetValue(m,c->value[0]);DJIMotorEnablePower(m); }
        if (c->op==AMP_OP_TORQUE) {
            DJIMotorSetValue(m, DJIMotorOutputTorqueToCurrentCommand(m->motor_type,
                                                                      c->value[0]));
            DJIMotorEnablePower(m);
        }
        e->phase=2;return;
    }
    DMMotorInstance *m=e->dm;
    if (c->op==AMP_OP_READ_REGISTER || c->op==AMP_OP_WRITE_REGISTER) {
        e->reply_before=m->register_reply_count;
        if (c->op==AMP_OP_READ_REGISTER) DMMotorReadRegister(m,(uint8_t)c->arg);
        else { uint8_t raw[4];memcpy(raw,&c->flags,4);DMMotorWriteRegister(m,(uint8_t)c->arg,raw); }
        e->phase=3;return;
    }
    if (c->op==AMP_OP_ZERO || c->op==AMP_OP_CLEAR_ERROR) {
        DMMotorSetMode(c->op==AMP_OP_ZERO ? DM_CMD_ZERO_POSITION : DM_CMD_CLEAR_ERROR,m);
        e->phase=2;return;
    }
    uint32_t mode=c->op==AMP_OP_VELOCITY ? DM_CTRL_VEL :
        c->op==AMP_OP_POSITION ? DM_CTRL_POS_VEL :
        c->op==AMP_OP_TORQUE || c->op==AMP_OP_MIT ? DM_CTRL_MIT :
        c->op==AMP_OP_MODE ? c->arg : (uint32_t)m->control_mode;
    /* Same-mode streaming setpoints do not disable/re-enable the drive. */
    if (m->control_mode_confirmed && m->control_mode==mode &&
        c->op!=AMP_OP_MODE) e->phase=1;
    else {
        disable(e);DMMotorSetControlMode(m,(DM_Control_Mode_e)mode);
    }
}
static void advance(struct motor_entry *e, uint32_t now)
{
    if (e->slot==NO_JOB) return;
    const struct amp_motor_command *c=&e->command;
    if (now-e->start_ms>AMP_MOTOR_TIMEOUT_MS) { finish_job(e,AMP_ERR_TIMEOUT,0);return; }
    if (MotorOutputsInhibited() && c->op!=AMP_OP_DISABLE) { finish_job(e,AMP_ERR_ESTOP,0);return; }
    if (e->phase==3) {
        if (e->dm->register_reply_count!=e->reply_before) {
            finish_job(e,AMP_OK,e->dm->register_reply_value);return;
        }
        if (now-e->start_ms>DM_REGISTER_REPLY_TIMEOUT_MS+50U) finish_job(e,AMP_ERR_TIMEOUT,0);
        return;
    }
    if (e->dm && e->phase==0) {
        if (!e->dm->control_mode_confirmed || !online(e,now)) return;
        if (e->dm->measure.state>1U) { finish_job(e,AMP_ERR_DRIVE,0);return; }
        if (e->dm->measure.state!=0U) return;
        if (c->op==AMP_OP_MODE) { finish_job(e,AMP_OK,ACK_DRIVE_CONFIRMED);return; }
        e->phase=1;
    }
    if (e->dm && e->phase==1) {
        DMMotorInstance *m=e->dm;
        if (!online(e,now)) return;
        if (m->measure.state>1U) { finish_job(e,AMP_ERR_DRIVE,0);return; }
        if (c->op==AMP_OP_ENABLE && m->control_mode==DM_CTRL_POS_VEL)
            DMMotorSetNativePosition(m,m->measure.position,e->cfg->max_velocity);
        else if (c->op==AMP_OP_MIT) DMMotorSetNativeMIT(m,c->value[0],c->value[1],c->value[2],c->value[3],c->value[4]);
        else if (c->op==AMP_OP_POSITION) DMMotorSetNativePosition(m,c->value[0],c->value[1]);
        else {
            if (c->op==AMP_OP_TORQUE) {
                m->motor_settings.outer_loop_type=OPEN_LOOP;m->motor_settings.close_loop_type=OPEN_LOOP;
            }
            DMMotorSetRef(m,c->op==AMP_OP_ENABLE ? 0 : c->value[0]);
        }
        e->tx_before=tx_count(e);e->feedback_before=feedback_count(e);
        DMMotorEnable(m);e->phase=2;
    }
    if (e->phase!=2 || tx_count(e)==e->tx_before) return;
    if (c->op==AMP_OP_DISABLE) {
        if (!e->dm || !online(e,now)) finish_job(e,AMP_OK,ACK_CONTROLLER_ACCEPTED);
        else if (e->dm->measure.state==0 && feedback_count(e)!=e->feedback_before)
            finish_job(e,AMP_OK,ACK_CONTROLLER_ACCEPTED|ACK_DRIVE_CONFIRMED);
        return;
    }
    if (!online(e,now)) { finish_job(e,AMP_ERR_OFFLINE,0);return; }
    if (e->dm && e->dm->measure.state>1U) {
        if (c->op!=AMP_OP_CLEAR_ERROR) finish_job(e,AMP_ERR_DRIVE,0);
        return;
    }
    if (feedback_count(e)==e->feedback_before) return;
    if (e->dm && c->op!=AMP_OP_CLEAR_ERROR && c->op!=AMP_OP_ZERO && e->dm->measure.state!=1U) return;
    finish_job(e,AMP_OK,ACK_CONTROLLER_ACCEPTED|(e->dm ? ACK_DRIVE_CONFIRMED : 0));
}
static void reject(volatile struct amp_motor_slot *p, uint32_t seq, int32_t error)
{
    p->result=error;uint32_t n=p->count>AMP_MOTOR_BATCH ? AMP_MOTOR_BATCH : p->count;
    for (uint32_t j=0;j<n;j++) p->item_result[j]=error;
    p->accepted_seq=seq;__sync_synchronize();p->completed_seq=seq;
}
void AmpMotorUpdate(void)
{
    if (!shared) return;
    uint32_t now=BSPTimeMillis();
    for (uint32_t i=0;i<count;i++) {
        struct motor_entry *e=&motor_entries[i];
        int enabled=e->dm ? e->dm->stop_flag!=MOTOR_STOP : e->dji ? e->dji->stop_flag!=MOTOR_STOP : 0;
        if (enabled && (!online(e,now) || (e->dm && e->dm->measure.state>1U))) {
            if (e->slot<AMP_MOTOR_QUEUE) finish_job(e,online(e,now) ? AMP_ERR_DRIVE : AMP_ERR_OFFLINE,0);
            else { e->last_result=online(e,now) ? AMP_ERR_DRIVE : AMP_ERR_OFFLINE;disable(e); }
        }
        advance(e,now);
    }
    resolve_slots();
    uint32_t consumer=shared->consumer, producer=shared->producer;
    __sync_synchronize();
    if (consumer==producer || producer-consumer>AMP_MOTOR_QUEUE) return;
    uint32_t s=consumer%AMP_MOTOR_QUEUE;
    volatile struct amp_motor_slot *p=&shared->slot[s];
    uint32_t seq=p->published_seq,n=p->count;
    struct amp_motor_command cmd[AMP_MOTOR_BATCH];
    int index[AMP_MOTOR_BATCH];int32_t result=AMP_OK;
    if (p->session!=shared->session) result=AMP_ERR_SESSION;
    else if (p->generation!=generation) result=AMP_ERR_CANCELED;
    else if (!n || n>AMP_MOTOR_BATCH) result=AMP_ERR_ARGUMENT;
    else {
        memcpy(cmd,(const void *)p->command,n*sizeof(cmd[0]));
        if (n==1 && cmd[0].op==AMP_OP_RESET_ESTOP && cmd[0].motor==0) {
            /* Release never enables or restores an old reference. Cancel older
             * work, then make the reset request itself complete successfully. */
            generation++;shared->generation=generation;
            for (uint32_t i=0;i<count;i++) {
                if (motor_entries[i].slot<AMP_MOTOR_QUEUE) finish_job(&motor_entries[i],AMP_ERR_CANCELED,0);
            }
            disable_all();MotorSetOutputInhibit(0);shared->estop_latched=0;
            p->item_result[0]=AMP_OK;p->reply[0]=0;p->result=AMP_OK;
            p->accepted_seq=seq;__sync_synchronize();p->completed_seq=seq;
            goto consumed;
        }
        for (uint32_t j=0;j<n;j++) {
            index[j]=find(cmd[j].motor);
            if (index[j]<0) { result=AMP_ERR_NOT_FOUND;break; }
            for (uint32_t k=0;k<j;k++) if (index[k]==index[j]) result=AMP_ERR_ARGUMENT;
            if (result!=AMP_OK) break;
            result=validate(&motor_entries[index[j]],&cmd[j],now);
            if (result!=AMP_OK) break;
        }
    }
    if (result!=AMP_OK) reject(p,seq,result);
    else {
        p->result=AMP_PENDING;p->applied_ms=now;
        for (uint32_t j=0;j<n;j++) { p->item_result[j]=AMP_PENDING;p->reply[j]=0; }
        p->accepted_seq=seq;
        for (uint32_t j=0;j<n;j++) start_job(&motor_entries[index[j]],&cmd[j],s,j,now);
        for (uint32_t j=0;j<n;j++) advance(&motor_entries[index[j]],now);
        resolve_slots();
    }
consumed:
    __sync_synchronize(); shared->consumer=consumer+1U;
}
void AmpMotorFinish(void)
{
    if (!shared) return;
    uint32_t now=BSPTimeMillis();tick_count++;
    for (uint32_t i=0;i<count;i++) {
        struct motor_entry *e=&motor_entries[i];
        /* No automatic restart following feedback loss, including native DM
         * modes whose legacy driver would otherwise re-enable on recovery. */
        int enabled=e->dm ? e->dm->stop_flag!=MOTOR_STOP : e->dji ? e->dji->stop_flag!=MOTOR_STOP : 0;
        if (enabled && (!online(e,now) || (e->dm && e->dm->measure.state>1U))) {
            if (e->slot<AMP_MOTOR_QUEUE && e->phase<2) continue; /* disabled mode negotiation */
            if (e->slot<AMP_MOTOR_QUEUE) finish_job(e,online(e,now) ? AMP_ERR_DRIVE : AMP_ERR_OFFLINE,0);
            else { e->last_result=online(e,now) ? AMP_ERR_DRIVE : AMP_ERR_OFFLINE;disable(e); }
        }
        advance(e,now);
    }
    resolve_slots();
    shared->control_tick=tick_count;shared->now_ms=now;
    if (force_status || now-status_last>=1000U/amp_motor_status_hz) {
        shared->status_seq++;__sync_synchronize();
        for (uint32_t i=0;i<count;i++) {
            struct motor_entry *e=&motor_entries[i];
            struct amp_motor_status s={.motor=e->cfg->number,.model=e->cfg->model,.bus=e->cfg->bus,
                .error=(!e->dm && !e->dji) ? 0x80000001U :
                    e->dm && e->dm->measure.state>1U ? e->dm->measure.state : 0,
                .offline=!online(e,now)};
            memcpy((void *)&shared->motor[i],&s,sizeof(s));
        }
        shared->status_ms=now;__sync_synchronize();shared->status_seq++;
        status_last=now;force_status=0;
    }
    if (now-feedback_last>=1000U/amp_motor_feedback_hz || !shared->feedback_seq) {
        shared->feedback_seq++;__sync_synchronize();
        for (uint32_t i=0;i<count;i++) {
            struct motor_entry *e=&motor_entries[i];
            if (motor_feedback_last[i] && now-motor_feedback_last[i]<e->cfg->feedback_period_ms) continue;
            motor_feedback_last[i]=now;
            struct amp_motor_feedback f={.motor=e->cfg->number,.sample_ms=now,
                .feedback_count=feedback_count(e),.age_ms=age(e,now),.control_tx_count=tx_count(e),
                .active_op=e->slot<AMP_MOTOR_QUEUE ? e->command.op : 0,.last_result=e->last_result,
                .max_velocity_rad_s=e->cfg->max_velocity,
                .max_torque_nm=e->cfg->max_torque > 0.0f ? e->cfg->max_torque :
                    (e->dji ? DJIMotorMaxTorqueNm(e->dji) : 0.0f)};
            if (e->dm) {
                DMMotorInstance *m=e->dm;f.drive_state=m->measure.state;
                f.mode=m->reported_control_mode;f.confirmed=m->control_mode_confirmed;f.enabled=m->stop_flag!=MOTOR_STOP;
                f.position_rad=m->measure.position;f.position_total_rad=(float)m->measure.total_position;f.velocity_rad_s=m->measure.velocity;
                f.torque_nm=m->measure.torque;f.temperature_c=m->measure.T_Mos;f.reference=m->pid_ref;
                f.pmax=m->limits.p_max;f.vmax=m->limits.v_max;f.tmax=m->limits.t_max;
            } else if (e->dji) {
                DJIMotorInstance *m=e->dji;f.enabled=m->stop_flag!=MOTOR_STOP;
                f.position_rad=m->measure.angle_single_round/DEG_PER_RAD;f.position_total_rad=m->measure.total_angle/DEG_PER_RAD;f.velocity_rad_s=m->measure.speed_aps/DEG_PER_RAD;
                f.current_raw=m->measure.real_current;
                f.torque_nm=DJIMotorCurrentCommandToOutputTorqueNm(m->motor_type,
                                                                   m->measure.real_current);
                f.temperature_c=m->measure.temperature;
                f.reference=m->stop_flag==MOTOR_POWER ?
                    (e->command.op==AMP_OP_TORQUE ?
                        DJIMotorCurrentCommandToOutputTorqueNm(m->motor_type,m->set_value) :
                        m->set_value) : m->motor_controller.pid_ref/DEG_PER_RAD;
                f.mode=m->stop_flag==MOTOR_POWER ? 0x100U : (uint32_t)m->motor_settings.outer_loop_type;
            }
            memcpy((void *)&shared->feedback[i],&f,sizeof(f));
        }
        shared->feedback_ms=now;__sync_synchronize();shared->feedback_seq++;
        feedback_last=now;
    }
}
