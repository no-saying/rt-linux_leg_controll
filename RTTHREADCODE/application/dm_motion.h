#ifndef DM_MOTION_H
#define DM_MOTION_H
#include "dmmotor.h"
#include "dm_motion_protocol.h"
/* Init before motor task startup. Poll exclusively from the motor task. */
void DMMotionInit(DMMotorInstance *motor, volatile struct dm_motion_page *page);
void DMMotionPoll(void);
#endif
