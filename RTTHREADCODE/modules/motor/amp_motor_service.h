#ifndef AMP_MOTOR_SERVICE_H
#define AMP_MOTOR_SERVICE_H
#include "amp_motor_protocol.h"
/* Register motor instances first; init never enables motors. */
void AmpMotorInit(volatile struct amp_motor_shared *shared);
void AmpMotorBegin(void);
void AmpMotorUpdate(void);
void AmpMotorFinish(void);
#endif
