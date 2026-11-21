#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>

int MotorServiceInit(void);
/* Latched software emergency gate; only protocol reset releases it. */
int MotorOutputsInhibited(void);
void MotorSetOutputInhibit(int inhibit);
void MotorControlTask(void);
uint32_t MotorControlTaskTicks(void);

/* Cumulative CNTPCT ticks at 24 MHz; zero period_min means no interval yet. */
struct MotorTiming {
    uint32_t samples;
    uint32_t period_min_ticks, period_max_ticks, execution_max_ticks;
    uint32_t period_over_1500us, execution_over_1000us;
};
void MotorControlTiming(struct MotorTiming *timing);

/* Override to register motors after CAN hardware init, before the service starts. */
void MotorApplicationInit(void);
/* Runs before each control iteration; use it for serialized motor management. */
void MotorApplicationUpdate(void);

#endif
