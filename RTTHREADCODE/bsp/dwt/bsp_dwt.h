#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>

/* Legacy DWT API, backed by the A55 physical counter, independent of CPU DVFS. */
uint64_t BSPTimeCounter(void);
uint32_t BSPTimeMillis(void);
void DWT_Init(uint32_t cpu_freq_mhz);
float DWT_GetDeltaT(uint32_t *last);
double DWT_GetDeltaT64(uint32_t *last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float seconds);
void DWT_SysTimeUpdate(void);

#endif
