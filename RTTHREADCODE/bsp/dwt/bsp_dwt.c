#include "bsp_dwt.h"
#include <math.h>
#include <rtthread.h>
#include "board.h"

uint64_t BSPTimeCounter(void)
{
    uint32_t low, high;
    __asm__ volatile("isb\n\tmrrc p15, 0, %0, %1, c14"
                     : "=r"(low), "=r"(high) :: "memory");
    return ((uint64_t)high << 32) | low;
}

uint32_t BSPTimeMillis(void)
{
    return (uint32_t)(BSPTimeCounter() / (ARM_TIMER_FREQ / 1000U));
}

void DWT_Init(uint32_t cpu_freq_mhz)
{
    (void)cpu_freq_mhz;
}

double DWT_GetDeltaT64(uint32_t *last)
{
    uint32_t now = (uint32_t)BSPTimeCounter();
    uint32_t delta = now - *last;
    *last = now;
    return (double)delta / ARM_TIMER_FREQ;
}

float DWT_GetDeltaT(uint32_t *last)
{
    return (float)DWT_GetDeltaT64(last);
}

uint64_t DWT_GetTimeline_us(void)
{
    return BSPTimeCounter() / (ARM_TIMER_FREQ / 1000000U);
}

float DWT_GetTimeline_s(void)
{
    return (float)((double)BSPTimeCounter() / ARM_TIMER_FREQ);
}

float DWT_GetTimeline_ms(void)
{
    return (float)((double)BSPTimeCounter() / (ARM_TIMER_FREQ / 1000U));
}

void DWT_Delay(float seconds)
{
    uint64_t start, duration;
    if (!isfinite(seconds) || seconds <= 0.0f || seconds > 60.0f)
        return;
    start = BSPTimeCounter();
    duration = (uint64_t)((double)seconds * ARM_TIMER_FREQ);
    while (BSPTimeCounter() - start < duration)
        ;
}

void DWT_SysTimeUpdate(void)
{
    /* CNTPCT is already 64-bit; no software overflow extension is needed. */
}
