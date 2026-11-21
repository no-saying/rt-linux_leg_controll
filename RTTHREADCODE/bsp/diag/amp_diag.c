#include <rtthread.h>
#include "amp_diag.h"

void amp_diag_init(void)
{
    unsigned int offset;
    uint32_t sctlr;

    for (offset = 0x0c; offset < 0x80; offset += 4)
        AMP_DIAG_SET(offset, 0);
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
    AMP_DIAG_SET(0x3c, sctlr);
    AMP_DIAG_SET(0x0c, 0x44494101);
}

void amp_diag_fault(uint32_t kind, uint32_t pc, uint32_t cpsr)
{
    uint32_t dfar, dfsr;
    __asm__ volatile("mrc p15, 0, %0, c6, c0, 0" : "=r"(dfar));
    __asm__ volatile("mrc p15, 0, %0, c5, c0, 0" : "=r"(dfsr));
    AMP_DIAG_SET(0x4c, pc);
    AMP_DIAG_SET(0x50, cpsr);
    AMP_DIAG_SET(0x54, dfar);
    AMP_DIAG_SET(0x58, dfsr);
    AMP_DIAG_SET(0x5c, rt_thread_self());
    AMP_DIAG_SET(0x48, kind);
    __asm__ volatile("dsb sy" ::: "memory");
}
