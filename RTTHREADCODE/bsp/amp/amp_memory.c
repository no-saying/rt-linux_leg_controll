/* Identity mapping for the CPU3 firmware. Shared RAM must match Linux WC
 * (Normal non-cacheable), never WB: Linux's diagnostic readers use WC too. */
#include <stdint.h>
#include "amp_memory.h"
#include "amp_memory_layout.h"

#ifndef AMP_PRIVATE_CACHE
#define AMP_PRIVATE_CACHE 1
#endif

extern unsigned long init_mtbl[4096];

void amp_memory_init(void)
{
    uint32_t i, value, zero = 0, table = (uint32_t)init_mtbl;

    /* Called before heap, IRQs or secondary RT threads. Startup has MMU and
     * D-cache off. Only this core's 2 MiB reservation becomes cacheable. */
    for (i = 0; i < 4096; ++i)
        init_mtbl[i] = amp_section_descriptor(i, AMP_PRIVATE_CACHE);

    __asm__ volatile("dsb sy\n"
                     "mcr p15, 0, %0, c2, c0, 2\n" /* TTBCR: short descriptors */
                     "mcr p15, 0, %1, c2, c0, 0\n" /* TTBR0, uncached walks */
                     "mcr p15, 0, %2, c3, c0, 0\n" /* domain 0 client */
                     "mcr p15, 0, %0, c8, c7, 0\n"
                     "dsb sy\n isb"
                     :: "r"(zero), "r"(table), "r"(1U) : "memory");
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(value));
    value &= ~((1U << 29) | (1U << 28)); /* AFE=0, TRE=0 */
    value &= ~(1U << 2);
    value |= (1U << 23) | (1U << 12) | 1U; /* XP,I,M */
    if (AMP_PRIVATE_CACHE)
        value |= (1U << 2);
    __asm__ volatile("dsb sy\n mcr p15, 0, %0, c1, c0, 0\n isb"
                     :: "r"(value) : "memory");
}
