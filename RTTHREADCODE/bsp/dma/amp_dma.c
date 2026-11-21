/* SPDX-License-Identifier: BSD-3-Clause
 * CPU3 owns the entire DMAC0, only after the Linux DT/clock/resource grant.
 * Channel buffers are private, cache-line isolated, static and never freed.
 */
#include <rtthread.h>
#include <rthw.h>
#include <stdint.h>
#include <string.h>
#include "amp_dma.h"
#include "board.h"
#include "bsp_dwt.h"
#include "gicv3.h"

#ifndef DMA_REG
#define DMA_REG(off) (*(volatile uint32_t *)(0xfe530000U + (off)))
#endif
#ifndef HANDSHAKE
#define HANDSHAKE(off) (*(volatile uint32_t *)(AMP_DMA_CONTROL + (off)))
#endif
#define DMA_EVENT_IRQ 46
#define DMA_ABORT_IRQ 45
#define DMA_STATE(ch) (DMA_REG(0x100 + (ch) * 8) & 15U)
#ifndef BARRIER
#define BARRIER() __asm__ volatile("dsb sy" ::: "memory")
#endif

extern void rt_hw_cpu_dcache_clean(void *, int);
extern void rt_hw_cpu_dcache_clean_and_invalidate(void *, int);
extern void rt_hw_cpu_dcache_invalidate(void *, int);

static struct rt_mutex engine_lock;
static struct rt_semaphore completion[AMP_DMA_CHANNELS];
static struct amp_dma_code code[AMP_DMA_CHANNELS] __attribute__((aligned(64)));
/* Pad each program to a separate 64-byte boundary, including its metadata. */
static uint8_t program[AMP_DMA_CHANNELS][192] __attribute__((aligned(64)));
static const uint32_t fifo[AMP_DMA_CHANNELS] = {0xfe640400, 0xfe640800, 0xfe680000, 0xfe680000};
static const uint8_t request[AMP_DMA_CHANNELS] = {26, 27, 8, 9};
static volatile uint8_t done[AMP_DMA_CHANNELS], error[AMP_DMA_CHANNELS];
static uint32_t address[AMP_DMA_CHANNELS], length[AMP_DMA_CHANNELS], epoch;
static uint8_t initialized;
static volatile uint8_t fatal; /* Written from the abort ISR. */
static struct amp_dma_stats counters;

void amp_dma_invalidate(void *buffer, uint32_t size)
{
    rt_hw_cpu_dcache_invalidate(buffer, (size + 63U) & ~63U);
}

void amp_dma_init(void)
{
    unsigned i;
    /* Clear stale Linux acknowledgement before publishing the new RT request. */
    HANDSHAKE(0) = 0; HANDSHAKE(8) = 0; HANDSHAKE(12) = 0; HANDSHAKE(16) = 0;
    epoch = (uint32_t)BSPTimeCounter() | 1U;
    HANDSHAKE(4) = epoch;
    rt_mutex_init(&engine_lock, "dmac0", RT_IPC_FLAG_PRIO);
    for (i = 0; i < AMP_DMA_CHANNELS; ++i)
        rt_sem_init(&completion[i], "dmaend", 0, RT_IPC_FLAG_FIFO);
    BARRIER(); HANDSHAKE(0) = AMP_DMA_RT_MAGIC;
}

static void dma_isr(int vector, void *arg)
{
    uint32_t events = DMA_REG(0x024), faults = DMA_REG(0x034);
    unsigned i;
    (void)vector; (void)arg;
    if (faults || DMA_REG(0x030)) {
        /* Fault IRQ is level-triggered and cannot be cleared by INTCLR.
         * Mask it before waking threads, otherwise they cannot run DMAKILL. */
        rt_hw_interrupt_mask(DMA_ABORT_IRQ);
        fatal = 1; faults = 15U;
    }
    DMA_REG(0x02c) = events & 15U;
    for (i = 0; i < AMP_DMA_CHANNELS; ++i) {
        if (faults & (1U << i)) { error[i] = 1; ++counters.faults; }
        if ((events | faults) & (1U << i)) {
            if (events & (1U << i)) { done[i] = 1; ++counters.completed[i]; }
            rt_sem_release(&completion[i]);
        }
    }
}

int amp_dma_available(void)
{
    unsigned i;
    if (fatal) return -RT_ERROR;
    if (HANDSHAKE(12) != AMP_DMA_LINUX_MAGIC || HANDSHAKE(8) != epoch)
        return 0;
    BARRIER();
    rt_mutex_take(&engine_lock, RT_WAITING_FOREVER);
    if (!initialized) {
        /* The Linux handoff cleared all channels before granting ownership. */
        for (i = 0; i < 8; ++i) {
            if (DMA_STATE(i)) { fatal = 1; break; }
        }
        if (!fatal && !(DMA_REG(0xe00) & (1U << 2))) fatal = 1;
        if (!fatal) {
            DMA_REG(0x020) = 0;
            DMA_REG(0x02c) = 0xffffffffU;
            rt_hw_interrupt_install(DMA_EVENT_IRQ, dma_isr, RT_NULL, "dma0");
            rt_hw_interrupt_install(DMA_ABORT_IRQ, dma_isr, RT_NULL, "dmaerr");
            arm_gic_set_cpu(0, DMA_EVENT_IRQ, AMP_CPU_AFFINITY);
            arm_gic_set_cpu(0, DMA_ABORT_IRQ, AMP_CPU_AFFINITY);
            arm_gic_set_priority(0, DMA_EVENT_IRQ, 0xc0);
            arm_gic_set_priority(0, DMA_ABORT_IRQ, 0xc0);
            rt_hw_interrupt_umask(DMA_EVENT_IRQ);
            rt_hw_interrupt_umask(DMA_ABORT_IRQ);
            initialized = 1;
            counters.granted = 1;
            HANDSHAKE(16) = 1;
        }
    }
    rt_mutex_release(&engine_lock);
    return fatal ? -RT_ERROR : initialized;
}

static int debug_command(unsigned ch, int start)
{
    uint64_t deadline = BSPTimeCounter() + 2400U; /* 100 us, IRQs remain enabled */
    uint32_t instruction;
    int rc = RT_EOK;
    rt_mutex_take(&engine_lock, RT_WAITING_FOREVER);
    while (DMA_REG(0xd00) & 1U) {
        if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; goto out; }
    }
    instruction = start ? ((uint32_t)0xa2 << 16) | (ch << 24) :
                          (1U << 16) | (ch << 8) | 1U;
    DMA_REG(0xd08) = instruction;
    DMA_REG(0xd0c) = start ? (uint32_t)(uintptr_t)program[ch] : 0;
    BARRIER(); DMA_REG(0xd04) = 0;
    /* Wait for the injected instruction, so a rapid stop cannot overtake GO. */
    BARRIER();
    while (DMA_REG(0xd00) & 1U) {
        if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; break; }
    }
out:
    rt_mutex_release(&engine_lock);
    return rc;
}

int amp_dma_prepare(unsigned ch, void *buffer, uint32_t size)
{
    rt_base_t level;
    uintptr_t addr = (uintptr_t)buffer;
    if (ch >= AMP_DMA_CHANNELS || !size || size > AMP_DMA_MAX_BYTES ||
        (addr & 63U) || addr < 0x03000000U || addr + ((size + 63U) & ~63U) > 0x03200000U)
        return -RT_EINVAL;
    if (amp_dma_available() <= 0) return -RT_ENOSYS;
    if (DMA_STATE(ch)) return -RT_EBUSY;
    if (amp_pl330_program(&code[ch], request[ch], ch, addr, fifo[ch], size, ch & 1U) < 0)
        return -RT_EINVAL;
    address[ch] = addr; length[ch] = size;
    if (ch & 1U) rt_hw_cpu_dcache_clean_and_invalidate(buffer, (size + 63U) & ~63U);
    else rt_hw_cpu_dcache_clean(buffer, (size + 63U) & ~63U);
    memcpy(program[ch], code[ch].bytes, code[ch].used);
    rt_hw_cpu_dcache_clean(program[ch], sizeof(program[ch]));
    level = rt_hw_interrupt_disable();
    DMA_REG(0x02c) = 1U << ch;
    done[ch] = error[ch] = 0;
    rt_sem_control(&completion[ch], RT_IPC_CMD_RESET, RT_NULL);
    DMA_REG(0x020) |= 1U << ch;
    rt_hw_interrupt_enable(level);
    BARRIER();
    return RT_EOK;
}

int amp_dma_start(unsigned ch)
{
    int rc;
    if (ch >= AMP_DMA_CHANNELS || amp_dma_available() <= 0) return -RT_EINVAL;
    rc = debug_command(ch, 1);
    if (!rc) ++counters.started[ch];
    return rc;
}

int amp_dma_stop(unsigned ch)
{
    uint64_t deadline;
    rt_base_t level;
    if (ch >= AMP_DMA_CHANNELS || !initialized) return -RT_EINVAL;
    if (DMA_STATE(ch) && debug_command(ch, 0)) goto failed;
    deadline = BSPTimeCounter() + 24000U;
    while (DMA_STATE(ch)) {
        if (BSPTimeCounter() >= deadline) goto failed;
    }
    level = rt_hw_interrupt_disable();
    DMA_REG(0x020) &= ~(1U << ch);
    DMA_REG(0x02c) = 1U << ch;
    rt_hw_interrupt_enable(level);
    BARRIER();
    return RT_EOK;
failed:
    /* Quarantine every static buffer: no retries or PIO on an active DMA. */
    fatal = 1; ++counters.stop_failures;
    return -RT_ERROR;
}

int amp_dma_wait(unsigned ch, uint64_t deadline)
{
    int rc = RT_EOK;
    if (ch >= AMP_DMA_CHANNELS || !initialized) return -RT_EINVAL;
    while (!done[ch] && !error[ch] && !fatal) {
        uint64_t now = BSPTimeCounter();
        rt_tick_t ticks;
        if (now >= deadline) { rc = -RT_ETIMEOUT; ++counters.timeouts; break; }
        ticks = (rt_tick_t)((deadline - now + 23999U) / 24000U);
        rt_sem_take(&completion[ch], ticks);
    }
    if (error[ch] || fatal) rc = -RT_ERROR;
    /* SEV precedes END: wait for STOP before reuse, or kill on failure. */
    if (!rc) {
        uint64_t end = BSPTimeCounter() + 2400U;
        while (DMA_STATE(ch)) {
            if (BSPTimeCounter() >= end) { rc = -RT_ETIMEOUT; break; }
        }
    }
    if (amp_dma_stop(ch)) return -RT_ERROR;
    if (ch & 1U) amp_dma_invalidate((void *)(uintptr_t)address[ch], length[ch]);
    return rc;
}

uint32_t amp_dma_received(unsigned ch)
{
    uint32_t pos;
    if (ch >= AMP_DMA_CHANNELS || !initialized || DMA_STATE(ch)) return 0;
    pos = DMA_REG(0x404 + ch * 0x20);
    return pos >= address[ch] && pos - address[ch] <= length[ch] ? pos - address[ch] : 0;
}

void amp_dma_get_stats(struct amp_dma_stats *stats)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (stats) {
        *stats = counters;
        stats->granted = initialized && !fatal &&
                         HANDSHAKE(12) == AMP_DMA_LINUX_MAGIC && HANDSHAKE(8) == epoch;
    }
    rt_hw_interrupt_enable(level);
}

int amp_dma_faulted(void) { return fatal; }
