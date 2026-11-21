/*
 * RK3568 AMP CPU3 — RT-Thread board initialization
 *
 * Hardware initialized by Linux AMP driver before CPU3 boot:
 *   - GICv3 distributor base at 0xfd400000
 *   - UART4 clocks (SCLK_UART4 + PCLK_UART4) enabled for RS485
 *   - Timer clocks (PCLK_TIMER, CLK_TIMER4/5) enabled
 *   - MCU ACLK enabled
 */
#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include "amp_ipc.h"
#include "amp_periph.h"
#include "amp_diag.h"
#include "amp_memory.h"
#include "amp_dma.h"
#include "amp_uart4.h"

extern unsigned char __bss_start;
extern unsigned char __bss_end;

/* Early assembly runs with MMU off; amp_memory_init installs the identity map. */
unsigned long init_mtbl[4096] __attribute__((aligned(16384)));
void rt_hw_mem_setup_early(void) { }
unsigned int rt_hw_cpu_id_early(void) { return AMP_CPU_LOGICAL_ID; }
int rt_hw_cpu_id(void) { return AMP_CPU_LOGICAL_ID; }
unsigned long long get_main_cpu_affval(void) { return AMP_CPU_AFFINITY; }
void rt_kmem_pvoff_set(unsigned long v) { }
void rt_unwind(void *regs, int adj) { }
extern int rtthread_startup(void);
void master_core_startup(void) { rtthread_startup(); }

/* RT console uses AMP; UART4/RS485 is implemented in amp_uart4.c. */
void rt_hw_console_output(const char *str)
{
    amp_ipc_send_log(str);
}

/* ─── ARM Generic Timer (OS tick) ─── */

static void _timer_isr(int vector, void *param)
{
    /*
     * Reload the countdown timer. The ARM PL1 physical timer interrupt is
     * level-sensitive: if the compare value is not moved forward, the timer
     * line stays asserted and the GIC re-triggers the IRQ immediately after
     * the EOI -> an IRQ storm that prevents the scheduler from ever leaving
     * the first context switch (and hammers the DDR bus, slowing the host).
     */
    __asm__ volatile("mcr p15, 0, %0, c14, c2, 0" ::
                     "r"(ARM_TIMER_FREQ / RT_TICK_PER_SECOND));
    rt_tick_increase();
    AMP_DIAG_INC(AMP_DIAG_TICK);
}

static void _timer_init(void)
{
    unsigned long ctl;

    /* Disable timer first */
    __asm__ volatile("mcr p15, 0, %0, c14, c2, 1" :: "r"(0));

    /* Set timer value = frequency / tick_rate = 24MHz / 1000 = 24000 */
    __asm__ volatile("mcr p15, 0, %0, c14, c2, 0" :: "r"(ARM_TIMER_FREQ / RT_TICK_PER_SECOND));

    /* Install ISR for PPI 30 (CNTPNSIRQ) */
    rt_hw_interrupt_install(ARM_TIMER_IRQ, _timer_isr, RT_NULL, "tick");

    /* Enable the timer interrupt in GIC at PPI priority */
    rt_hw_interrupt_umask(ARM_TIMER_IRQ);

    /* Enable timer: CNTP_CTL.IMASK=0, CNTP_CTL.ENABLE=1 */
    ctl = 1;
    __asm__ volatile("mcr p15, 0, %0, c14, c2, 1" :: "r"(ctl));
}

/* ─── Board init ─── */

void rt_hw_board_init(void)
{
    /* AMP boot trace marker #2: C entry reached (see start_gcc.S) */
    *(volatile unsigned int *)0x07840008 = 0x0BADC0DE;
    amp_memory_init();
    amp_diag_init();

    /* 1. GICv3 — from libcpu/arm/cortex-a/interrupt.c */
    rt_hw_interrupt_init();

    /* CPU3 owns these peripherals; configure their clocks and IOMUX first. */
    amp_hw_init();

    /* 2. UART4 — RS485 data port, not console */
    rt_hw_uart_init();

	/* 3. Heap */
#ifdef RT_USING_HEAP
	rt_system_heap_init((void *)&__bss_end, (void *)0x031F0000);
#endif

    amp_dma_init();
    amp_uart4_runtime_init();

    /* 4. Timer — OS tick */
    _timer_init();

    /* 5. AMP IPC — shared memory + mailbox */
    amp_ipc_init();

}

void rt_hw_cpu_reset(void) { while (1); }
