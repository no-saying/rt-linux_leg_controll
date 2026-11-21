#ifndef __BOARD_H__
#define __BOARD_H__

/* ===== GICv3 (from rk3568.dtsi) ===== */
#define GIC_DIST_BASE       0xfd400000
#define GIC_REDIS_BASE      0xfd460000
#define GIC_REDIS_STRIDE    0x20000
#define AMP_CPU_MPIDR       0x3
#define AMP_CPU_LOGICAL_ID  0
/* RK3568 MPIDR affinities are 0x000, 0x100, 0x200, 0x300. */
#define AMP_CPU_AFFINITY    0x300
#define AMP_CPU_REDIS_BASE  (GIC_REDIS_BASE + AMP_CPU_MPIDR * GIC_REDIS_STRIDE)
#define GIC_IRQ_START       0
#define ARM_GIC_MAX_NR      1
#define ARM_GIC_NR_IRQS     512

/* ===== UART4: DW 8250 @ 0xfe680000 (from rk3568.dtsi) ===== */
#define UART4_BASE          0xfe680000
/* reg-shift = <2> → offset multiply by 4, reg-io-width = <4> → 32-bit access */
#define UART4_REG(off)      (*(volatile unsigned int *)(UART4_BASE + ((off) << 2)))
#define UART_THR            UART4_REG(0x00)
#define UART_RBR            UART4_REG(0x00)
#define UART_DLL            UART4_REG(0x00)    /* LCR[7]=1 */
#define UART_IER            UART4_REG(0x01)
#define UART_DLH            UART4_REG(0x01)    /* LCR[7]=1 */
#define UART_FCR            UART4_REG(0x02)
#define UART_LCR            UART4_REG(0x03)
#define UART_MCR            UART4_REG(0x04)
#define UART_LSR            UART4_REG(0x05)
#define UART_LSR_DR         (1 << 0)
#define UART_LSR_THRE       (1 << 5)
#define UART_LSR_TEMT       (1 << 6)

/* ===== RS485 direction: GPIO3_PB5, Rockchip GPIO v2 write-mask registers ===== */
#define GPIO3_BASE          0xfe760000
#define GPIO_REG(off)       (*(volatile unsigned int *)(GPIO3_BASE + (off)))
#define GPIO_SWPORT_DR_L    GPIO_REG(0x00)
#define GPIO_SWPORT_DDR_L   GPIO_REG(0x08)
#define RS485_RTS_PIN       13
#define RS485_RTS_BIT       (1U << RS485_RTS_PIN)
#define RS485_RTS_DR        GPIO_SWPORT_DR_L
#define RS485_RTS_DDR       GPIO_SWPORT_DDR_L

/* ===== ARM Generic Timer (Cortex-A55) ===== */
/* PPI 30 = CNTPNSIRQ (non-secure physical timer) */
#define ARM_TIMER_IRQ       30
/* CNTPCT frequency on RK3568: 24MHz */
#define ARM_TIMER_FREQ      24000000

/* ===== Mailbox @ 0xfe780000 ===== */
/* B2A ch0: rk3568.dtsi GIC_SPI 183, actual GIC INTID 183 + 32. */
#define MAILBOX_A2B_IRQ     215

/* ===== AMP shared memory ===== */
#define AMP_SHMEM_BASE      0x07800000
#define AMP_SHMEM_SIZE      0x00400000

/* Platform GIC interface (required by libcpu interrupt.c) */
static inline unsigned long platform_get_gic_dist_base(void) { return GIC_DIST_BASE; }
static inline unsigned long platform_get_gic_cpu_base(void)  { return 0; }
static inline unsigned long platform_get_gic_redist_base(void) { return AMP_CPU_REDIS_BASE; }

void rt_hw_board_init(void);
void rt_hw_uart_init(void);
int rt_hw_rs485_send(const void *data, rt_size_t len);
int rt_hw_rs485_recv(void *data, rt_size_t max_len);
void rt_hw_cpu_reset(void);

#endif
