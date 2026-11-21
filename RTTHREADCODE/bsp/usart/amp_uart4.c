#include <rtthread.h>
#include <rthw.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "amp_uart4.h"
#include "amp_dma.h"
#include "bsp_dwt.h"
#include "gicv3.h"

#define UART4_IRQ 152
#define UART_IIR UART4_REG(2)
#define UART_USR UART4_REG(0x1f)
#define UART_TFL UART4_REG(0x20)
#define RX_RING_SIZE 1024U
#ifndef UART_BARRIER
#define UART_BARRIER() __asm__ volatile("dmb sy" ::: "memory")
#endif
static uint8_t ring[RX_RING_SIZE];
static volatile uint32_t produced, consumed;
static volatile int rx_dma_active;
static int irq_mode;
static struct rt_mutex bus_lock, rx_lock;
static uint8_t tx_buffer[AMP_DMA_MAX_BYTES] __attribute__((aligned(64)));
static uint8_t rx_buffer[AMP_DMA_MAX_BYTES] __attribute__((aligned(64)));
static struct amp_uart4_stats counters;

void rt_hw_uart_init(void)
{
    UART_IER = 0;
    UART_LCR = 0x80;
    UART_DLL = 13; UART_DLH = 0; /* 24 MHz / (16 * 13): approximately 115200 */
    UART_LCR = 0x03;
    UART_FCR = 0x0f; /* FIFO reset, DMA mode 1, RX trigger one byte */
    UART_MCR = 0x01;
    RS485_RTS_DDR = RS485_RTS_BIT | (RS485_RTS_BIT << 16);
    RS485_RTS_DR = RS485_RTS_BIT << 16;
}

static void uart_isr(int vector, void *parameter)
{
    unsigned budget = 128;
    (void)vector; (void)parameter;
    while (budget--) {
        uint32_t lsr = UART_LSR;
        uint32_t next;
        uint8_t byte;
        if (lsr & 0x1eU) ++counters.line_errors;
        if (!(lsr & UART_LSR_DR) || rx_dma_active) break;
        byte = UART_RBR;
        next = (produced + 1U) & (RX_RING_SIZE - 1U);
        ++counters.rx_bytes;
        if (next == consumed) { ++counters.rx_dropped; continue; }
        ring[produced] = byte;
        UART_BARRIER();
        produced = next;
    }
    (void)UART_IIR;
}

void amp_uart4_runtime_init(void)
{
    rt_mutex_init(&bus_lock, "rs485", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&rx_lock, "485rx", RT_IPC_FLAG_PRIO);
}

static void uart_enable_irq(void)
{
    if (irq_mode || amp_dma_available() <= 0) return;
    /* The grant also proves Linux installed the UART4 CPU3 GIC reservation.
     * With old boot.img keep polling: its GIC init may overwrite our route. */
    rt_hw_interrupt_install(UART4_IRQ, uart_isr, RT_NULL, "uart4");
    arm_gic_set_cpu(0, UART4_IRQ, AMP_CPU_AFFINITY);
    rt_hw_interrupt_umask(UART4_IRQ);
    irq_mode = 1;
    UART_IER = 0x05; /* RX data and line status; TX never spins in an ISR. */
}

static uint32_t ring_read(uint8_t *dst, uint32_t count)
{
    uint32_t n = 0;
    while (n < count && consumed != produced) {
        UART_BARRIER();
        dst[n++] = ring[consumed];
        UART_BARRIER();
        consumed = (consumed + 1U) & (RX_RING_SIZE - 1U);
    }
    return n;
}

int rt_hw_rs485_recv(void *data, rt_size_t max_len)
{
    int count;
    if (!data) return -RT_EINVAL;
    rt_mutex_take(&rx_lock, RT_WAITING_FOREVER);
    if (amp_dma_available() < 0) { rt_mutex_release(&rx_lock); return -RT_ERROR; }
    uart_enable_irq();
    if (!irq_mode) uart_isr(0, RT_NULL);
    count = ring_read(data, max_len);
    rt_mutex_release(&rx_lock);
    return count;
}

int rt_hw_rs485_send(const void *data, rt_size_t size)
{
    const uint8_t *bytes = data;
    uint64_t deadline;
    uint32_t i;
    int ready, rc = RT_EOK;
    if (!data || size > AMP_DMA_MAX_BYTES) return -RT_EINVAL;
    if (!size) return 0;
    rt_mutex_take(&bus_lock, RT_WAITING_FOREVER);
    ready = amp_dma_available();
    if (ready < 0) { rc = -RT_ERROR; goto out; }
    deadline = BSPTimeCounter() + (uint64_t)(size * 100U + 10000U) * 24U;
    /* A previous aborted frame may still have one character in the shifter.
     * Keep DE low until it drains, so no old tail enters the next frame. */
    while (!(UART_LSR & UART_LSR_TEMT)) {
        if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; goto direction; }
        rt_thread_mdelay(1);
    }
    if (size >= 16 && ready) {
        memcpy(tx_buffer, data, size);
        rc = amp_dma_prepare(AMP_DMA_UART_TX, tx_buffer, size);
        if (rc) goto out;
        RS485_RTS_DR = RS485_RTS_BIT | (RS485_RTS_BIT << 16);
        rc = amp_dma_start(AMP_DMA_UART_TX);
        if (!rc) rc = amp_dma_wait(AMP_DMA_UART_TX, deadline);
        else amp_dma_stop(AMP_DMA_UART_TX);
        ++counters.tx_dma;
        if (rc) goto direction;
    } else {
        ++counters.tx_pio;
        RS485_RTS_DR = RS485_RTS_BIT | (RS485_RTS_BIT << 16);
        for (i = 0; i < size; ++i) {
            while (!(UART_USR & 2U)) {
                if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; goto direction; }
                rt_thread_mdelay(1);
            }
            UART_THR = bytes[i];
        }
    }
    /* DMA completion means FIFO filled, not the last stop bit transmitted. */
    while (!(UART_LSR & UART_LSR_TEMT)) {
        if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; break; }
        if (UART_TFL > 2U) rt_thread_mdelay(1);
    }
direction:
    if (rc) UART_FCR = 0x0d; /* Drop an aborted TX FIFO, preserve received data. */
    RS485_RTS_DR = RS485_RTS_BIT << 16;
    if (rc == -RT_ETIMEOUT) ++counters.timeouts;
out:
    rt_mutex_release(&bus_lock);
    return rc ? rc : (int)size;
}

int rt_hw_rs485_recv_dma(void *data, uint32_t size, uint32_t timeout_ms)
{
    uint32_t count, partial, ier;
    uint64_t deadline;
    rt_base_t level;
    int rc;
    if (!data || !size || size > AMP_DMA_MAX_BYTES || !timeout_ms || timeout_ms > 1000)
        return -RT_EINVAL;
    rt_mutex_take(&bus_lock, RT_WAITING_FOREVER);
    rt_mutex_take(&rx_lock, RT_WAITING_FOREVER);
    if (amp_dma_available() <= 0) { rc = -RT_ENOSYS; goto out; }
    level = rt_hw_interrupt_disable();
    ier = UART_IER; UART_IER = 0; rx_dma_active = 1;
    rt_hw_interrupt_enable(level);
    count = ring_read(data, size);
    if (count == size) { rc = count; goto restore; }
    deadline = BSPTimeCounter() + (uint64_t)timeout_ms * 24000U;
    rc = amp_dma_prepare(AMP_DMA_UART_RX, rx_buffer, size - count);
    if (rc) goto restore;
    rc = amp_dma_start(AMP_DMA_UART_RX);
    if (!rc) rc = amp_dma_wait(AMP_DMA_UART_RX, deadline);
    else amp_dma_stop(AMP_DMA_UART_RX);
    ++counters.rx_dma;
    if (!rc || rc == -RT_ETIMEOUT) {
        /* wait() has stopped the channel and invalidated the isolated buffer. */
        partial = amp_dma_received(AMP_DMA_UART_RX);
        memcpy((uint8_t *)data + count, rx_buffer, partial);
        count += partial;
        counters.rx_bytes += partial;
        if (rc) ++counters.timeouts;
        rc = count;
    }
restore:
    level = rt_hw_interrupt_disable();
    /* A failed DMA stop quarantines the port instead of racing a live reader. */
    if (!amp_dma_faulted()) { rx_dma_active = 0; UART_IER = ier; }
    rt_hw_interrupt_enable(level);
out:
    rt_mutex_release(&rx_lock);
    rt_mutex_release(&bus_lock);
    return rc;
}

void amp_uart4_get_stats(struct amp_uart4_stats *stats)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (stats) *stats = counters;
    rt_hw_interrupt_enable(level);
}
