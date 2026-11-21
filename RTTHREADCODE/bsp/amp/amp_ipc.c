/*
 * Minimal AMP IPC driver — ring buffer + mailbox
 */
#include <rtthread.h>
#include <rthw.h>
#include <string.h>
#include "gicv3.h"
#include "board.h"
#include "amp_ipc.h"
#include "amp_diag.h"
#include "amp_fast_protocol.h"
#include "amp_ring_copy.h"

/* Two unidirectional rings in shared memory:
 *   ring_l2r: Linux → RT-Thread (offset 0x00000)
 *   ring_r2l: RT-Thread → Linux (offset 0x10000)
 */
#define RING_L2R  ((struct amp_ring *)(SHMEM_BASE))
#define RING_R2L  ((struct amp_ring *)(SHMEM_BASE + 0x10000))

static struct amp_ring *_rx_ring;  /* ring we receive from */
static struct amp_ring *_tx_ring;  /* ring we send to */
static rt_sem_t _rx_sem = RT_NULL;
static volatile int _ipc_ready = 0;
static volatile uint32_t _mailbox_ticks;
static volatile uint32_t _mailbox_sequence;

static uint32_t _counter_low(void)
{
    uint32_t low, high;
    __asm__ volatile("isb\n mrrc p15, 0, %0, %1, c14"
                     : "=r"(low), "=r"(high) :: "memory");
    return low;
}

#define FAST_WORD(offset) (*(volatile uint32_t *)(AMP_FAST_BASE + (offset)))

/* Echo-only diagnostics; this path never dispatches motor commands. */
static void _fast_process(void)
{
    uint32_t seq = FAST_WORD(AMP_FAST_REQ_SEQ);
    uint32_t len, start, wake, irq_ticks, irq_sequence;
    rt_base_t level;

    if (seq == FAST_WORD(AMP_FAST_RESP_SEQ))
        return;
    __asm__ volatile("dmb sy" ::: "memory");
    len = FAST_WORD(AMP_FAST_REQ_LEN);
    level = rt_hw_interrupt_disable();
    irq_ticks = _mailbox_ticks;
    irq_sequence = _mailbox_sequence;
    start = _counter_low();
    rt_hw_interrupt_enable(level);
    /* Another RT wake may observe published data before its mailbox ISR.
     * Report a sentinel instead of attributing an old IRQ to this request. */
    wake = seq == irq_sequence ? start - irq_ticks : UINT32_MAX;
    if (amp_fast_valid_length(len)) {
        memcpy((void *)(AMP_FAST_BASE + AMP_FAST_RESP_DATA),
               (const void *)(AMP_FAST_BASE + AMP_FAST_REQ_DATA), len);
        FAST_WORD(AMP_FAST_STATUS) = 0;
        FAST_WORD(AMP_FAST_RESP_LEN) = len;
    } else {
        FAST_WORD(AMP_FAST_STATUS) = 1;
        FAST_WORD(AMP_FAST_RESP_LEN) = 0;
    }
    FAST_WORD(AMP_FAST_WAKE_TICKS) = wake;
    FAST_WORD(AMP_FAST_COPY_TICKS) = _counter_low() - start;
    __asm__ volatile("dmb sy" ::: "memory");
    FAST_WORD(AMP_FAST_RESP_SEQ) = seq;
}

static inline uint32_t _ring_avail_read(struct amp_ring *r)
{
    uint32_t write_idx = r->write_idx, read_idx = r->read_idx;
    return write_idx >= read_idx ? write_idx - read_idx :
                                  r->size - read_idx + write_idx;
}

static inline uint32_t _ring_avail_write(struct amp_ring *r)
{
    return r->size - _ring_avail_read(r) - 1;
}

static uint32_t _ring_read(struct amp_ring *r, uint8_t *dst, uint32_t len)
{
    uint32_t read_idx = r->read_idx;
    uint32_t count = _ring_avail_read(r);

    if (len < count)
        count = len;
    __asm__ volatile("dmb sy" ::: "memory");
    read_idx = amp_ring_copy_out(dst, r->buf, r->size, read_idx, count);
    __asm__ volatile("dmb sy" ::: "memory");
    r->read_idx = read_idx;
    return count;
}

static uint32_t _ring_write(struct amp_ring *r, const uint8_t *src, uint32_t len)
{
    uint32_t write_idx = r->write_idx;
    uint32_t count = _ring_avail_write(r);

    if (len < count)
        count = len;
    __asm__ volatile("dmb sy" ::: "memory");
    write_idx = amp_ring_copy_in(r->buf, r->size, write_idx, src, count);
    __asm__ volatile("dmb sy" ::: "memory");
    r->write_idx = write_idx;
    return count;
}

/* ─── Mailbox helpers ─── */
static void _mb_enable_rx(void)
{
    /* B2A outputs reach AP GIC; preserve Linux's other receive channels. */
    MAILBOX_B2A_STATUS = (1 << MB_CH_L2R);
    MAILBOX_B2A_INTEN |= (1 << MB_CH_L2R);
}

static void _mb_send_kick(uint32_t ch, uint32_t cmd, uint32_t data)
{
    MAILBOX_B2A_CMD(ch) = cmd;
    MAILBOX_B2A_DAT(ch) = data;
}

/* ─── ISR ─── */
void amp_ipc_isr(void)
{
    uint32_t status;

    /* Read and clear only CPU3's B2A channel. */
    status = MAILBOX_B2A_STATUS;
    if (status & (1 << MB_CH_L2R)) {
        _mailbox_ticks = _counter_low();
        __asm__ volatile("dmb sy" ::: "memory");
        _mailbox_sequence = FAST_WORD(AMP_FAST_REQ_SEQ);
        MAILBOX_B2A_STATUS = (1 << MB_CH_L2R);
        __asm__ volatile("dsb sy" ::: "memory");
        AMP_DIAG_INC(AMP_DIAG_MAILBOX);
        /* Notify waiting thread */
        if (_rx_sem)
            rt_sem_release(_rx_sem);
    }
}

/* ─── Public API ─── */

int amp_ipc_init(void)
{
    /* Zero rings */
    memset((void *)SHMEM_BASE, 0, 0x20000);
    memset((void *)AMP_FAST_BASE, 0, AMP_FAST_BYTES);
    __asm__ volatile("dmb sy" ::: "memory");
    FAST_WORD(AMP_FAST_READY) = AMP_FAST_MAGIC;

    /* Init ring metadata */
    RING_L2R->size = RING_BUF_SIZE - sizeof(struct amp_ring);
    RING_L2R->write_idx = 0;
    RING_L2R->read_idx = 0;

    RING_R2L->size = RING_BUF_SIZE - sizeof(struct amp_ring);
    RING_R2L->write_idx = 0;
    RING_R2L->read_idx = 0;

    _rx_ring = RING_L2R;
    _tx_ring = RING_R2L;

    /* Create semaphore for RX wait */
    _rx_sem = rt_sem_create("amp_rx", 0, RT_IPC_FLAG_FIFO);

    /* Install mailbox ISR */
    rt_hw_interrupt_install(MAILBOX_A2B_IRQ, (rt_isr_handler_t)amp_ipc_isr,
                            RT_NULL, "mbox");
    /* GICv3 routes SPIs by affinity, not the GICv2 ITARGETSR byte. */
    arm_gic_set_cpu(0, MAILBOX_A2B_IRQ, AMP_CPU_AFFINITY);
    rt_hw_interrupt_umask(MAILBOX_A2B_IRQ);

    /* Enable RX interrupt from Linux */
    _mb_enable_rx();

    _ipc_ready = 1;

    rt_kprintf("[AMP_IPC] Initialized, fast echo v1, cache state in diag SCTLR\n");
    return 0;
}

int amp_ipc_is_ready(void)
{
    return _ipc_ready;
}

int amp_ipc_send(const void *data, uint32_t len)
{
    uint32_t written;
    rt_base_t level;

    if (!_ipc_ready || !_tx_ring)
        return -RT_ERROR;

    level = rt_hw_interrupt_disable();
    AMP_DIAG_SET(AMP_DIAG_IPC_STEP, 1);

    if (len > _ring_avail_write(_tx_ring)) {
        AMP_DIAG_INC(AMP_DIAG_DROP);
        AMP_DIAG_SET(AMP_DIAG_IPC_STEP, 0);
        rt_hw_interrupt_enable(level);
        return -RT_ERROR;
    }

    written = _ring_write(_tx_ring, (const uint8_t *)data, len);
    AMP_DIAG_SET(AMP_DIAG_IPC_STEP, 2);

    /* Kick Linux via mailbox */
    _mb_send_kick(MB_CH_R2L, MB_CMD_SEND, written);
    AMP_DIAG_INC(AMP_DIAG_TX);
    AMP_DIAG_SET(AMP_DIAG_IPC_STEP, 0);

    rt_hw_interrupt_enable(level);

    return (int)written;
}

void amp_ipc_send_log(const char *str)
{
    char pkt[128];
    uint32_t n;

    if (!_ipc_ready || !str)
        return;

    while (*str) {
        n = 0;
        pkt[n++] = 'L';
        pkt[n++] = 'O';
        pkt[n++] = 'G';
        pkt[n++] = ' ';

        while (*str && n < sizeof(pkt) - 1)
            pkt[n++] = *str++;

        amp_ipc_send(pkt, n);
    }
}

int amp_ipc_recv(void *buf, uint32_t max_len)
{
    int ret;

    if (!_ipc_ready || !_rx_ring || !_rx_sem)
        return -RT_ERROR;

    while (1) {
        _fast_process();
        if (_ring_avail_read(_rx_ring) != 0)
            break;
        ret = rt_sem_take(_rx_sem, RT_WAITING_FOREVER);
        if (ret != RT_EOK)
            return ret;
    }

    ret = (int)_ring_read(_rx_ring, (uint8_t *)buf, max_len);
    AMP_DIAG_INC(AMP_DIAG_RX);
    return ret;
}
