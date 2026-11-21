#include <rtthread.h>
#include <rthw.h>
#include <stdint.h>
#include <string.h>
#include "amp_periph.h"
#include "bsp_dwt.h"

#ifndef REG32
#define REG32(addr) (*(volatile uint32_t *)(addr))
#endif
#ifndef CAN_REG_READ
#define CAN_REG_READ(base, off) REG32((base) + (off))
#define CAN_REG_WRITE(base, off, val) (REG32((base) + (off)) = (val))
#endif

void amp_hw_init(void)
{
    /* Controller setup is performed after Linux has enabled the AMP clocks. */
    amp_spi3_hw_init();
}

#define PWM14_BASE              0xfe700020U
#define PWM_PERIOD              REG32(PWM14_BASE + 0x04)
#define PWM_DUTY                REG32(PWM14_BASE + 0x08)
#define PWM_CTRL                REG32(PWM14_BASE + 0x0c)
#define PWM_ENABLE              (1U << 0)
#define PWM_CONTINUOUS          (1U << 1)
#define PWM_DUTY_POSITIVE       (1U << 3)

#define CAN1_BASE               0xfe580000U
#define CAN2_BASE               0xfe590000U
#define CAN_MODE                0x000
#define CAN_CMD                 0x004
#define CAN_STATE               0x008
#define CAN_INT                 0x00c
#define CAN_INT_MASK            0x010
#define CAN_ERR_CODE            0x02c
#define CAN_RX_ERR_CNT          0x034
#define CAN_TX_ERR_CNT          0x038
#define CAN_IDCODE              0x03c
#define CAN_IDMASK              0x040
#define CAN_NBTP                0x100
#define CAN_RXFC                0x118
#define CAN_AFC                 0x11c
#define CAN_IDCODE0             0x120
#define CAN_IDMASK0             0x124
#define CAN_IDCODE1             0x128
#define CAN_IDMASK1             0x12c
#define CAN_IDCODE2             0x130
#define CAN_IDMASK2             0x134
#define CAN_IDCODE3             0x138
#define CAN_IDMASK3             0x13c
#define CAN_IDCODE4             0x140
#define CAN_IDMASK4             0x144
#define CAN_TXFIC               0x200
#define CAN_TXID                0x204
#define CAN_TXDAT0              0x208
#define CAN_RXFRD               0x400

#define CAN_WORK_MODE           (1U << 0)
#define CAN_MODE_FDOE           (1U << 15)
#define CAN_MODE_SPACE_RX       (1U << 12)
#define CAN_TX0_REQ             (1U << 0)
#define CAN_TX1_REQ             (1U << 1)
#define CAN_FIFO_ENABLE         (1U << 0)
#define CAN_RX_FIFO_CNT_SHIFT   4
#define CAN_RX_FIFO_CNT_MASK    (0x7U << CAN_RX_FIFO_CNT_SHIFT)
#define CAN_RX_FILTER_MASK      0x1fffffffU
#define CAN_FORMAT_EXT          (1U << 7)
#define CAN_FORMAT_FD           (1U << 5)
#define CAN_RTR                 (1U << 6)
#define CAN_STATE_BUS_OFF       (1U << 5)
#define CAN_TX_FINISH_INT       (1U << 1)
#define CAN_ERROR_INTS          ((1U << 2) | (1U << 4) | (1U << 5) | \
                                 (1U << 6) | (1U << 9))
#define CAN_BUS_OFF_RECOVERY_INT (1U << 10)
#define CAN_STATUS_INTS         (CAN_TX_FINISH_INT | CAN_ERROR_INTS | \
                                 CAN_BUS_OFF_RECOVERY_INT)
#define CAN_RECOVERY_MS         100U

#define CRU_CAN_SOFTRST         REG32(0xfdd20454U)
#define CRU_CAN1_RESET_MASK     ((1U << 6) | (1U << 7))
#define CRU_CAN2_RESET_MASK     ((1U << 8) | (1U << 9))

#define NBTP_NSJW_SHIFT         24
#define NBTP_NBRP_SHIFT         16
#define NBTP_NTSEG2_SHIFT       8
#define NBTP_NTSEG1_SHIFT       0

/* DTS supplies 100 MHz. Fields encode segment lengths minus one:
 * 100 MHz / (2 * (4 + 1) * (1 + (6 + 1) + (1 + 1))) = 1 Mbps, sample at 80%. */
#define CAN_1M_NBRP_REG         4U
#define CAN_1M_NSJW_REG         1U
#define CAN_1M_NTSEG1_REG       6U
#define CAN_1M_NTSEG2_REG       1U

static uint32_t can_last_recovery_ms[2];
static struct amp_can_diagnostic can_diagnostics[2];
static uint8_t can_bus_off_seen[2];
#ifdef AMP_CAN_LOOPBACK_TEST
static uint8_t can_loopback_active[2];
#endif

static inline uint32_t can_read(uint32_t base, uint32_t off)
{
    return CAN_REG_READ(base, off);
}

static inline void can_write(uint32_t base, uint32_t off, uint32_t val)
{
    CAN_REG_WRITE(base, off, val);
}

static uint32_t can_base(int bus)
{
    if (bus == 1)
        return CAN1_BASE;
    if (bus == 2)
        return CAN2_BASE;
    return 0;
}

int amp_pwm14_set(uint32_t duty_permille)
{
    uint32_t period = 100000; /* 100 MHz assigned PWM clock, 1 kHz output. */

    if (duty_permille > 1000)
        duty_permille = 1000;

    PWM_PERIOD = period;
    PWM_DUTY = (period * duty_permille) / 1000;
    /*
     * Always keep the channel enabled. With DUTY_POSITIVE the output is HIGH
     * while the counter < DUTY, so duty=0 forces the pin LOW (MOSFET heater
     * off). Disabling the channel (PWM_CTRL=0) can leave the pin floating or
     * HIGH, which turns the heater fully on.
     */
    PWM_CTRL = PWM_ENABLE | PWM_CONTINUOUS | PWM_DUTY_POSITIVE;

    return RT_EOK;
}

/* Called by the CAN owner with interrupts disabled, never by the reporter. */
static void can_sample_status(uint32_t base, unsigned int index)
{
    struct amp_can_diagnostic *diag = &can_diagnostics[index];
    uint32_t bus_off;

    diag->sample_ms = BSPTimeMillis();
    diag->mode = can_read(base, CAN_MODE);
    diag->cmd = can_read(base, CAN_CMD);
    diag->state = can_read(base, CAN_STATE);
    diag->irq = can_read(base, CAN_INT);
    diag->error = can_read(base, CAN_ERR_CODE);
    diag->rx_error = can_read(base, CAN_RX_ERR_CNT);
    diag->tx_error = can_read(base, CAN_TX_ERR_CNT);
    diag->nbtp = can_read(base, CAN_NBTP);
    diag->tx_id = can_read(base, CAN_TXID);
    diag->tx_fic = can_read(base, CAN_TXFIC);
    bus_off = !!(diag->state & CAN_STATE_BUS_OFF);

    if (diag->irq & CAN_TX_FINISH_INT)
        diag->tx_completed++;
    if (diag->irq & CAN_ERROR_INTS)
        diag->error_events++;
    if (bus_off && !can_bus_off_seen[index])
        diag->bus_off_events++;
    if ((diag->irq & CAN_ERROR_INTS) ||
        (bus_off && !can_bus_off_seen[index])) {
        diag->last_error = diag->error;
        diag->last_state = diag->state;
        diag->last_rx_error = diag->rx_error;
        diag->last_tx_error = diag->tx_error;
    }
    can_bus_off_seen[index] = bus_off;

    /* Save error evidence before W1C; leave RX flags/FIFO to the RX owner. */
    if (diag->irq & CAN_STATUS_INTS)
        can_write(base, CAN_INT, diag->irq & CAN_STATUS_INTS);
}

static void can_init_one(uint32_t base)
{
    uint32_t nbtp;
    uint32_t reset_mask = base == CAN1_BASE ? CRU_CAN1_RESET_MASK : CRU_CAN2_RESET_MASK;

    /* Mode reset alone leaves a pending TX request latched after Bus-Off. */
    CRU_CAN_SOFTRST = (reset_mask << 16) | reset_mask;
    DWT_Delay(0.000002f);
    CRU_CAN_SOFTRST = reset_mask << 16;

    can_write(base, CAN_MODE, 0);
    can_write(base, CAN_CMD, 0);
    /* INT reads are masked on this IP. Keep flags visible for polling;
     * neither OS installs/enables a CAN GIC interrupt in this AMP setup. */
    can_write(base, CAN_INT_MASK, 0);
    can_write(base, CAN_INT, 0xffffffffU);

    can_write(base, CAN_IDCODE, 0);
    can_write(base, CAN_IDMASK, CAN_RX_FILTER_MASK);
    can_write(base, CAN_IDCODE0, 0);
    can_write(base, CAN_IDMASK0, CAN_RX_FILTER_MASK);
    can_write(base, CAN_IDCODE1, 0);
    can_write(base, CAN_IDMASK1, CAN_RX_FILTER_MASK);
    can_write(base, CAN_IDCODE2, 0);
    can_write(base, CAN_IDMASK2, CAN_RX_FILTER_MASK);
    can_write(base, CAN_IDCODE3, 0);
    can_write(base, CAN_IDMASK3, CAN_RX_FILTER_MASK);
    can_write(base, CAN_IDCODE4, 0);
    can_write(base, CAN_IDMASK4, CAN_RX_FILTER_MASK);
    can_write(base, CAN_RXFC, can_read(base, CAN_RXFC) | CAN_FIFO_ENABLE);
    can_write(base, CAN_AFC, 0);

    nbtp = (CAN_1M_NBRP_REG << NBTP_NBRP_SHIFT) |
           (CAN_1M_NSJW_REG << NBTP_NSJW_SHIFT) |
           (CAN_1M_NTSEG1_REG << NBTP_NTSEG1_SHIFT) |
           (CAN_1M_NTSEG2_REG << NBTP_NTSEG2_SHIFT);
    can_write(base, CAN_NBTP, nbtp);
    can_write(base, CAN_MODE, CAN_MODE_FDOE | CAN_WORK_MODE);
}

int amp_can_try_send(int bus, uint32_t id, const uint8_t *data, uint8_t len, int ide)
{
    uint32_t base = can_base(bus);
    uint32_t fic;
    uint32_t word;
    uint32_t i;
    unsigned int index;
    rt_base_t level;

    if (!base || !data || len > 8 || id > (ide ? 0x1fffffffU : 0x7ffU))
        return -RT_EINVAL;

    index = (unsigned int)bus - 1;
    level = rt_hw_interrupt_disable();
    can_sample_status(base, index);
    if (can_diagnostics[index].state & CAN_STATE_BUS_OFF) {
        can_diagnostics[index].tx_busy++;
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }
    if (can_read(base, CAN_CMD) & (CAN_TX0_REQ | CAN_TX1_REQ)) {
        can_diagnostics[index].tx_busy++;
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    fic = len & 0x0f;
    if (ide) {
        id &= 0x1fffffffU;
        fic |= CAN_FORMAT_EXT;
    } else {
        id &= 0x7ffU;
    }

    can_write(base, CAN_TXID, id);
    can_write(base, CAN_TXFIC, fic);
    for (i = 0; i < len; i += 4) {
        word = 0;
        if (i < len)
            word |= data[i];
        if (i + 1 < len)
            word |= (uint32_t)data[i + 1] << 8;
        if (i + 2 < len)
            word |= (uint32_t)data[i + 2] << 16;
        if (i + 3 < len)
            word |= (uint32_t)data[i + 3] << 24;
        can_write(base, CAN_TXDAT0 + i, word);
    }

    can_write(base, CAN_MODE, can_read(base, CAN_MODE) | CAN_MODE_SPACE_RX);
    can_write(base, CAN_CMD, CAN_TX0_REQ);
    can_write(base, CAN_MODE, can_read(base, CAN_MODE) & ~CAN_MODE_SPACE_RX);
    can_diagnostics[index].tx_submitted++;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

int amp_can_send(int bus, uint32_t id, const uint8_t *data, uint8_t len, int ide)
{
    uint32_t timeout = 1000000;
    int result;
    do {
        result = amp_can_try_send(bus, id, data, len, ide);
    } while (result == -RT_EBUSY && --timeout);
    return result == -RT_EBUSY ? -RT_ETIMEOUT : result;
}

int amp_can_recv(int bus, struct amp_can_frame *frame)
{
    uint32_t base = can_base(bus);
    uint32_t cnt;
    uint32_t fic, id, ts;
    uint32_t data[16];
    uint32_t i;
    unsigned int index;
    rt_base_t level;

    if (!base || !frame)
        return -RT_EINVAL;

    index = (unsigned int)bus - 1;
    level = rt_hw_interrupt_disable();
    can_sample_status(base, index);
    if (can_diagnostics[index].state & CAN_STATE_BUS_OFF) {
#ifdef AMP_CAN_LOOPBACK_TEST
        /* A failed self-test must never resume test frames in normal mode. */
        if (can_loopback_active[index]) {
            rt_hw_interrupt_enable(level);
            return 0;
        }
#endif
        uint32_t now = BSPTimeMillis();
        if ((uint32_t)(now - can_last_recovery_ms[index]) >= CAN_RECOVERY_MS) {
            can_last_recovery_ms[index] = now;
            can_diagnostics[index].recovery_attempts++;
            can_init_one(base);
            can_sample_status(base, index);
        }
        rt_hw_interrupt_enable(level);
        return 0;
    }
    rt_hw_interrupt_enable(level);

    cnt = (can_read(base, CAN_RXFC) & CAN_RX_FIFO_CNT_MASK) >> CAN_RX_FIFO_CNT_SHIFT;
    if (!cnt)
        return 0;

    fic = can_read(base, CAN_RXFRD);
    id = can_read(base, CAN_RXFRD);
    ts = can_read(base, CAN_RXFRD);
    (void)ts;
    for (i = 0; i < 16; i++)
        data[i] = can_read(base, CAN_RXFRD);

    frame->ide = !!(fic & CAN_FORMAT_EXT);
    frame->rtr = !!(fic & CAN_RTR);
    frame->len = fic & 0x0f;
    if ((fic & CAN_FORMAT_FD) || frame->len > 8)
        return -RT_EINVAL;
    frame->id = frame->ide ? (id & 0x1fffffffU) : (id & 0x7ffU);

    memset(frame->data, 0, sizeof(frame->data));
    for (i = 0; i < frame->len; i++)
        frame->data[i] = (uint8_t)(data[i / 4] >> ((i % 4) * 8));

    return 1;
}

int amp_can_get_diagnostic(int bus, struct amp_can_diagnostic *diagnostic)
{
    rt_base_t level;

    if (!can_base(bus) || !diagnostic)
        return -RT_EINVAL;
    level = rt_hw_interrupt_disable();
    *diagnostic = can_diagnostics[bus - 1];
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

#ifdef AMP_CAN_LOOPBACK_TEST
int amp_can_loopback_test(int bus, struct amp_can_loopback_result *result)
{
    uint32_t base = can_base(bus);
    unsigned int index, sequence, byte, wait;
    struct amp_can_frame frame;
    struct amp_can_diagnostic diag;
    uint8_t payload[8];
    rt_base_t level;
    int received, rc, passed;

    if (!base || !result)
        return -RT_EINVAL;
    index = (unsigned int)bus - 1;
    memset(result, 0, sizeof(*result));
    level = rt_hw_interrupt_disable();
    can_loopback_active[index] = 1;
    can_init_one(base);
    memset(&can_diagnostics[index], 0, sizeof(can_diagnostics[index]));
    can_bus_off_seen[index] = 0;
    can_write(base, CAN_MODE, 0);
    /* SELF_TEST supplies local ACK, LBACK loops TX to RX, RXSTX allows self RX. */
    can_write(base, CAN_MODE, CAN_MODE_FDOE | (1U << 2) | (1U << 4) |
              (1U << 5) | CAN_WORK_MODE);
    rt_hw_interrupt_enable(level);

    for (sequence = 0; sequence < 128; sequence++) {
        uint32_t id = 0x600U | (sequence & 0xfU);
        for (byte = 0; byte < sizeof(payload); byte++)
            payload[byte] = (uint8_t)(sequence * 17U + byte * 31U + bus * 32U);
        if (amp_can_try_send(bus, id, payload, sizeof(payload), 0) != RT_EOK) {
            result->timeouts++;
            break;
        }
        result->sent++;
        received = 0;
        for (wait = 0; wait < 20; wait++) {
            rc = amp_can_recv(bus, &frame);
            if (rc > 0) {
                result->received++;
                if (received || frame.id != id || frame.ide || frame.rtr ||
                    frame.len != sizeof(payload) ||
                    memcmp(frame.data, payload, sizeof(payload)))
                    result->mismatched++;
                received = 1;
            } else if (rc < 0) {
                result->mismatched++;
            }
            amp_can_get_diagnostic(bus, &diag);
            if (result->mismatched || diag.error_events || diag.bus_off_events ||
                diag.rx_error || diag.tx_error)
                goto restore;
            if (received && diag.tx_completed == result->sent)
                break;
            rt_thread_mdelay(1);
        }
        if (wait == 20) {
            result->timeouts++;
            break;
        }
    }

restore:
    amp_can_get_diagnostic(bus, &result->during);
    result->rxfc = can_read(base, CAN_RXFC);
    passed = result->sent == 128 && result->received == 128 &&
             result->during.tx_completed == 128 &&
             !result->mismatched && !result->timeouts &&
             !result->during.error_events && !result->during.bus_off_events &&
             !result->during.rx_error && !result->during.tx_error;
    level = rt_hw_interrupt_disable();
    can_init_one(base);
    can_loopback_active[index] = 0;
    memset(&can_diagnostics[index], 0, sizeof(can_diagnostics[index]));
    can_bus_off_seen[index] = 0;
    can_last_recovery_ms[index] = BSPTimeMillis();
    result->normal_mode = can_read(base, CAN_MODE);
    rt_hw_interrupt_enable(level);
    return passed && result->normal_mode == (CAN_MODE_FDOE | CAN_WORK_MODE) ?
           RT_EOK : -RT_ERROR;
}
#endif

int amp_periph_init(void)
{
    amp_pwm14_set(0);
    can_init_one(CAN1_BASE);
    can_init_one(CAN2_BASE);

    if (amp_spi3_init() == RT_EOK)
        amp_pwm14_set(300);

    return RT_EOK;
}

