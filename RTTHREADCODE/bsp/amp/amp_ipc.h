/*
 * Minimal AMP shared-memory IPC for RK3568
 *
 * Ring buffer in AMP shared memory (0x07800000, 4MB)
 * Mailbox hardware (0xfe780000) for interrupt notifications
 *
 * AP-to-AP: B2A ch0 (GIC INTID 215) wakes CPU3 before reading L2R.
 * A2B targets the MCU side and cannot wake CPU3 through this GIC interrupt.
 * Linux reads R2L shared memory through the rockchip_amp diagnostic node.
 */
#ifndef __AMP_IPC_H__
#define __AMP_IPC_H__

#include <stdint.h>

/* Hardware addresses */
#define MAILBOX_BASE          0xfe780000
#define MAILBOX_REG(off)      (*(volatile uint32_t *)(MAILBOX_BASE + (off)))

/* Mailbox v2 registers */
#define MAILBOX_A2B_INTEN     MAILBOX_REG(0x00)
#define MAILBOX_A2B_STATUS    MAILBOX_REG(0x04)
#define MAILBOX_A2B_CMD(ch)   MAILBOX_REG(0x08 + (ch) * 8)
#define MAILBOX_A2B_DAT(ch)   MAILBOX_REG(0x0c + (ch) * 8)
#define MAILBOX_B2A_INTEN     MAILBOX_REG(0x28)
#define MAILBOX_B2A_STATUS    MAILBOX_REG(0x2c)
#define MAILBOX_B2A_CMD(ch)   MAILBOX_REG(0x30 + (ch) * 8)
#define MAILBOX_B2A_DAT(ch)   MAILBOX_REG(0x34 + (ch) * 8)

/* Both receivers are AP cores: Linux -> RT=B2A ch0, RT -> Linux=B2A ch3. */
#define MB_CH_L2R             0
#define MB_CH_R2L             3
#define MB_CMD_SEND           1

/* Shared memory ring buffer */
#define SHMEM_BASE            0x07800000
#define SHMEM_SIZE            0x00400000
#define RING_BUF_SIZE         0x00010000   /* 64KB ring */

/* Ring buffer (lock-free single-producer single-consumer) */
struct amp_ring {
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
    uint32_t size;
    uint8_t buf[0];
};

/* API */
int  amp_ipc_init(void);
int  amp_ipc_send(const void *data, uint32_t len);
int  amp_ipc_recv(void *buf, uint32_t max_len);
int  amp_ipc_is_ready(void);
void amp_ipc_send_log(const char *str);

/* ISR called from mailbox interrupt handler */
void amp_ipc_isr(void);

#endif
