#ifndef AMP_DIAG_H
#define AMP_DIAG_H

#include <stdint.h>

/* Persistent progress words exposed by Linux /sys/rk_amp/diag. */
#define AMP_DIAG_WORD(offset) (*(volatile uint32_t *)(0x07840000U + (offset)))
#define AMP_DIAG_TICK       0x10
#define AMP_DIAG_MOTOR      0x14
#define AMP_DIAG_MOTOR_STEP 0x18
#define AMP_DIAG_PERIPH     0x1c
#define AMP_DIAG_PERIPH_STEP 0x20
#define AMP_DIAG_HEARTBEAT  0x24
#define AMP_DIAG_IPC_STEP   0x28
#define AMP_DIAG_TX         0x2c
#define AMP_DIAG_DROP       0x30
#define AMP_DIAG_MAILBOX    0x34
#define AMP_DIAG_RX         0x38
#define AMP_DIAG_IRQ        0x40

#ifdef RT_USING_AMP_BAREMETAL
#define AMP_DIAG_SET(offset, value) (AMP_DIAG_WORD(offset) = (uint32_t)(value))
#define AMP_DIAG_INC(offset) (++AMP_DIAG_WORD(offset))
void amp_diag_init(void);
void amp_diag_fault(uint32_t kind, uint32_t pc, uint32_t cpsr);
#else
#define AMP_DIAG_SET(offset, value) ((void)0)
#define AMP_DIAG_INC(offset) ((void)0)
#endif

#endif
