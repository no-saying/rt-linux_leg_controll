#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "amp_periph.h"

#define CAN_MX_REGISTER_CNT 32
#define DEVICE_CAN_CNT 2

typedef struct { unsigned int bus; } BSP_CAN_Handle;
extern BSP_CAN_Handle bsp_can1;
extern BSP_CAN_Handle bsp_can2;

typedef struct CANInstance CANInstance;
struct CANInstance {
    BSP_CAN_Handle *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t tx_buff[8];
    uint8_t rx_buff[8];
    uint8_t tx_len;
    uint8_t rx_len;
    void (*can_module_callback)(CANInstance *);
    void *id;
};

typedef struct {
    BSP_CAN_Handle *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

typedef struct {
    uint32_t tx_success_count; /* Accepted by controller, not proof of bus ACK. */
    uint32_t tx_failure_count;
    uint32_t last_tx_failure_age_ms;
    uint32_t rx_count;
    uint32_t unmatched_dropped;
} CAN_Diagnostic_s;

/* Register before starting MotorServiceInit(); callbacks run in its thread. */
CANInstance *CANRegister(const CAN_Init_Config_s *config);
void CANUnregister(CANInstance *instance); /* Initialization rollback only. */
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, float timeout_ms);
void CANGetDiagnostic(BSP_CAN_Handle *handle, CAN_Diagnostic_s *diagnostic);

/* One owner drains both hardware FIFOs and fans out shared feedback IDs. */
void CANServicePoll(void);
unsigned int CANDispatchFrame(BSP_CAN_Handle *handle, const struct amp_can_frame *frame);
int CANReceiveUnmatched(int bus, struct amp_can_frame *frame);

#endif
