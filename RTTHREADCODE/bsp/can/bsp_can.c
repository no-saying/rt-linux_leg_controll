#include "bsp_can.h"
#include "bsp_dwt.h"
#include <math.h>
#include <rtthread.h>
#include <string.h>

#define RAW_QUEUE_LENGTH 16U
#define RX_POLL_BUDGET 32U

BSP_CAN_Handle bsp_can1 = {1};
BSP_CAN_Handle bsp_can2 = {2};
static CANInstance *instances[CAN_MX_REGISTER_CNT];
static struct {
    struct amp_can_frame frames[RAW_QUEUE_LENGTH];
    unsigned int read, write;
    CAN_Diagnostic_s diagnostic;
    uint32_t last_failure_ms;
} buses[DEVICE_CAN_CNT];

static int bus_index(const BSP_CAN_Handle *handle)
{
    if (handle == &bsp_can1)
        return 0;
    if (handle == &bsp_can2)
        return 1;
    return -1;
}

CANInstance *CANRegister(const CAN_Init_Config_s *config)
{
    unsigned int slot;
    CANInstance *instance;
    if (!config || bus_index(config->can_handle) < 0 ||
        config->tx_id > 0x7ffU || config->rx_id > 0x7ffU ||
        !config->can_module_callback)
        return RT_NULL;
    for (slot = 0; slot < CAN_MX_REGISTER_CNT; slot++) {
        CANInstance *other = instances[slot];
        if (other && other->can_handle == config->can_handle &&
            other->rx_id == config->rx_id &&
            other->can_module_callback != config->can_module_callback)
            return RT_NULL;
    }
    for (slot = 0; slot < CAN_MX_REGISTER_CNT && instances[slot]; slot++)
        ;
    if (slot == CAN_MX_REGISTER_CNT)
        return RT_NULL;
    instance = rt_calloc(1, sizeof(*instance));
    if (!instance)
        return RT_NULL;
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->tx_len = 8;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instances[slot] = instance;
    return instance;
}

void CANUnregister(CANInstance *instance)
{
    unsigned int i;
    if (!instance)
        return;
    for (i = 0; i < CAN_MX_REGISTER_CNT; i++) {
        if (instances[i] == instance) {
            instances[i] = RT_NULL;
            rt_free(instance);
            return;
        }
    }
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (instance && length <= 8)
        instance->tx_len = length;
}

uint8_t CANTransmit(CANInstance *instance, float timeout_ms)
{
    uint64_t start, timeout_us;
    int index, result;
    uint32_t id;
    uint8_t data[8], len;
    if (!instance || (index = bus_index(instance->can_handle)) < 0 ||
        instance->tx_id > 0x7ffU || instance->tx_len > 8 ||
        !isfinite(timeout_ms) || timeout_ms < 0.0f || timeout_ms > 1000.0f)
        return 0;
    id = instance->tx_id;
    len = instance->tx_len;
    memcpy(data, instance->tx_buff, sizeof(data));
    start = DWT_GetTimeline_us();
    timeout_us = (uint64_t)(timeout_ms * 1000.0f);
    do {
        result = amp_can_try_send(instance->can_handle->bus, id, data, len, 0);
        if (result != -RT_EBUSY)
            break;
    } while (DWT_GetTimeline_us() - start < timeout_us);
    if (result == RT_EOK) {
        buses[index].diagnostic.tx_success_count++;
        return 1;
    }
    buses[index].diagnostic.tx_failure_count++;
    buses[index].last_failure_ms = BSPTimeMillis();
    return 0;
}

unsigned int CANDispatchFrame(BSP_CAN_Handle *handle, const struct amp_can_frame *frame)
{
    unsigned int i, matched = 0;
    int index = bus_index(handle);
    if (index < 0 || !frame || frame->len > 8)
        return 0;
    buses[index].diagnostic.rx_count++;
    if (!frame->ide && !frame->rtr && frame->id <= 0x7ffU) {
        for (i = 0; i < CAN_MX_REGISTER_CNT; i++) {
            CANInstance *instance = instances[i];
            if (!instance || instance->can_handle != handle || instance->rx_id != frame->id)
                continue;
            memset(instance->rx_buff, 0, sizeof(instance->rx_buff));
            memcpy(instance->rx_buff, frame->data, frame->len);
            instance->rx_len = frame->len;
            instance->can_module_callback(instance);
            matched++;
        }
    }
    if (!matched) {
        unsigned int next = (buses[index].write + 1U) % RAW_QUEUE_LENGTH;
        if (next == buses[index].read)
            buses[index].diagnostic.unmatched_dropped++;
        else {
            buses[index].frames[buses[index].write] = *frame;
            buses[index].write = next;
        }
    }
    return matched;
}

void CANServicePoll(void)
{
    struct amp_can_frame frame;
    unsigned int i, bus;
    BSP_CAN_Handle *handles[DEVICE_CAN_CNT] = {&bsp_can1, &bsp_can2};
    for (bus = 0; bus < DEVICE_CAN_CNT; bus++) {
        for (i = 0; i < RX_POLL_BUDGET; i++) {
            if (amp_can_recv(handles[bus]->bus, &frame) <= 0)
                break;
            CANDispatchFrame(handles[bus], &frame);
        }
    }
}

int CANReceiveUnmatched(int bus, struct amp_can_frame *frame)
{
    int index = bus - 1;
    int result = 0;
    if (index < 0 || index >= DEVICE_CAN_CNT || !frame)
        return 0;
    rt_enter_critical();
    if (buses[index].read != buses[index].write) {
        *frame = buses[index].frames[buses[index].read];
        buses[index].read = (buses[index].read + 1U) % RAW_QUEUE_LENGTH;
        result = 1;
    }
    rt_exit_critical();
    return result;
}

void CANGetDiagnostic(BSP_CAN_Handle *handle, CAN_Diagnostic_s *diagnostic)
{
    int index = bus_index(handle);
    if (index < 0 || !diagnostic)
        return;
    rt_enter_critical();
    *diagnostic = buses[index].diagnostic;
    diagnostic->last_tx_failure_age_ms = diagnostic->tx_failure_count
        ? BSPTimeMillis() - buses[index].last_failure_ms : 0;
    rt_exit_critical();
}
