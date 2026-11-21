#ifndef AMP_UART4_H
#define AMP_UART4_H
#include <stdint.h>
struct amp_uart4_stats {
    uint32_t tx_dma, tx_pio, rx_dma, rx_bytes, rx_dropped, line_errors, timeouts;
};
void amp_uart4_runtime_init(void);
void amp_uart4_get_stats(struct amp_uart4_stats *stats);
/* Fixed-length, thread-context RX. Returns partial bytes on timeout, or error.
 * Streaming callers continue using rt_hw_rs485_recv() and the IRQ RX ring. */
int rt_hw_rs485_recv_dma(void *data, uint32_t length, uint32_t timeout_ms);
#endif
