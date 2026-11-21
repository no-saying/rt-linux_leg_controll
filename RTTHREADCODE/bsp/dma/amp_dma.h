#ifndef AMP_DMA_H
#define AMP_DMA_H
#include <stdint.h>
#include <rtthread.h>
#include "amp_pl330_code.h"

#define AMP_DMA_CONTROL 0x07860000U
#define AMP_DMA_RT_MAGIC 0x52544401U
#define AMP_DMA_LINUX_MAGIC 0x4c444d01U
enum amp_dma_channel { AMP_DMA_SPI_TX, AMP_DMA_SPI_RX, AMP_DMA_UART_TX, AMP_DMA_UART_RX,
                       AMP_DMA_CHANNELS };
struct amp_dma_stats {
    uint32_t granted, started[AMP_DMA_CHANNELS], completed[AMP_DMA_CHANNELS];
    uint32_t timeouts, faults, stop_failures;
};
void amp_dma_init(void);
int amp_dma_available(void);
int amp_dma_faulted(void);
int amp_dma_prepare(unsigned channel, void *aligned_buffer, uint32_t length);
int amp_dma_start(unsigned channel);
/* Returns only after channel stop; error never permits reusing an active buffer. */
int amp_dma_wait(unsigned channel, uint64_t deadline);
int amp_dma_stop(unsigned channel);
uint32_t amp_dma_received(unsigned channel);
void amp_dma_invalidate(void *buffer, uint32_t length);
void amp_dma_get_stats(struct amp_dma_stats *stats);
#endif
