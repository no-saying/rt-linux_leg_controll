/* SPDX-License-Identifier: BSD-3-Clause
 * Bounded PL330 byte peripheral programs. RK3568 requires burst handshakes
 * even with CCR burst length one (same quirk as Linux arm,pl330-periph-burst).
 */
#ifndef AMP_PL330_CODE_H
#define AMP_PL330_CODE_H
#include <stdint.h>
#include <stddef.h>

#define AMP_DMA_PROGRAM_BYTES 128U
#define AMP_DMA_MAX_BYTES 4096U

struct amp_dma_code { uint8_t bytes[AMP_DMA_PROGRAM_BYTES]; unsigned used; };

static inline void amp_code_byte(struct amp_dma_code *p, uint8_t value)
{
    p->bytes[p->used++] = value;
}

static inline void amp_code_mov(struct amp_dma_code *p, uint8_t reg, uint32_t value)
{
    unsigned i;
    amp_code_byte(p, 0xbc); amp_code_byte(p, reg);
    for (i = 0; i < 4; ++i) amp_code_byte(p, value >> (i * 8));
}

static inline void amp_code_body(struct amp_dma_code *p, unsigned peripheral, int rx)
{
    amp_code_byte(p, 0x32); /* DMAWFP burst */
    amp_code_byte(p, peripheral << 3);
    if (rx) {
        amp_code_byte(p, 0x27); /* DMALDP burst */
        amp_code_byte(p, peripheral << 3);
        amp_code_byte(p, 0x08); /* DMAST unconditional */
    } else {
        amp_code_byte(p, 0x04); /* DMALD unconditional */
        amp_code_byte(p, 0x2b); /* DMASTP burst */
        amp_code_byte(p, peripheral << 3);
    }
}

static inline void amp_code_loop(struct amp_dma_code *p, unsigned count,
                                 unsigned peripheral, int rx)
{
    unsigned begin;
    amp_code_byte(p, 0x20); amp_code_byte(p, count - 1); /* DMALP LC0 */
    begin = p->used;
    amp_code_body(p, peripheral, rx);
    amp_code_byte(p, 0x38); /* DMALPEND LC0, unconditional, finite */
    amp_code_byte(p, p->used - 1 - begin);
}

static inline int amp_pl330_program(struct amp_dma_code *p, unsigned peripheral,
                                    unsigned event, uint32_t memory,
                                    uint32_t fifo, unsigned length, int rx)
{
    unsigned begin, blocks;
    /* Nonsecure data accesses, one byte per request, increment RAM only. */
    uint32_t ccr = (1U << 9) | (1U << 23) | (rx ? (1U << 14) : 1U);
    if (!p || !length || length > AMP_DMA_MAX_BYTES || peripheral >= 32 || event >= 8)
        return -1;
    p->used = 0;
    amp_code_mov(p, 1, ccr);
    amp_code_mov(p, 0, rx ? fifo : memory);
    amp_code_mov(p, 2, rx ? memory : fifo);
    amp_code_byte(p, 0x35); amp_code_byte(p, peripheral << 3); /* DMAFLUSHP */
    blocks = length / 256;
    if (blocks) {
        amp_code_byte(p, 0x22); amp_code_byte(p, blocks - 1); /* LC1 */
        begin = p->used;
        amp_code_loop(p, 256, peripheral, rx);
        amp_code_byte(p, 0x3c);
        amp_code_byte(p, p->used - 1 - begin);
    }
    if (length % 256) amp_code_loop(p, length % 256, peripheral, rx);
    amp_code_byte(p, 0x13); /* DMAWMB: data committed before completion */
    amp_code_byte(p, 0x34); amp_code_byte(p, event << 3);
    amp_code_byte(p, 0x00); /* DMAEND */
    return (int)p->used;
}
#endif
