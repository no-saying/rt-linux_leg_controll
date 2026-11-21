#ifndef AMP_RING_COPY_H
#define AMP_RING_COPY_H
#include <stdint.h>
#include <string.h>

/* Caller validates count <= capacity and provides acquire/release barriers.
 * The ring leaves one byte unused, so index+count cannot exceed 2*capacity. */
static inline uint32_t amp_ring_copy_out(uint8_t *destination,
    const uint8_t *ring, uint32_t capacity, uint32_t index, uint32_t count)
{
    uint32_t first = count < capacity - index ? count : capacity - index;
    memcpy(destination, ring + index, first);
    memcpy(destination + first, ring, count - first);
    index += count;
    return index >= capacity ? index - capacity : index;
}

static inline uint32_t amp_ring_copy_in(uint8_t *ring, uint32_t capacity,
    uint32_t index, const uint8_t *source, uint32_t count)
{
    uint32_t first = count < capacity - index ? count : capacity - index;
    memcpy(ring + index, source, first);
    memcpy(ring, source + first, count - first);
    index += count;
    return index >= capacity ? index - capacity : index;
}
#endif
