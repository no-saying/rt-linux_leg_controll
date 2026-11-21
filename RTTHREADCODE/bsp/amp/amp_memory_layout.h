#ifndef AMP_MEMORY_LAYOUT_H
#define AMP_MEMORY_LAYOUT_H
#include <stdint.h>
#define AMP_SECTION_AP_RW (3U << 10)
#define AMP_SECTION_S     (1U << 16)
#define AMP_SECTION_XN    (1U << 4)
#define AMP_SECTION       2U
#define AMP_DEVICE       (AMP_SECTION | AMP_SECTION_AP_RW | AMP_SECTION_S | AMP_SECTION_XN | (1U << 2))
#define AMP_NORMAL_NC    (AMP_SECTION | AMP_SECTION_AP_RW | AMP_SECTION_S | AMP_SECTION_XN | (1U << 12))
#define AMP_NORMAL_WBWA  (AMP_SECTION | AMP_SECTION_AP_RW | AMP_SECTION_S | (1U << 12) | (3U << 2))
static inline uint32_t amp_section_descriptor(uint32_t section, int private_cache)
{
    uint32_t attributes = AMP_DEVICE;
    if (section >= 0x30 && section < 0x32)
        attributes = private_cache ? AMP_NORMAL_WBWA : (AMP_NORMAL_NC & ~AMP_SECTION_XN);
    else if (section >= 0x78 && section < 0x7c)
        attributes = AMP_NORMAL_NC;
    return (section << 20) | attributes;
}
#endif
