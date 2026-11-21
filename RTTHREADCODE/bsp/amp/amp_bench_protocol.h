/* Fixed-size diagnostic echo; payload is opaque to RT-Thread. */
#ifndef AMP_BENCH_PROTOCOL_H
#define AMP_BENCH_PROTOCOL_H

#define AMP_BENCH_SIZE 64
#define AMP_BENCH_MAGIC_SIZE 8
#define AMP_BENCH_REQUEST "AMP_RTT?"
#define AMP_BENCH_REPLY   "AMP_RTT!"

struct amp_bench_parser {
    unsigned int used;
    unsigned char frame[AMP_BENCH_SIZE];
};

/* Handles split/coalesced ring reads and skips unrelated log bytes. */
static inline int amp_bench_feed(struct amp_bench_parser *parser,
                                unsigned char byte, const char *magic)
{
    if (parser->used < AMP_BENCH_MAGIC_SIZE &&
        byte != (unsigned char)magic[parser->used]) {
        parser->used = 0;
        if (byte != (unsigned char)magic[0])
            return 0;
    }
    parser->frame[parser->used++] = byte;
    if (parser->used != AMP_BENCH_SIZE)
        return 0;
    parser->used = 0;
    return 1;
}

#endif
