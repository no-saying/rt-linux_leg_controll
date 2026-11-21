/* Dedicated single-outstanding diagnostic data channel. All values LE32.
 * Linux owns request fields, RT owns response fields. Publish sequence last
 * with a release barrier, read it before an acquire barrier. No ring/log use.
 * Fixed slots remain reserved even after a timed-out Linux client exits. */
#ifndef AMP_FAST_PROTOCOL_H
#define AMP_FAST_PROTOCOL_H

#define AMP_FAST_BASE        0x07850000U
#define AMP_FAST_BYTES       0x10000U
#define AMP_FAST_MAGIC       0x41465001U
#define AMP_FAST_READY       0x000U
#define AMP_FAST_REQ_SEQ     0x040U
#define AMP_FAST_REQ_LEN     0x044U
#define AMP_FAST_RESP_SEQ    0x080U
#define AMP_FAST_RESP_LEN    0x084U
#define AMP_FAST_STATUS      0x088U
#define AMP_FAST_WAKE_TICKS  0x08cU
#define AMP_FAST_COPY_TICKS  0x090U
#define AMP_FAST_REQ_DATA    0x1000U
#define AMP_FAST_RESP_DATA   0x5000U
#define AMP_FAST_MAX_PAYLOAD 0x4000U

static inline int amp_fast_valid_length(unsigned int length)
{
    return length != 0 && length <= AMP_FAST_MAX_PAYLOAD;
}

#endif
