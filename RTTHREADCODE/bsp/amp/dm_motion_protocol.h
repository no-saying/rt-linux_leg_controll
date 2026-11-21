#ifndef DM_MOTION_PROTOCOL_H
#define DM_MOTION_PROTOCOL_H
#include <stdint.h>
/* Dedicated NC page inside the existing 4 MiB AMP no-map reservation.
 * Linux owns request fields; RT owns status. No mailbox or kernel change. */
#define DM_MOTION_BASE 0x07861000U
#define DM_MOTION_MAGIC 0x444d5401U
#define DM_MOTION_START 1U
#define DM_MOTION_STOP 2U
#define DM_MOTION_MAX_SPEED 1.0f
/* Host convenience command bound; duration remains entirely on Linux. */
#define DM_MOTION_MAX_DURATION_MS 15000U
enum dm_motion_state {
    DM_IDLE, DM_READ_LIMITS, DM_SET_MODE, DM_ARMING, DM_RUNNING,
    DM_BRAKING, DM_RESTORE, DM_DONE, DM_FAILED
};
enum dm_motion_error {
    DM_OK, DM_BAD_REQUEST, DM_OFFLINE, DM_DRIVE_FAULT,
    DM_SETUP_TIMEOUT, DM_PARAMETER_ERROR, DM_USER_STOP, DM_STOP_TIMEOUT
};
struct dm_motion_page {
    uint32_t magic, epoch;
    uint32_t request_epoch, command;
    float requested_speed;
    uint32_t reserved_request, request_seq;
    uint32_t reserved;
    /* seqlock: odd while RT updates; Linux must read an equal even pair. */
    uint32_t status_seq, accepted_seq, state, error;
    uint32_t now_ms, run_ms, feedback_age_ms, drive_state;
    uint32_t mode, confirmed, feedback_count, disabled;
    float command_speed, measured_speed, displacement;
    float p_max, v_max, t_max;
    uint32_t start_ms, stop_ms;
};
#endif
