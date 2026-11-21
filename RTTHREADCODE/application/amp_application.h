#ifndef AMP_APPLICATION_H
#define AMP_APPLICATION_H

#include <stdint.h>
#include "amp_protocol.h"

/* A validated command in the units used by the application layer. */
struct amp_application_command {
    float x_speed_m_s;
    float y_speed_m_s;
    float z_speed_rad_s;
    float leg_height_m;
    uint32_t vehicle_state;
    uint32_t motion_state;
    uint16_t sequence;
    uint32_t timestamp_ms;
};

/* Fixed-storage stream parser. It is safe to keep one instance in a thread or
 * static application object; it never allocates memory or blocks. */
struct amp_application {
    uint8_t parser_state;
    uint8_t header_pos;
    uint8_t crc_pos;
    uint8_t frame_valid;
    uint16_t payload_len;
    uint16_t payload_pos;
    uint16_t frame_sequence;
    uint32_t frame_timestamp_ms;
    uint8_t header[12];
    uint8_t header_reason;
    uint16_t crc;
    uint8_t crc_wire[2];
    uint8_t payload[AMP_MAX_PAYLOAD_BYTES];

    uint8_t have_sequence;
    uint8_t timeout_latched;
    uint16_t last_sequence;
    uint32_t last_valid_ms;
    uint32_t active_vehicle_state;
    uint32_t active_motion_state;
    uint32_t last_fault_reason;
    struct amp_application_command command;
};

enum amp_application_event {
    AMP_APP_EVENT_NONE = 0,
    AMP_APP_EVENT_PARTIAL,
    AMP_APP_EVENT_ACK_READY,
};

void amp_application_init(struct amp_application *app);

/* Feed one byte from amp_ipc_recv(). On a complete frame, reply receives a
 * complete AMP ACK frame and reply_len receives its byte count. */
int amp_application_feed(struct amp_application *app, uint8_t byte,
                         uint32_t now_ms, uint8_t *reply,
                         uint32_t reply_capacity, uint32_t *reply_len);

/* Enforce the 50 ms valid-command watchdog from AMP_PROTOCOL.md. */
void amp_application_tick(struct amp_application *app, uint32_t now_ms);

int amp_application_get_command(const struct amp_application *app,
                                struct amp_application_command *command);
void amp_application_get_state(const struct amp_application *app,
                               uint32_t *vehicle_state,
                               uint32_t *motion_state,
                               uint32_t *fault_reason);

#endif
