#include "amp_application.h"

#include <math.h>
#include <rtthread.h>
#include <string.h>

enum parser_state {
    AMP_PARSE_SOF0 = 0,
    AMP_PARSE_SOF1,
    AMP_PARSE_HEADER,
    AMP_PARSE_PAYLOAD,
    AMP_PARSE_CRC,
};

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= (uint16_t)byte << 8;
    for (bit = 0; bit < 8; ++bit)
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                              : (uint16_t)(crc << 1);
    return crc;
}

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void write_le32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/* Decode IEEE-754 binary16 without using a compiler-specific half type. */
static int half_to_float(uint16_t bits, float *value)
{
    uint32_t sign = bits >> 15;
    uint32_t exponent = (bits >> 10) & 0x1fU;
    uint32_t fraction = bits & 0x3ffU;
    float result;

    if (exponent == 0x1fU)
        return -1; /* Inf and NaN are rejected by the wire contract. */
    if (exponent == 0U)
        result = ldexpf((float)fraction, -24);
    else
        result = ldexpf((float)(1024U + fraction), (int)exponent - 25);
    *value = sign ? -result : result;
    return isfinite(*value) ? 0 : -1;
}

static int sequence_is_new(const struct amp_application *app, uint16_t sequence)
{
    int16_t delta;

    if (!app->have_sequence)
        return 1;
    delta = (int16_t)(sequence - app->last_sequence);
    return delta > 0;
}

static void parser_reset(struct amp_application *app)
{
    app->parser_state = AMP_PARSE_SOF0;
    app->header_pos = 0;
    app->crc_pos = 0;
    app->payload_len = 0;
    app->payload_pos = 0;
    app->frame_valid = 1;
    app->header_reason = AMP_ACK_REASON_NONE;
    app->crc = 0xffffU;
}

static void set_active_state(struct amp_application *app, uint32_t vehicle,
                             uint32_t motion, uint32_t fault)
{
    app->active_vehicle_state = vehicle;
    app->active_motion_state = motion;
    app->last_fault_reason = fault;
}

static uint32_t make_ack(struct amp_application *app, uint8_t result,
                         uint8_t reason, uint32_t now_ms, uint8_t *reply,
                         uint32_t capacity)
{
    uint16_t crc = 0xffffU;
    uint32_t i;
    uint32_t payload_offset = 14U;
    uint32_t total = AMP_FRAME_OVERHEAD + AMP_ACK_PAYLOAD_BYTES;

    if (!reply || capacity < total)
        return 0;

    reply[0] = AMP_SOF0;
    reply[1] = AMP_SOF1;
    reply[2] = AMP_PROTOCOL_VERSION;
    reply[3] = AMP_MSG_COMMAND_ACK;
    write_le16(reply + 4, 0U);
    write_le16(reply + 6, AMP_ACK_PAYLOAD_BYTES);
    write_le16(reply + 8, app->frame_sequence);
    write_le32(reply + 10, now_ms);

    write_le16(reply + payload_offset, app->frame_sequence);
    reply[payload_offset + 2U] = result;
    reply[payload_offset + 3U] = reason;
    write_le32(reply + payload_offset + 4U, app->active_vehicle_state);
    write_le32(reply + payload_offset + 8U, app->active_motion_state);

    for (i = 2U; i < payload_offset + AMP_ACK_PAYLOAD_BYTES; ++i)
        crc = crc16_update(crc, reply[i]);
    write_le16(reply + payload_offset + AMP_ACK_PAYLOAD_BYTES, crc);
    return total;
}

static int finish_frame(struct amp_application *app, uint32_t now_ms,
                        uint8_t *reply, uint32_t reply_capacity,
                        uint32_t *reply_len)
{
    rt_base_t level = rt_hw_interrupt_disable();
    uint16_t wire_crc = read_le16(app->crc_wire);
    uint8_t reason = AMP_ACK_REASON_NONE;
    uint8_t result = AMP_ACK_REJECTED;
    float x_speed, y_speed, z_speed, leg_height;
    uint32_t vehicle_state, motion_state;

    if (wire_crc != app->crc)
        reason = AMP_ACK_REASON_CRC;
    else if (!app->frame_valid)
        reason = app->header_reason ? app->header_reason : AMP_ACK_REASON_VERSION;
    else if (app->payload_len != AMP_COMMAND_PAYLOAD_BYTES)
        reason = AMP_ACK_REASON_LENGTH;
    else if (!sequence_is_new(app, app->frame_sequence))
        reason = AMP_ACK_REASON_SEQUENCE;
    else if (half_to_float(read_le16(app->payload + 0), &x_speed) ||
             half_to_float(read_le16(app->payload + 2), &y_speed) ||
             half_to_float(read_le16(app->payload + 4), &z_speed) ||
             half_to_float(read_le16(app->payload + 6), &leg_height))
        reason = AMP_ACK_REASON_VALUE;
    else {
        vehicle_state = read_le32(app->payload + 8);
        motion_state = read_le32(app->payload + 12);
        if (vehicle_state > AMP_VEHICLE_FAULT ||
            motion_state > AMP_MOTION_STEP_DOWN)
            reason = AMP_ACK_REASON_STATE;
        else {
            app->command.x_speed_m_s = x_speed;
            app->command.y_speed_m_s = y_speed;
            app->command.z_speed_rad_s = z_speed;
            app->command.leg_height_m = leg_height;
            app->command.vehicle_state = vehicle_state;
            app->command.motion_state = motion_state;
            app->command.sequence = app->frame_sequence;
            app->command.timestamp_ms = app->frame_timestamp_ms;
            app->last_sequence = app->frame_sequence;
            app->have_sequence = 1;
            app->last_valid_ms = now_ms;
            app->timeout_latched = 0;
            set_active_state(app, vehicle_state, motion_state,
                             AMP_ACK_REASON_NONE);
            result = AMP_ACK_ACCEPTED;
        }
    }

    if (result == AMP_ACK_REJECTED)
        app->last_fault_reason = reason;
    if (reply_len)
        *reply_len = make_ack(app, result, reason, now_ms, reply,
                              reply_capacity);
    rt_hw_interrupt_enable(level);
    return AMP_APP_EVENT_ACK_READY;
}

void amp_application_init(struct amp_application *app)
{
    memset(app, 0, sizeof(*app));
    app->parser_state = AMP_PARSE_SOF0;
    app->frame_valid = 1;
    set_active_state(app, AMP_VEHICLE_DISABLED, AMP_MOTION_STOP, 0);
}

int amp_application_feed(struct amp_application *app, uint8_t byte,
                         uint32_t now_ms, uint8_t *reply,
                         uint32_t reply_capacity, uint32_t *reply_len)
{
    if (reply_len)
        *reply_len = 0;

    switch (app->parser_state) {
    case AMP_PARSE_SOF0:
        if (byte == AMP_SOF0)
            app->parser_state = AMP_PARSE_SOF1;
        return byte == AMP_SOF0 ? AMP_APP_EVENT_PARTIAL : AMP_APP_EVENT_NONE;
    case AMP_PARSE_SOF1:
        if (byte == AMP_SOF1) {
            app->parser_state = AMP_PARSE_HEADER;
            app->header_pos = 0;
            app->crc = 0xffffU;
            app->frame_valid = 1;
            return AMP_APP_EVENT_PARTIAL;
        }
        app->parser_state = byte == AMP_SOF0 ? AMP_PARSE_SOF1 : AMP_PARSE_SOF0;
        return app->parser_state == AMP_PARSE_SOF1 ? AMP_APP_EVENT_PARTIAL
                                                   : AMP_APP_EVENT_NONE;
    case AMP_PARSE_HEADER:
        app->header[app->header_pos] = byte;
        app->crc = crc16_update(app->crc, byte);
        if (app->header_pos == 0U && byte != AMP_PROTOCOL_VERSION) {
            app->frame_valid = 0;
            app->header_reason = AMP_ACK_REASON_VERSION;
        } else if (app->header_pos == 1U && byte != AMP_MSG_COMMAND_SETPOINT &&
                   app->header_reason == AMP_ACK_REASON_NONE) {
            app->frame_valid = 0;
            app->header_reason = AMP_ACK_REASON_TYPE;
        } else if ((app->header_pos == 2U || app->header_pos == 3U) &&
                   byte != 0U && app->header_reason == AMP_ACK_REASON_NONE) {
            app->frame_valid = 0;
            app->header_reason = AMP_ACK_REASON_FLAGS;
        }
        app->header_pos++;
        if (app->header_pos == sizeof(app->header)) {
            app->payload_len = read_le16(app->header + 4U);
            app->frame_sequence = read_le16(app->header + 6U);
            app->frame_timestamp_ms = read_le32(app->header + 8U);
            if (app->payload_len > AMP_MAX_PAYLOAD_BYTES) {
                rt_base_t level = rt_hw_interrupt_disable();
                uint32_t length = make_ack(app, AMP_ACK_REJECTED,
                                           AMP_ACK_REASON_LENGTH, now_ms,
                                           reply, reply_capacity);
                app->last_fault_reason = AMP_ACK_REASON_LENGTH;
                rt_hw_interrupt_enable(level);
                parser_reset(app);
                if (reply_len)
                    *reply_len = length;
                return AMP_APP_EVENT_ACK_READY;
            }
            app->payload_pos = 0;
            app->crc_pos = 0;
            app->parser_state = app->payload_len ? AMP_PARSE_PAYLOAD
                                                 : AMP_PARSE_CRC;
        }
        return AMP_APP_EVENT_PARTIAL;
    case AMP_PARSE_PAYLOAD:
        app->payload[app->payload_pos++] = byte;
        app->crc = crc16_update(app->crc, byte);
        if (app->payload_pos == app->payload_len) {
            app->crc_pos = 0;
            app->parser_state = AMP_PARSE_CRC;
        }
        return AMP_APP_EVENT_PARTIAL;
    case AMP_PARSE_CRC:
        app->crc_wire[app->crc_pos++] = byte;
        if (app->crc_pos == sizeof(app->crc_wire)) {
            int event = finish_frame(app, now_ms, reply, reply_capacity,
                                     reply_len);
            parser_reset(app);
            return event;
        }
        return AMP_APP_EVENT_PARTIAL;
    default:
        break;
    }

    return AMP_APP_EVENT_NONE;
}

void amp_application_tick(struct amp_application *app, uint32_t now_ms)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (app->have_sequence && (uint32_t)(now_ms - app->last_valid_ms) > 50U &&
        !app->timeout_latched) {
        app->timeout_latched = 1;
        set_active_state(app, AMP_VEHICLE_DISABLED, AMP_MOTION_STOP,
                         AMP_ACK_REASON_TIMEOUT);
    }
    rt_hw_interrupt_enable(level);
}

int amp_application_get_command(const struct amp_application *app,
                                struct amp_application_command *command)
{
    rt_base_t level;
    if (!app->have_sequence || !command)
        return -1;
    level = rt_hw_interrupt_disable();
    *command = app->command;
    rt_hw_interrupt_enable(level);
    return 0;
}

void amp_application_get_state(const struct amp_application *app,
                               uint32_t *vehicle_state,
                               uint32_t *motion_state,
                               uint32_t *fault_reason)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (vehicle_state)
        *vehicle_state = app->active_vehicle_state;
    if (motion_state)
        *motion_state = app->active_motion_state;
    if (fault_reason)
        *fault_reason = app->last_fault_reason;
    rt_hw_interrupt_enable(level);
}
