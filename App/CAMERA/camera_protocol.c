/*
 * Streaming parser for:
 *   $B,seq,t_ms,valid,x_mm,v_mm_s,confidence*CS\r\n
 * CS is the XOR of every payload byte between '$' and '*'.  A '$' received in
 * any state immediately starts a new frame, so line noise or a reset cannot
 * leave the receiver permanently misaligned.
 */
#include "camera_protocol.h"

#include <stddef.h>

enum {
    PARSER_WAIT_START = 0,
    PARSER_PAYLOAD,
    PARSER_CHECKSUM_HIGH,
    PARSER_CHECKSUM_LOW
};

static void parser_wait(CameraProtocolParser *parser)
{
    parser->state = PARSER_WAIT_START;
    parser->payload_len = 0U;
    parser->calculated_checksum = 0U;
    parser->received_checksum = 0U;
    parser->payload[0] = '\0';
}

static void parser_start(CameraProtocolParser *parser)
{
    parser->state = PARSER_PAYLOAD;
    parser->payload_len = 0U;
    parser->calculated_checksum = 0U;
    parser->received_checksum = 0U;
    parser->payload[0] = '\0';
}

static int8_t hex_nibble(uint8_t byte)
{
    if (byte >= (uint8_t)'0' && byte <= (uint8_t)'9') {
        return (int8_t)(byte - (uint8_t)'0');
    }
    if (byte >= (uint8_t)'A' && byte <= (uint8_t)'F') {
        return (int8_t)(byte - (uint8_t)'A' + 10U);
    }
    if (byte >= (uint8_t)'a' && byte <= (uint8_t)'f') {
        return (int8_t)(byte - (uint8_t)'a' + 10U);
    }
    return -1;
}

static uint8_t parse_u32(const char **cursor,
                         const char *end,
                         char delimiter,
                         uint32_t limit,
                         uint32_t *value_out)
{
    const char *p = *cursor;
    uint32_t value = 0U;
    uint8_t digits = 0U;

    while (p < end && *p >= '0' && *p <= '9') {
        uint32_t digit = (uint32_t)(*p - '0');
        if (digit > limit || value > (limit - digit) / 10U) {
            return 0U;
        }
        value = value * 10U + digit;
        digits++;
        p++;
    }
    if (digits == 0U) {
        return 0U;
    }

    if (delimiter == '\0') {
        if (p != end) {
            return 0U;
        }
    } else {
        if (p >= end || *p != delimiter) {
            return 0U;
        }
        p++;
    }

    *cursor = p;
    *value_out = value;
    return 1U;
}

static uint8_t parse_i16(const char **cursor,
                         const char *end,
                         char delimiter,
                         int16_t *value_out)
{
    const char *p = *cursor;
    uint8_t negative = 0U;
    uint32_t magnitude;
    uint32_t limit;

    if (p < end && (*p == '-' || *p == '+')) {
        negative = (*p == '-') ? 1U : 0U;
        p++;
    }
    limit = negative ? 32768U : 32767U;
    if (!parse_u32(&p, end, delimiter, limit, &magnitude)) {
        return 0U;
    }

    if (negative) {
        *value_out = (magnitude == 32768U)
                         ? (int16_t)(-32767 - 1)
                         : (int16_t)(-(int32_t)magnitude);
    } else {
        *value_out = (int16_t)magnitude;
    }
    *cursor = p;
    return 1U;
}

static uint8_t parse_payload(const char *payload,
                             uint8_t payload_len,
                             CameraProtocolFrame *frame)
{
    const char *cursor;
    const char *end = payload + payload_len;
    uint32_t seq;
    uint32_t timestamp_ms;
    uint32_t valid;
    uint32_t confidence;
    int16_t pos_mm;
    int16_t vel_mm_s;

    if (frame == NULL || payload_len < 3U || payload[0] != 'B' || payload[1] != ',') {
        return 0U;
    }
    cursor = payload + 2;

    if (!parse_u32(&cursor, end, ',', 65535U, &seq) ||
        !parse_u32(&cursor, end, ',', UINT32_MAX, &timestamp_ms) ||
        !parse_u32(&cursor, end, ',', 1U, &valid) ||
        !parse_i16(&cursor, end, ',', &pos_mm) ||
        !parse_i16(&cursor, end, ',', &vel_mm_s) ||
        !parse_u32(&cursor, end, '\0', 1000U, &confidence)) {
        return 0U;
    }

    /* Invalid measurements are required to carry zero data. */
    if (valid == 0U && (pos_mm != 0 || vel_mm_s != 0 || confidence != 0U)) {
        return 0U;
    }

    frame->seq = (uint16_t)seq;
    frame->timestamp_ms = timestamp_ms;
    frame->valid = (uint8_t)valid;
    frame->pos_mm = pos_mm;
    frame->vel_mm_s = vel_mm_s;
    frame->confidence_permille = (uint16_t)confidence;
    return 1U;
}

void CameraProtocol_Init(CameraProtocolParser *parser)
{
    if (parser != NULL) {
        parser_wait(parser);
    }
}

CameraParseResult CameraProtocol_Push(CameraProtocolParser *parser,
                                      uint8_t byte,
                                      CameraProtocolFrame *frame)
{
    int8_t nibble;
    CameraParseResult result;

    if (parser == NULL) {
        return CAMERA_PARSE_FORMAT_ERROR;
    }

    /* '$' always re-synchronizes, including in the middle of a bad frame. */
    if (byte == (uint8_t)'$') {
        parser_start(parser);
        return CAMERA_PARSE_NONE;
    }

    switch (parser->state) {
    case PARSER_WAIT_START:
        return CAMERA_PARSE_NONE;

    case PARSER_PAYLOAD:
        if (byte == (uint8_t)'*') {
            parser->payload[parser->payload_len] = '\0';
            parser->state = PARSER_CHECKSUM_HIGH;
            return CAMERA_PARSE_NONE;
        }
        if (byte == (uint8_t)'\r' || byte == (uint8_t)'\n' ||
            parser->payload_len >= CAMERA_PROTOCOL_PAYLOAD_MAX) {
            parser_wait(parser);
            return CAMERA_PARSE_FORMAT_ERROR;
        }
        parser->payload[parser->payload_len++] = (char)byte;
        parser->calculated_checksum ^= byte;
        return CAMERA_PARSE_NONE;

    case PARSER_CHECKSUM_HIGH:
        nibble = hex_nibble(byte);
        if (nibble < 0) {
            parser_wait(parser);
            return CAMERA_PARSE_FORMAT_ERROR;
        }
        parser->received_checksum = (uint8_t)((uint8_t)nibble << 4);
        parser->state = PARSER_CHECKSUM_LOW;
        return CAMERA_PARSE_NONE;

    case PARSER_CHECKSUM_LOW:
        nibble = hex_nibble(byte);
        if (nibble < 0) {
            parser_wait(parser);
            return CAMERA_PARSE_FORMAT_ERROR;
        }
        parser->received_checksum |= (uint8_t)nibble;
        if (parser->received_checksum != parser->calculated_checksum) {
            result = CAMERA_PARSE_CHECKSUM_ERROR;
        } else if (!parse_payload(parser->payload, parser->payload_len, frame)) {
            result = CAMERA_PARSE_FORMAT_ERROR;
        } else {
            result = CAMERA_PARSE_FRAME;
        }
        parser_wait(parser);
        return result;

    default:
        parser_wait(parser);
        return CAMERA_PARSE_FORMAT_ERROR;
    }
}

uint8_t CameraProtocol_Xor(const char *payload)
{
    uint8_t checksum = 0U;
    if (payload == NULL) {
        return 0U;
    }
    while (*payload != '\0') {
        checksum ^= (uint8_t)*payload++;
    }
    return checksum;
}
