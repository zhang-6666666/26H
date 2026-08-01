/* K230 -> STM32 vision protocol parser (hardware independent). */
#ifndef CAMERA_PROTOCOL_H
#define CAMERA_PROTOCOL_H

#include <stdint.h>

#define CAMERA_PROTOCOL_PAYLOAD_MAX  63U
#define CAMERA_PROTOCOL_TARGET_LIMIT_MM  125

typedef struct {
    uint16_t seq;
    uint32_t timestamp_ms;
    uint8_t valid;
    int16_t pos_mm;
    int16_t vel_mm_s;
    uint16_t confidence_permille;
    int16_t target_mm;
} CameraProtocolFrame;

typedef enum {
    CAMERA_PARSE_NONE = 0,
    CAMERA_PARSE_FRAME,
    CAMERA_PARSE_CHECKSUM_ERROR,
    CAMERA_PARSE_FORMAT_ERROR
} CameraParseResult;

typedef struct {
    uint8_t state;
    uint8_t payload_len;
    uint8_t calculated_checksum;
    uint8_t received_checksum;
    char payload[CAMERA_PROTOCOL_PAYLOAD_MAX + 1U];
} CameraProtocolParser;

void CameraProtocol_Init(CameraProtocolParser *parser);
CameraParseResult CameraProtocol_Push(CameraProtocolParser *parser,
                                      uint8_t byte,
                                      CameraProtocolFrame *frame);
uint8_t CameraProtocol_Xor(const char *payload);

#endif
