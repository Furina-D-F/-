#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <stdint.h>

#define ROBOT_PROTOCOL_SOF0 0xAAU
#define ROBOT_PROTOCOL_SOF1 0x55U
#define ROBOT_PROTOCOL_VERSION 1U
#define ROBOT_PROTOCOL_MAX_PAYLOAD 128U
#define ROBOT_PROTOCOL_HEADER_SIZE 9U
#define ROBOT_PROTOCOL_CRC_SIZE 2U
#define ROBOT_PROTOCOL_MAX_FRAME \
    (ROBOT_PROTOCOL_HEADER_SIZE + ROBOT_PROTOCOL_MAX_PAYLOAD + ROBOT_PROTOCOL_CRC_SIZE)

typedef enum {
    ROBOT_FRAME_COMMAND = 0x01,
    ROBOT_FRAME_RESPONSE = 0x02,
    ROBOT_FRAME_STATUS = 0x03
} robot_frame_type_t;

typedef enum {
    ROBOT_CMD_MOTION = 0x01,
    ROBOT_CMD_CONFIG = 0x02,
    ROBOT_CMD_STATUS = 0x03
} robot_command_t;

typedef enum {
    ROBOT_STATUS_OK = 0x00,
    ROBOT_STATUS_BAD_LENGTH = 0x01,
    ROBOT_STATUS_BAD_COMMAND = 0x02,
    ROBOT_STATUS_BAD_CRC = 0x03,
    ROBOT_STATUS_TIMEOUT = 0x04,
    ROBOT_STATUS_DUPLICATE = 0x05,
        ROBOT_STATUS_OVERFLOW = 0x06,
        ROBOT_STATUS_INVALID_ARGUMENT = 0x07,
        ROBOT_STATUS_INVALID_STATE = 0x08,
        ROBOT_STATUS_LIMIT = 0x09
} robot_response_code_t;

typedef struct {
    uint8_t type;
    uint8_t sequence;
    uint8_t command;
    uint8_t response_code;
    uint16_t payload_length;
    uint8_t payload[ROBOT_PROTOCOL_MAX_PAYLOAD];
} robot_frame_t;

typedef enum {
    ROBOT_PROTOCOL_OK = 0,
    ROBOT_PROTOCOL_NEED_MORE = 1,
    ROBOT_PROTOCOL_FRAME_READY = 2,
    ROBOT_PROTOCOL_BAD_FRAME = -1,
    ROBOT_PROTOCOL_OVERSIZE = -2,
    ROBOT_PROTOCOL_TIMEOUT = -3,
    ROBOT_PROTOCOL_DUPLICATE = -4
} robot_protocol_result_t;

uint16_t robot_protocol_crc16(const uint8_t *data, uint16_t length);
int robot_protocol_encode(const robot_frame_t *frame, uint8_t *buffer, uint16_t capacity);

typedef struct {
    uint8_t buffer[ROBOT_PROTOCOL_MAX_FRAME];
    uint16_t length;
    uint16_t expected_length;
    uint8_t last_sequence;
    uint8_t has_last_sequence;
    uint32_t last_byte_tick;
} robot_protocol_parser_t;

void robot_protocol_parser_init(robot_protocol_parser_t *parser);
robot_protocol_result_t robot_protocol_parser_feed(
    robot_protocol_parser_t *parser,
    uint8_t byte,
    uint32_t tick,
    robot_frame_t *frame
);
robot_protocol_result_t robot_protocol_parser_poll_timeout(
    robot_protocol_parser_t *parser,
    uint32_t tick,
    uint32_t timeout_ticks
);

#endif