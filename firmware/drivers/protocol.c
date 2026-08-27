#include "protocol.h"

#define CRC16_POLYNOMIAL 0xA001U

uint16_t robot_protocol_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t index = 0; index < length; index++) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc & 1U) != 0U
                ? (uint16_t) ((crc >> 1U) ^ CRC16_POLYNOMIAL)
                : (uint16_t) (crc >> 1U);
        }
    }

    return crc;
}

int robot_protocol_encode(const robot_frame_t *frame, uint8_t *buffer, uint16_t capacity)
{
    uint16_t total_length;

    if (frame == 0 || buffer == 0 || frame->payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD) {
        return -1;
    }

    total_length = (uint16_t) (ROBOT_PROTOCOL_HEADER_SIZE
        + frame->payload_length + ROBOT_PROTOCOL_CRC_SIZE);
    if (capacity < total_length) {
        return -1;
    }

    buffer[0] = ROBOT_PROTOCOL_SOF0;
    buffer[1] = ROBOT_PROTOCOL_SOF1;
    buffer[2] = ROBOT_PROTOCOL_VERSION;
    buffer[3] = frame->type;
    buffer[4] = (uint8_t) (frame->payload_length & 0xFFU);
    buffer[5] = (uint8_t) (frame->payload_length >> 8U);
    buffer[6] = frame->sequence;
    buffer[7] = frame->command;
    buffer[8] = frame->response_code;

    for (uint16_t index = 0; index < frame->payload_length; index++) {
        buffer[ROBOT_PROTOCOL_HEADER_SIZE + index] = frame->payload[index];
    }

    uint16_t crc = robot_protocol_crc16(buffer, total_length - ROBOT_PROTOCOL_CRC_SIZE);
    buffer[total_length - 2U] = (uint8_t) (crc & 0xFFU);
    buffer[total_length - 1U] = (uint8_t) (crc >> 8U);
    return (int) total_length;
}

void robot_protocol_parser_init(robot_protocol_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
    parser->has_last_sequence = 0U;
    parser->last_byte_tick = 0U;
}

static void parser_reset(robot_protocol_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

robot_protocol_result_t robot_protocol_parser_feed(
    robot_protocol_parser_t *parser,
    uint8_t byte,
    uint32_t tick,
    robot_frame_t *frame
)
{
    parser->last_byte_tick = tick;

    if (parser->length == 0U) {
        if (byte == ROBOT_PROTOCOL_SOF0) {
            parser->buffer[parser->length++] = byte;
        }
        return ROBOT_PROTOCOL_NEED_MORE;
    }

    if (parser->length == 1U && byte != ROBOT_PROTOCOL_SOF1) {
        parser_reset(parser);
        return ROBOT_PROTOCOL_BAD_FRAME;
    }

    parser->buffer[parser->length++] = byte;

    if (parser->length == ROBOT_PROTOCOL_HEADER_SIZE) {
        uint16_t payload_length = (uint16_t) parser->buffer[4]
            | (uint16_t) ((uint16_t) parser->buffer[5] << 8U);
        if (parser->buffer[2] != ROBOT_PROTOCOL_VERSION) {
            parser_reset(parser);
            return ROBOT_PROTOCOL_BAD_FRAME;
        }
        if (payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD) {
            parser_reset(parser);
            return ROBOT_PROTOCOL_OVERSIZE;
        }
        parser->expected_length = (uint16_t) (ROBOT_PROTOCOL_HEADER_SIZE
            + payload_length + ROBOT_PROTOCOL_CRC_SIZE);
    }

    if (parser->expected_length != 0U && parser->length == parser->expected_length) {
        uint16_t received_crc = (uint16_t) parser->buffer[parser->length - 2U]
            | (uint16_t) ((uint16_t) parser->buffer[parser->length - 1U] << 8U);
        uint16_t calculated_crc = robot_protocol_crc16(
            parser->buffer,
            (uint16_t) (parser->length - ROBOT_PROTOCOL_CRC_SIZE)
        );
        if (received_crc != calculated_crc) {
            parser_reset(parser);
            return ROBOT_PROTOCOL_BAD_FRAME;
        }

        frame->type = parser->buffer[3];
        frame->payload_length = (uint16_t) parser->buffer[4]
            | (uint16_t) ((uint16_t) parser->buffer[5] << 8U);
        frame->sequence = parser->buffer[6];
        frame->command = parser->buffer[7];
        frame->response_code = parser->buffer[8];
        for (uint16_t index = 0; index < frame->payload_length; index++) {
            frame->payload[index] = parser->buffer[ROBOT_PROTOCOL_HEADER_SIZE + index];
        }
        parser_reset(parser);

        if (parser->has_last_sequence && frame->sequence == parser->last_sequence) {
            return ROBOT_PROTOCOL_DUPLICATE;
        }
        parser->last_sequence = frame->sequence;
        parser->has_last_sequence = 1U;
        return ROBOT_PROTOCOL_FRAME_READY;
    }

    return ROBOT_PROTOCOL_NEED_MORE;
}

robot_protocol_result_t robot_protocol_parser_poll_timeout(
    robot_protocol_parser_t *parser,
    uint32_t tick,
    uint32_t timeout_ticks
)
{
    if (parser->length != 0U && (tick - parser->last_byte_tick) > timeout_ticks) {
        parser_reset(parser);
        return ROBOT_PROTOCOL_TIMEOUT;
    }

    return ROBOT_PROTOCOL_NEED_MORE;
}