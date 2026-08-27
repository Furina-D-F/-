#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "protocol.h"
#include "uart.h"

static void test_encode_and_fragmented_parse(void)
{
    robot_frame_t input = {
        .type = ROBOT_FRAME_COMMAND,
        .sequence = 7U,
        .command = ROBOT_CMD_MOTION,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 3U,
        .payload = {0x10U, 0x20U, 0x30U}
    };
    robot_frame_t output;
    robot_protocol_parser_t parser;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int encoded_length = robot_protocol_encode(&input, encoded, sizeof(encoded));

    assert(encoded_length == 14);
    robot_protocol_parser_init(&parser);
    for (int index = 0; index < encoded_length - 1; index++) {
        assert(robot_protocol_parser_feed(&parser, encoded[index], (uint32_t) index, &output)
            == ROBOT_PROTOCOL_NEED_MORE);
    }
    assert(robot_protocol_parser_feed(&parser, encoded[encoded_length - 1], 20U, &output)
        == ROBOT_PROTOCOL_FRAME_READY);
    assert(output.sequence == input.sequence);
    assert(output.command == input.command);
    assert(memcmp(output.payload, input.payload, input.payload_length) == 0);
}

static void test_crc_timeout_and_duplicate(void)
{
    robot_frame_t frame = {
        .type = ROBOT_FRAME_COMMAND,
        .sequence = 1U,
        .command = ROBOT_CMD_STATUS,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 0U
    };
    robot_frame_t output;
    robot_protocol_parser_t parser;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int encoded_length = robot_protocol_encode(&frame, encoded, sizeof(encoded));

    robot_protocol_parser_init(&parser);
    encoded[encoded_length - 1] ^= 0x01U;
    for (int index = 0; index < encoded_length; index++) {
        robot_protocol_result_t result = robot_protocol_parser_feed(
            &parser, encoded[index], 0U, &output
        );
        if (index == encoded_length - 1) {
            assert(result == ROBOT_PROTOCOL_BAD_FRAME);
        }
    }

    robot_protocol_parser_init(&parser);
    assert(robot_protocol_parser_feed(&parser, ROBOT_PROTOCOL_SOF0, 10U, &output)
        == ROBOT_PROTOCOL_NEED_MORE);
    assert(robot_protocol_parser_poll_timeout(&parser, 20U, 5U) == ROBOT_PROTOCOL_TIMEOUT);

    encoded_length = robot_protocol_encode(&frame, encoded, sizeof(encoded));
    robot_protocol_parser_init(&parser);
    for (int pass = 0; pass < 2; pass++) {
        robot_protocol_result_t result = ROBOT_PROTOCOL_NEED_MORE;
        for (int index = 0; index < encoded_length; index++) {
            result = robot_protocol_parser_feed(&parser, encoded[index], 0U, &output);
        }
        assert(result == (pass == 0 ? ROBOT_PROTOCOL_FRAME_READY : ROBOT_PROTOCOL_DUPLICATE));
    }
}

static void test_uart_ring(void)
{
    robot_uart_rx_ring_t ring;
    uint8_t byte;

    robot_uart_init(&ring);
    assert(robot_uart_available(&ring) == 0U);
    assert(robot_uart_read(&ring, &byte) == ROBOT_UART_EMPTY);
    for (uint16_t index = 0; index < ROBOT_UART_RX_BUFFER_SIZE - 1U; index++) {
        assert(robot_uart_rx_isr_push(&ring, (uint8_t) index) == ROBOT_UART_OK);
    }
    assert(robot_uart_rx_isr_push(&ring, 0xFFU) == ROBOT_UART_FULL);
    assert(robot_uart_available(&ring) == ROBOT_UART_RX_BUFFER_SIZE - 1U);
    assert(robot_uart_read(&ring, &byte) == ROBOT_UART_OK);
    assert(byte == 0U);
}

int main(void)
{
    test_encode_and_fragmented_parse();
    test_crc_timeout_and_duplicate();
    test_uart_ring();
    return 0;
}