#include <stdio.h>
#include <stdint.h>

#include "protocol.h"
#include "uart.h"

static int send_response(const robot_frame_t *request)
{
    robot_frame_t response = {
        .type = ROBOT_FRAME_RESPONSE,
        .sequence = request->sequence,
        .command = request->command,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 0U
    };
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int length = robot_protocol_encode(&response, encoded, sizeof(encoded));

    if (length < 0) {
        return 1;
    }

    return fwrite(encoded, 1U, (size_t) length, stdout) == (size_t) length ? 0 : 1;
}

int main(void)
{
    robot_uart_rx_ring_t rx;
    robot_protocol_parser_t parser;
    robot_frame_t frame;
    robot_uart_init(&rx);
    robot_protocol_parser_init(&parser);

    for (;;) {
        int input = fgetc(stdin);
        if (input == EOF) {
            return ferror(stdin) != 0 ? 1 : 0;
        }

        if (robot_uart_rx_isr_push(&rx, (uint8_t) input) != ROBOT_UART_OK) {
            return 1;
        }

        uint8_t byte;
        while (robot_uart_read(&rx, &byte) == ROBOT_UART_OK) {
            robot_protocol_result_t result = robot_protocol_parser_feed(
                &parser, byte, 0U, &frame
            );
            if (result == ROBOT_PROTOCOL_FRAME_READY) {
                if (send_response(&frame) != 0) {
                    return 1;
                }
                fflush(stdout);
            }
        }
    }
}