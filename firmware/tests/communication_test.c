#include <assert.h>
#include <string.h>

#include "FreeRTOS.h"
#include "communication.h"

TickType_t xTaskGetTickCount(void)
{
    return 0U;
}

void vTaskDelay(const TickType_t ticks)
{
    (void) ticks;
}

static void put_float(uint8_t *payload, uint16_t offset, float value)
{
    const uint8_t *bytes = (const uint8_t *) &value;
    for (uint16_t index = 0U; index < sizeof(float); index++) {
        payload[offset + index] = bytes[index];
    }
}

static robot_frame_t exchange(
    robot_communication_t *communication,
    robot_uart_rx_ring_t *rx,
    const robot_frame_t *request
)
{
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    robot_frame_t response;
    robot_protocol_parser_t parser;
    int length = robot_protocol_encode(request, encoded, sizeof(encoded));

    assert(length > 0);
    for (int index = 0; index < length; index++) {
        assert(robot_uart_rx_isr_push(rx, encoded[index]) == ROBOT_UART_OK);
    }
    robot_communication_poll(communication, 0U);
    robot_protocol_parser_init(&parser);
    while (robot_uart_tx_available(communication->tx) != 0U) {
        uint8_t byte;
        assert(robot_uart_tx_read(communication->tx, &byte) == ROBOT_UART_OK);
        if (robot_protocol_parser_feed(&parser, byte, 0U, &response)
            == ROBOT_PROTOCOL_FRAME_READY) {
            return response;
        }
    }
    assert(0);
    return response;
}

int main(void)
{
    robot_uart_rx_ring_t rx;
    robot_uart_tx_ring_t tx;
    robot_communication_t communication;
    robot_frame_t request = {
        .type = ROBOT_FRAME_COMMAND,
        .sequence = 1U,
        .command = ROBOT_CMD_MOTION,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 34U
    };
    robot_frame_t response;

    request.payload[0] = 0U;
    request.payload[1] = 0x01U;
    put_float(request.payload, 2U, 1.0f);
    put_float(request.payload, 26U, 1.0f);
    put_float(request.payload, 30U, 1.0f);

    robot_communication_init(&communication, &rx, &tx);
    response = exchange(&communication, &rx, &request);
    assert(response.response_code == ROBOT_STATUS_OK);

    request.sequence = 2U;
    request.command = ROBOT_CMD_STATUS;
    request.payload_length = 0U;
    response = exchange(&communication, &rx, &request);
    assert(response.response_code == ROBOT_STATUS_OK);
    assert(response.payload_length == 50U);
    assert(response.payload[0] == ROBOT_CONTROL_RUNNING);
    assert(response.payload[1] == ROBOT_APP_OK);

    request.sequence = 3U;
    request.command = ROBOT_CMD_MOTION;
    request.payload_length = 34U;
    request.payload[0] = 1U;
    response = exchange(&communication, &rx, &request);
    assert(response.response_code == ROBOT_STATUS_OK);

    request.sequence = 4U;
    request.command = ROBOT_CMD_STATUS;
    request.payload_length = 0U;
    response = exchange(&communication, &rx, &request);
    assert(response.payload[0] == ROBOT_CONTROL_STOPPED);
    return 0;
}