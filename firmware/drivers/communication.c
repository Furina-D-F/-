#include "communication.h"

#include "FreeRTOS.h"
#include "task.h"

static int queue_frame(robot_communication_t *communication, const robot_frame_t *frame)
{
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int length = robot_protocol_encode(frame, encoded, sizeof(encoded));

    if (length < 0) {
        return -1;
    }

    for (int index = 0; index < length; index++) {
        if (robot_uart_tx_write(communication->tx, encoded[index]) != ROBOT_UART_OK) {
            return -1;
        }
    }

    return length;
}

static void send_response(
    robot_communication_t *communication,
    const robot_frame_t *request,
    uint8_t response_code
)
{
    robot_frame_t response = {
        .type = ROBOT_FRAME_RESPONSE,
        .sequence = request->sequence,
        .command = request->command,
        .response_code = response_code,
        .payload_length = 0U
    };
    (void) queue_frame(communication, &response);
}

static void handle_frame(robot_communication_t *communication, const robot_frame_t *frame)
{
    if (frame->type != ROBOT_FRAME_COMMAND) {
        send_response(communication, frame, ROBOT_STATUS_BAD_COMMAND);
        return;
    }

    if (frame->command != ROBOT_CMD_MOTION
        && frame->command != ROBOT_CMD_CONFIG
        && frame->command != ROBOT_CMD_STATUS) {
        send_response(communication, frame, ROBOT_STATUS_BAD_COMMAND);
        return;
    }

    send_response(communication, frame, ROBOT_STATUS_OK);
    communication->handled_frames++;
}

void robot_communication_init(
    robot_communication_t *communication,
    robot_uart_rx_ring_t *rx,
    robot_uart_tx_ring_t *tx
)
{
    communication->rx = rx;
    communication->tx = tx;
    communication->last_tick = 0U;
    communication->rx_errors = 0U;
    communication->duplicate_frames = 0U;
    communication->handled_frames = 0U;
    robot_protocol_parser_init(&communication->parser);
    robot_uart_init(rx);
    robot_uart_tx_init(tx);
}

void robot_communication_poll(robot_communication_t *communication, uint32_t tick)
{
    uint8_t byte;
    robot_frame_t frame;

    while (robot_uart_read(communication->rx, &byte) == ROBOT_UART_OK) {
        robot_protocol_result_t result = robot_protocol_parser_feed(
            &communication->parser, byte, tick, &frame
        );
        if (result == ROBOT_PROTOCOL_FRAME_READY) {
            handle_frame(communication, &frame);
        } else if (result == ROBOT_PROTOCOL_DUPLICATE) {
            communication->duplicate_frames++;
            send_response(communication, &frame, ROBOT_STATUS_DUPLICATE);
        } else if (result < 0) {
            communication->rx_errors++;
        }
    }

    if (robot_protocol_parser_poll_timeout(
        &communication->parser, tick, pdMS_TO_TICKS(100U)
    ) == ROBOT_PROTOCOL_TIMEOUT) {
        communication->rx_errors++;
    }
    communication->last_tick = tick;
}

int robot_communication_send_status(
    robot_communication_t *communication,
    uint8_t sequence,
    uint32_t task_counter,
    uint32_t timer_counter
)
{
    robot_frame_t status = {
        .type = ROBOT_FRAME_STATUS,
        .sequence = sequence,
        .command = ROBOT_CMD_STATUS,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 8U
    };

    status.payload[0] = (uint8_t) task_counter;
    status.payload[1] = (uint8_t) (task_counter >> 8U);
    status.payload[2] = (uint8_t) (task_counter >> 16U);
    status.payload[3] = (uint8_t) (task_counter >> 24U);
    status.payload[4] = (uint8_t) timer_counter;
    status.payload[5] = (uint8_t) (timer_counter >> 8U);
    status.payload[6] = (uint8_t) (timer_counter >> 16U);
    status.payload[7] = (uint8_t) (timer_counter >> 24U);
    return queue_frame(communication, &status);
}

void robot_communication_task(void *argument)
{
    robot_communication_t *communication = argument;

    for (;;) {
        robot_communication_poll(communication, (uint32_t) xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}