#include "communication.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef ROBOT_QEMU
#include "qemu_uart.h"
#endif

static uint8_t map_app_result(robot_app_result_t result)
{
    if (result == ROBOT_APP_OK) {
        return ROBOT_STATUS_OK;
    }
    if (result == ROBOT_APP_INVALID_ARGUMENT || result == ROBOT_APP_LIMIT) {
        return ROBOT_STATUS_BAD_LENGTH;
    }
    return ROBOT_STATUS_BAD_COMMAND;
}

static void copy_bytes(uint8_t *destination, const uint8_t *source, uint16_t length)
{
    for (uint16_t index = 0U; index < length; index++) {
        destination[index] = source[index];
    }
}

static uint16_t append_status_payload(uint8_t *payload, const robot_control_status_t *status)
{
    uint16_t offset = 0U;
    payload[offset++] = (uint8_t) status->state;
    payload[offset++] = status->error_code;
    copy_bytes(&payload[offset], (const uint8_t *) status->position_rad, 24U);
    offset += 24U;
    copy_bytes(&payload[offset], (const uint8_t *) status->velocity_rad_s, 24U);
    return (uint16_t) (offset + 24U);
}

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
    uint8_t response_code,
    const uint8_t *payload,
    uint16_t payload_length
)
{
    robot_frame_t response = {
        .type = ROBOT_FRAME_RESPONSE,
        .sequence = request->sequence,
        .command = request->command,
        .response_code = response_code,
        .payload_length = payload_length
    };
    if (payload != 0 && payload_length <= ROBOT_PROTOCOL_MAX_PAYLOAD) {
        copy_bytes(response.payload, payload, payload_length);
    }
    (void) queue_frame(communication, &response);
}

static void handle_frame(robot_communication_t *communication, const robot_frame_t *frame)
{
    robot_app_result_t app_result;
    motion_command_t command;
    robot_control_status_t status;

    if (frame->type != ROBOT_FRAME_COMMAND) {
        send_response(communication, frame, ROBOT_STATUS_BAD_COMMAND, 0, 0U);
        return;
    }

    if (frame->command == ROBOT_CMD_STATUS) {
        if (frame->payload_length != 0U) {
            send_response(communication, frame, ROBOT_STATUS_BAD_LENGTH, 0, 0U);
            return;
        }
        robot_control_get_status(&status);
        uint8_t status_payload[50];
        uint16_t status_length = append_status_payload(status_payload, &status);
        send_response(communication, frame, ROBOT_STATUS_OK, status_payload, status_length);
    } else if (frame->command == ROBOT_CMD_MOTION) {
        if (frame->payload_length != 34U) {
            send_response(communication, frame, ROBOT_STATUS_BAD_LENGTH, 0, 0U);
            return;
        }
        command.mode = frame->payload[0];
        command.joint_mask = frame->payload[1];
        copy_bytes((uint8_t *) command.target_position_rad, &frame->payload[2], 24U);
        copy_bytes((uint8_t *) &command.max_velocity_rad_s, &frame->payload[26], sizeof(float));
        copy_bytes((uint8_t *) &command.max_acceleration_rad_s2, &frame->payload[30], sizeof(float));
        app_result = robot_control_handle_motion(&command);
        send_response(communication, frame, map_app_result(app_result), 0, 0U);
    } else {
        send_response(communication, frame, ROBOT_STATUS_BAD_COMMAND, 0, 0U);
        return;
    }

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
    robot_control_init();
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
            send_response(communication, &frame, ROBOT_STATUS_DUPLICATE, 0, 0U);
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
    robot_control_status_t control_status;
    robot_frame_t status = {
        .type = ROBOT_FRAME_STATUS,
        .sequence = sequence,
        .command = ROBOT_CMD_STATUS,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 58U
    };

    status.payload[0] = (uint8_t) task_counter;
    status.payload[1] = (uint8_t) (task_counter >> 8U);
    status.payload[2] = (uint8_t) (task_counter >> 16U);
    status.payload[3] = (uint8_t) (task_counter >> 24U);
    status.payload[4] = (uint8_t) timer_counter;
    status.payload[5] = (uint8_t) (timer_counter >> 8U);
    status.payload[6] = (uint8_t) (timer_counter >> 16U);
    status.payload[7] = (uint8_t) (timer_counter >> 24U);
    robot_control_get_status(&control_status);
    (void) append_status_payload(&status.payload[8], &control_status);
    return queue_frame(communication, &status);
}

void robot_communication_task(void *argument)
{
    robot_communication_t *communication = argument;

    for (;;) {
    #ifdef ROBOT_QEMU
        bsp_qemu_uart_poll_rx(communication->rx);
    #endif
        robot_communication_poll(communication, (uint32_t) xTaskGetTickCount());
    #ifdef ROBOT_QEMU
        bsp_qemu_uart_flush_tx(communication->tx);
    #endif
        robot_control_update(0.01f);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}