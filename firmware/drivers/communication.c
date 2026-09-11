#include "communication.h"
#include "robot_tasks.h"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef ROBOT_QEMU
#include "qemu_uart.h"
#endif

#define ROBOT_COMMUNICATION_FRAME_TIMEOUT_TICKS pdMS_TO_TICKS(500U)
#define ROBOT_CARTESIAN_LINE_PAYLOAD_LENGTH 64U
#define ROBOT_CARTESIAN_ARC_PAYLOAD_LENGTH 93U

static void copy_bytes(uint8_t *destination, const uint8_t *source, uint16_t length)
{
    for (uint16_t index = 0U; index < length; index++) {
        destination[index] = source[index];
    }
}

static int decode_pose(const uint8_t *payload, uint16_t offset, robot_pose_t *pose)
{
    float value[7];
    float norm;

    copy_bytes((uint8_t *) value, &payload[offset], sizeof(value));
    norm = sqrtf(value[3] * value[3] + value[4] * value[4]
        + value[5] * value[5] + value[6] * value[6]);
    if (!isfinite(value[0]) || !isfinite(value[1]) || !isfinite(value[2])
        || !isfinite(value[3]) || !isfinite(value[4]) || !isfinite(value[5])
        || !isfinite(value[6]) || !isfinite(norm) || norm <= 1.0e-6f) {
        return 0;
    }
    value[3] /= norm;
    value[4] /= norm;
    value[5] /= norm;
    value[6] /= norm;
    pose->value[0][0] = 1.0f - 2.0f * (value[4] * value[4] + value[5] * value[5]);
    pose->value[0][1] = 2.0f * (value[3] * value[4] - value[5] * value[6]);
    pose->value[0][2] = 2.0f * (value[3] * value[5] + value[4] * value[6]);
    pose->value[1][0] = 2.0f * (value[3] * value[4] + value[5] * value[6]);
    pose->value[1][1] = 1.0f - 2.0f * (value[3] * value[3] + value[5] * value[5]);
    pose->value[1][2] = 2.0f * (value[4] * value[5] - value[3] * value[6]);
    pose->value[2][0] = 2.0f * (value[3] * value[5] - value[4] * value[6]);
    pose->value[2][1] = 2.0f * (value[4] * value[5] + value[3] * value[6]);
    pose->value[2][2] = 1.0f - 2.0f * (value[3] * value[3] + value[4] * value[4]);
    pose->value[0][3] = value[0];
    pose->value[1][3] = value[1];
    pose->value[2][3] = value[2];
    pose->value[3][0] = 0.0f;
    pose->value[3][1] = 0.0f;
    pose->value[3][2] = 0.0f;
    pose->value[3][3] = 1.0f;
    return 1;
}

static uint8_t task_status_to_response(robot_tasks_status_t status)
{
    if (status == ROBOT_TASKS_OK) {
        return ROBOT_STATUS_OK;
    }
    if (status == ROBOT_TASKS_QUEUE_FULL) {
        return ROBOT_STATUS_OVERFLOW;
    }
    if (status == ROBOT_TASKS_INVALID_ARGUMENT) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    return ROBOT_STATUS_INVALID_STATE;
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

#ifdef ROBOT_QEMU
    bsp_qemu_uart_enable_tx_irq();
#endif

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
        robot_tasks_status_t task_result = robot_tasks_submit_motion(&command);
        send_response(communication, frame, task_status_to_response(task_result), 0, 0U);
    } else if (frame->command == ROBOT_CMD_CARTESIAN_LINE
        || frame->command == ROBOT_CMD_CARTESIAN_ARC) {
        uint16_t expected_length = frame->command == ROBOT_CMD_CARTESIAN_LINE
            ? ROBOT_CARTESIAN_LINE_PAYLOAD_LENGTH : ROBOT_CARTESIAN_ARC_PAYLOAD_LENGTH;
        robot_path_command_t command = {0};
        if (frame->payload_length != expected_length) {
            send_response(communication, frame, ROBOT_STATUS_BAD_LENGTH, 0, 0U);
            return;
        }
        command.type = frame->command == ROBOT_CMD_CARTESIAN_LINE
            ? ROBOT_PATH_CARTESIAN_LINE : ROBOT_PATH_CARTESIAN_ARC;
        if (!decode_pose(frame->payload, 0U, &command.start_pose)
            || !decode_pose(frame->payload, 28U, &command.end_pose)) {
            send_response(communication, frame, ROBOT_STATUS_INVALID_ARGUMENT, 0, 0U);
            return;
        }
        if (command.type == ROBOT_PATH_CARTESIAN_ARC
            && (!decode_pose(frame->payload, 56U, &command.center_pose)
                || frame->payload[84U] > 1U)) {
            send_response(communication, frame, ROBOT_STATUS_INVALID_ARGUMENT, 0, 0U);
            return;
        }
        command.direction = command.type == ROBOT_PATH_CARTESIAN_ARC
            ? frame->payload[84U] : 0U;
        copy_bytes((uint8_t *) &command.duration_s, &frame->payload[
            command.type == ROBOT_PATH_CARTESIAN_ARC ? 85U : 56U], sizeof(float));
        copy_bytes((uint8_t *) &command.period_s, &frame->payload[
            command.type == ROBOT_PATH_CARTESIAN_ARC ? 89U : 60U], sizeof(float));
        if (!isfinite(command.duration_s) || !isfinite(command.period_s)
            || command.duration_s <= 0.0f || command.period_s <= 0.0f) {
            send_response(communication, frame, ROBOT_STATUS_INVALID_ARGUMENT, 0, 0U);
            return;
        }
        robot_tasks_status_t task_result = robot_tasks_submit_cartesian(&command);
        send_response(communication, frame, task_status_to_response(task_result), 0, 0U);
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
        &communication->parser, tick, ROBOT_COMMUNICATION_FRAME_TIMEOUT_TICKS
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
        bsp_qemu_uart_service();
    #endif
        robot_communication_poll(communication, (uint32_t) xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}