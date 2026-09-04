#include <math.h>
#include <string.h>

#include "unity.h"
#include "joint_motor.h"
#include "protocol.h"
#include "uart.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static robot_frame_t make_frame(uint8_t sequence)
{
    robot_frame_t frame = {
        .type = ROBOT_FRAME_COMMAND,
        .sequence = sequence,
        .command = ROBOT_CMD_MOTION,
        .response_code = ROBOT_STATUS_OK,
        .payload_length = 3U,
        .payload = {0x10U, 0x20U, 0x30U}
    };
    return frame;
}

static int encode_frame(const robot_frame_t *frame, uint8_t *encoded)
{
    int length = robot_protocol_encode(frame, encoded, ROBOT_PROTOCOL_MAX_FRAME);
    TEST_ASSERT_GREATER_THAN_INT(0, length);
    return length;
}

void test_uart_rx_initializes_empty(void)
{
    robot_uart_rx_ring_t ring;
    robot_uart_init(&ring);
    TEST_ASSERT_EQUAL_UINT16(0U, robot_uart_available(&ring));
}

void test_uart_rx_preserves_order(void)
{
    robot_uart_rx_ring_t ring;
    uint8_t byte;
    robot_uart_init(&ring);
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_rx_isr_push(&ring, 0x11U));
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_rx_isr_push(&ring, 0x22U));
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_read(&ring, &byte));
    TEST_ASSERT_EQUAL_UINT8(0x11U, byte);
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_read(&ring, &byte));
    TEST_ASSERT_EQUAL_UINT8(0x22U, byte);
}

void test_uart_rx_rejects_full_buffer(void)
{
    robot_uart_rx_ring_t ring;
    robot_uart_init(&ring);
    for (uint16_t index = 0U; index < ROBOT_UART_RX_BUFFER_SIZE - 1U; index++) {
        TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_rx_isr_push(&ring, (uint8_t) index));
    }
    TEST_ASSERT_EQUAL(ROBOT_UART_FULL, robot_uart_rx_isr_push(&ring, 0xFFU));
    TEST_ASSERT_EQUAL_UINT16(ROBOT_UART_RX_BUFFER_SIZE - 1U, robot_uart_available(&ring));
}

void test_uart_rx_empty_and_null_read(void)
{
    robot_uart_rx_ring_t ring;
    uint8_t byte;
    robot_uart_init(&ring);
    TEST_ASSERT_EQUAL(ROBOT_UART_EMPTY, robot_uart_read(&ring, &byte));
    TEST_ASSERT_EQUAL(ROBOT_UART_EMPTY, robot_uart_read(&ring, NULL));
}

void test_uart_tx_preserves_order_and_empty(void)
{
    robot_uart_tx_ring_t ring;
    uint8_t byte;
    robot_uart_tx_init(&ring);
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_tx_write(&ring, 0xA5U));
    TEST_ASSERT_EQUAL(ROBOT_UART_OK, robot_uart_tx_read(&ring, &byte));
    TEST_ASSERT_EQUAL_UINT8(0xA5U, byte);
    TEST_ASSERT_EQUAL(ROBOT_UART_EMPTY, robot_uart_tx_read(&ring, &byte));
}

void test_protocol_round_trip_fragmented(void)
{
    robot_frame_t input = make_frame(7U);
    robot_frame_t output;
    robot_protocol_parser_t parser;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int length = encode_frame(&input, encoded);
    robot_protocol_parser_init(&parser);
    for (int index = 0; index < length - 1; index++) {
        TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_NEED_MORE,
            robot_protocol_parser_feed(&parser, encoded[index], (uint32_t) index, &output));
    }
    TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_FRAME_READY,
        robot_protocol_parser_feed(&parser, encoded[length - 1], 20U, &output));
    TEST_ASSERT_EQUAL_UINT8(input.sequence, output.sequence);
    TEST_ASSERT_EQUAL_UINT16(input.payload_length, output.payload_length);
    TEST_ASSERT_EQUAL_MEMORY(input.payload, output.payload, input.payload_length);
}

void test_protocol_rejects_invalid_arguments_and_capacity(void)
{
    robot_frame_t frame = make_frame(1U);
    uint8_t buffer[ROBOT_PROTOCOL_MAX_FRAME];
    TEST_ASSERT_EQUAL_INT(-1, robot_protocol_encode(NULL, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_INT(-1, robot_protocol_encode(&frame, NULL, sizeof(buffer)));
    TEST_ASSERT_EQUAL_INT(-1, robot_protocol_encode(&frame, buffer, 1U));
    frame.payload_length = ROBOT_PROTOCOL_MAX_PAYLOAD + 1U;
    TEST_ASSERT_EQUAL_INT(-1, robot_protocol_encode(&frame, buffer, sizeof(buffer)));
}

void test_protocol_rejects_bad_crc(void)
{
    robot_frame_t frame = make_frame(2U);
    robot_frame_t output;
    robot_protocol_parser_t parser;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int length = encode_frame(&frame, encoded);
    encoded[length - 1] ^= 1U;
    robot_protocol_parser_init(&parser);
    for (int index = 0; index < length; index++) {
        robot_protocol_result_t result = robot_protocol_parser_feed(
            &parser, encoded[index], 0U, &output);
        if (index == length - 1) {
            TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_BAD_FRAME, result);
        }
    }
}

void test_protocol_rejects_bad_version_and_oversize(void)
{
    robot_frame_t output;
    robot_protocol_parser_t parser;
    robot_protocol_parser_init(&parser);
    uint8_t bad_version[] = {ROBOT_PROTOCOL_SOF0, ROBOT_PROTOCOL_SOF1, 2U,
        ROBOT_FRAME_COMMAND, 0U, 0U, 0U, ROBOT_CMD_STATUS, ROBOT_STATUS_OK};
    for (uint32_t index = 0U; index < sizeof(bad_version); index++) {
        robot_protocol_result_t result = robot_protocol_parser_feed(
            &parser, bad_version[index], 0U, &output);
        if (index == sizeof(bad_version) - 1U) {
            TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_BAD_FRAME, result);
        }
    }

    robot_protocol_parser_init(&parser);
    uint8_t oversize[] = {ROBOT_PROTOCOL_SOF0, ROBOT_PROTOCOL_SOF1, ROBOT_PROTOCOL_VERSION,
        ROBOT_FRAME_COMMAND, 129U, 0U, 0U, ROBOT_CMD_STATUS, ROBOT_STATUS_OK};
    for (uint32_t index = 0U; index < sizeof(oversize); index++) {
        robot_protocol_result_t result = robot_protocol_parser_feed(
            &parser, oversize[index], index, &output);
        if (index == sizeof(oversize) - 1U) {
            TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_OVERSIZE, result);
        }
    }
}

void test_protocol_detects_timeout_and_duplicate(void)
{
    robot_frame_t frame = make_frame(3U);
    robot_frame_t output;
    robot_protocol_parser_t parser;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_FRAME];
    int length = encode_frame(&frame, encoded);
    robot_protocol_parser_init(&parser);
    TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_NEED_MORE,
        robot_protocol_parser_feed(&parser, encoded[0], 10U, &output));
    TEST_ASSERT_EQUAL(ROBOT_PROTOCOL_TIMEOUT,
        robot_protocol_parser_poll_timeout(&parser, 20U, 5U));

    robot_protocol_parser_init(&parser);
    for (int pass = 0; pass < 2; pass++) {
        robot_protocol_result_t result = ROBOT_PROTOCOL_NEED_MORE;
        for (int index = 0; index < length; index++) {
            result = robot_protocol_parser_feed(&parser, encoded[index], 0U, &output);
        }
        TEST_ASSERT_EQUAL(pass == 0 ? ROBOT_PROTOCOL_FRAME_READY : ROBOT_PROTOCOL_DUPLICATE, result);
    }
}

void test_motor_initializes_all_joints(void)
{
    robot_joint_state_t state;
    for (uint8_t id = 0U; id < ROBOT_JOINT_COUNT; id++) {
        robot_joint_init(id);
        TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(id, &state));
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.position_rad);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.velocity_rad_s);
    }
}

void test_motor_moves_with_acceleration_and_velocity_limits(void)
{
    robot_joint_state_t state;
    robot_joint_init(0U);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_set_target(0U, 1.0f, 0.5f, 1.0f));
    robot_joint_update(0.1f);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(0U, &state));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, state.position_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, state.velocity_rad_s);
    for (int index = 0; index < 20; index++) {
        robot_joint_update(0.1f);
    }
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(0U, &state));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, state.velocity_rad_s);
    TEST_ASSERT_TRUE(state.position_rad > 0.0f);
    TEST_ASSERT_TRUE(state.position_rad < 1.0f);
}

void test_motor_encoder_feedback_is_quantized(void)
{
    int32_t encoder_count;
    robot_joint_init(0U);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_set_target(0U, 1.0f, 1.0f, 1.0f));
    robot_joint_update(0.1f);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_read_encoder(0U, &encoder_count));
    TEST_ASSERT_TRUE(encoder_count > 0);
}

void test_motor_reaches_target_and_stop_clears_velocity(void)
{
    robot_joint_state_t state;
    robot_joint_init(0U);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_set_target(0U, 0.2f, 2.0f, 4.0f));
    for (int index = 0; index < 30; index++) {
        robot_joint_update(0.1f);
    }
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(0U, &state));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, state.position_rad);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_stop(0U));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(0U, &state));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.velocity_rad_s);
}

void test_motor_rejects_invalid_ids_and_pointers(void)
{
    robot_joint_state_t state;
    int32_t encoder_count;
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ID, robot_joint_set_target(ROBOT_JOINT_COUNT, 0, 1, 1));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ID, robot_joint_read_encoder(ROBOT_JOINT_COUNT, &encoder_count));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_read_encoder(0U, NULL));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_get_state(0U, NULL));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_get_position(0U, NULL));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_get_velocity(0U, NULL));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ID, robot_joint_stop(ROBOT_JOINT_COUNT));
    (void) state;
}

void test_motor_rejects_invalid_parameters_and_time(void)
{
    float position;
    robot_joint_state_t state;
    robot_joint_init(0U);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_set_target(0U, 0, 0, 1));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT, robot_joint_set_target(0U, 0, 1, 0));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_INVALID_ARGUMENT,
        robot_joint_set_target(0U, NAN, 1, 1));
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_position(0U, &position));
    robot_joint_update(0.0f);
    robot_joint_update(-0.1f);
    robot_joint_update(NAN);
    TEST_ASSERT_EQUAL(ROBOT_JOINT_OK, robot_joint_get_state(0U, &state));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.position_rad);
}

int main(void)
{
    UnityBegin("unity_driver_test.c");
    RUN_TEST(test_uart_rx_initializes_empty);
    RUN_TEST(test_uart_rx_preserves_order);
    RUN_TEST(test_uart_rx_rejects_full_buffer);
    RUN_TEST(test_uart_rx_empty_and_null_read);
    RUN_TEST(test_uart_tx_preserves_order_and_empty);
    RUN_TEST(test_protocol_round_trip_fragmented);
    RUN_TEST(test_protocol_rejects_invalid_arguments_and_capacity);
    RUN_TEST(test_protocol_rejects_bad_crc);
    RUN_TEST(test_protocol_rejects_bad_version_and_oversize);
    RUN_TEST(test_protocol_detects_timeout_and_duplicate);
    RUN_TEST(test_motor_initializes_all_joints);
    RUN_TEST(test_motor_moves_with_acceleration_and_velocity_limits);
    RUN_TEST(test_motor_encoder_feedback_is_quantized);
    RUN_TEST(test_motor_reaches_target_and_stop_clears_velocity);
    RUN_TEST(test_motor_rejects_invalid_ids_and_pointers);
    RUN_TEST(test_motor_rejects_invalid_parameters_and_time);
    return UnityEnd();
}
