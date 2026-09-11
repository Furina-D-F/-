#include <math.h>
#include <string.h>

#include "unity.h"
#include "joint_motor.h"
#include "protocol.h"
#include "uart.h"
#include "kinematics.h"
#include "trajectory.h"
#include "cartesian_trajectory.h"
#include "artificial_potential_field.h"
#include "joint_pid.h"

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

void test_kinematics_fk_zero_pose(void)
{
    const float joint[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    robot_pose_t pose;

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK, robot_kinematics_fk(joint, &pose));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.81725f, pose.value[0][3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.19145f, pose.value[1][3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.005491f, pose.value[2][3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pose.value[3][3]);
}

void test_kinematics_ik_returns_fk_valid_solutions(void)
{
    const float joint[6] = {0.35f, -0.60f, 0.80f, -0.45f, 0.55f, -0.70f};
    robot_pose_t pose;
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS];
    uint8_t count = 0U;

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK, robot_kinematics_fk(joint, &pose));
    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_ik(&pose, joint, solutions, &count));
    TEST_ASSERT_TRUE(count > 0U);
    TEST_ASSERT_TRUE(count <= ROBOT_KINEMATICS_MAX_SOLUTIONS);
    for (uint8_t index = 0U; index < count; index++) {
        robot_pose_t candidate;
        TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
            robot_kinematics_fk(solutions[index].joint, &candidate));
        TEST_ASSERT_FLOAT_WITHIN(0.0003f, pose.value[0][3], candidate.value[0][3]);
        TEST_ASSERT_FLOAT_WITHIN(0.0003f, pose.value[1][3], candidate.value[1][3]);
        TEST_ASSERT_FLOAT_WITHIN(0.0003f, pose.value[2][3], candidate.value[2][3]);
        TEST_ASSERT_TRUE(robot_kinematics_joint_limits_ok(solutions[index].joint));
    }
}

void test_kinematics_rejects_joint_limits(void)
{
    const float invalid_joint[6] = {0.0f, 0.0f, 3.2f, 0.0f, 0.0f, 0.0f};
    robot_pose_t pose;

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_INVALID_ARGUMENT,
        robot_kinematics_fk(invalid_joint, &pose));
}

void test_kinematics_selects_shortest_safe_solution(void)
{
    const float current[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.8f, 0.0f};
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS] = {
        {.joint = {0.2f, 0.2f, 0.2f, 0.2f, 0.8f, 0.2f}},
        {.joint = {2.0f, 2.0f, 2.0f, 2.0f, 0.8f, 2.0f}}
    };
    robot_kinematics_solution_score_t score;
    uint8_t index = 0U;

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_select_best(solutions, 2U, current, &index, &score));
    TEST_ASSERT_EQUAL_UINT8(0U, index);
    TEST_ASSERT_TRUE(score.travel_cost < 1.0f);
    TEST_ASSERT_EQUAL_UINT8(0U, score.singular);
}

void test_kinematics_rejects_invalid_pose(void)
{
    robot_pose_t pose = {{{0.0f}}};
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS];
    uint8_t count = 0U;

    pose.value[3][3] = 1.0f;
    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_INVALID_POSE,
        robot_kinematics_ik(&pose, NULL, solutions, &count));
}

void test_kinematics_rejects_all_singular_solutions(void)
{
    const float current[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.8f, 0.0f};
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS] = {
        {.joint = {0.2f, 0.2f, 0.0f, 0.2f, 0.8f, 0.2f}}
    };
    robot_kinematics_solution_score_t score;
    uint8_t index = 0U;

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_SINGULAR,
        robot_kinematics_select_best(solutions, 1U, current, &index, &score));
}

void test_trajectory_cubic_meets_boundary_conditions(void)
{
    const float zero[6] = {0.0f};
    const float end[6] = {1.0f, -0.5f, 0.25f, 0.0f, 0.0f, 0.0f};
    robot_trajectory_polynomial_t trajectory;
    robot_trajectory_point_t point;

    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK, robot_trajectory_plan_cubic(
        &trajectory, zero, zero, end, zero, 2.0f));
    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK,
        robot_trajectory_sample_polynomial(&trajectory, 0.0f, &point));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, point.position[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, point.velocity[0]);
    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK,
        robot_trajectory_sample_polynomial(&trajectory, 2.0f, &point));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 1.0f, point.position[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, point.velocity[0]);
}

void test_trajectory_quintic_meets_acceleration_conditions(void)
{
    const float zero[6] = {0.0f};
    const float end[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    robot_trajectory_polynomial_t trajectory;
    robot_trajectory_point_t point;

    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK, robot_trajectory_plan_quintic(
        &trajectory, zero, zero, zero, end, zero, zero, 2.0f));
    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK,
        robot_trajectory_sample_polynomial(&trajectory, 2.0f, &point));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 1.0f, point.position[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, point.velocity[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, point.acceleration[0]);
}

void test_trajectory_trapezoid_respects_limits_and_syncs_axes(void)
{
    const float start[6] = {0.0f};
    const float end[6] = {1.0f, -0.5f, 0.25f, 0.0f, 0.0f, 0.0f};
    const float velocity[6] = {1.0f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f};
    const float acceleration[6] = {2.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    robot_trajectory_trapezoid_t trajectory;
    robot_trajectory_point_t point;

    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK, robot_trajectory_plan_trapezoid(
        &trajectory, start, end, velocity, acceleration));
    TEST_ASSERT_TRUE(trajectory.duration_s > 1.3f);
    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_OK,
        robot_trajectory_sample_trapezoid(&trajectory, trajectory.duration_s, &point));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, end[0], point.position[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, end[1], point.position[1]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, point.velocity[0]);
    TEST_ASSERT_TRUE(fabsf(point.velocity[1]) <= velocity[1] + 1.0e-5f);
}

void test_trajectory_rejects_invalid_constraints(void)
{
    const float zero[6] = {0.0f};
    robot_trajectory_polynomial_t polynomial;
    robot_trajectory_trapezoid_t trapezoid;
    float velocity[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f};
    float acceleration[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_INVALID_CONSTRAINT,
        robot_trajectory_plan_cubic(&polynomial, zero, zero, zero, zero, 0.0f));
    TEST_ASSERT_EQUAL(ROBOT_TRAJECTORY_INVALID_CONSTRAINT,
        robot_trajectory_plan_trapezoid(&trapezoid, zero, zero, velocity, acceleration));
}

void test_cartesian_line_outputs_periodic_joint_commands(void)
{
    const float start_joint[6] = {0.3f, -1.0f, 1.0f, -1.0f, 0.8f, 0.2f};
    const float end_joint[6] = {0.5f, -0.8f, 0.7f, -1.1f, 0.9f, 0.3f};
    robot_pose_t start_pose;
    robot_pose_t end_pose;
    robot_cartesian_trajectory_t trajectory;
    float output[6];

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_fk(start_joint, &start_pose));
    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_fk(end_joint, &end_pose));
    TEST_ASSERT_EQUAL(ROBOT_CARTESIAN_OK, robot_cartesian_plan_line(
        &trajectory, &start_pose, &end_pose, start_joint, 0.10f, 0.01f));
    for (uint8_t sample = 0U; sample < 10U; sample++) {
        robot_cartesian_status_t status = robot_cartesian_update(
            &trajectory, 0.01f, output);
        TEST_ASSERT_TRUE(status == ROBOT_CARTESIAN_OK
            || status == ROBOT_CARTESIAN_COMPLETE);
        TEST_ASSERT_TRUE(robot_kinematics_joint_limits_ok(output));
    }
    TEST_ASSERT_FALSE(trajectory.active);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-3f, end_joint[0], output[0]);
}

void test_cartesian_arc_interpolates_about_center(void)
{
    const float start_joint[6] = {0.3f, -1.0f, 1.0f, -1.0f, 0.8f, 0.2f};
    const float end_joint[6] = {-0.3f, -1.0f, 1.0f, -1.0f, 0.8f, 0.2f};
    robot_pose_t start_pose;
    robot_pose_t end_pose;
    robot_pose_t center_pose = {{
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    }};
    robot_cartesian_trajectory_t trajectory;
    robot_pose_t output_pose;
    float output[6];

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_fk(start_joint, &start_pose));
    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_fk(end_joint, &end_pose));
    center_pose.value[2][3] = start_pose.value[2][3];
    TEST_ASSERT_EQUAL(ROBOT_CARTESIAN_OK, robot_cartesian_plan_arc(
        &trajectory, &start_pose, &end_pose, &center_pose, 0U,
        start_joint, 0.20f, 0.01f));
    for (uint8_t sample = 0U; sample < 20U; sample++) {
        robot_cartesian_status_t status = robot_cartesian_update(
            &trajectory, 0.01f, output);
        TEST_ASSERT_TRUE(status == ROBOT_CARTESIAN_OK
            || status == ROBOT_CARTESIAN_COMPLETE);
    }
    TEST_ASSERT_FALSE(trajectory.active);
    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK,
        robot_kinematics_fk(output, &output_pose));
    TEST_ASSERT_FLOAT_WITHIN(2.0e-3f, end_pose.value[0][3], output_pose.value[0][3]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-3f, end_pose.value[1][3], output_pose.value[1][3]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-3f, end_pose.value[2][3], output_pose.value[2][3]);
}

void test_cartesian_rejects_invalid_period_and_ik_failure(void)
{
    const float joint[6] = {0.3f, -1.0f, 1.0f, -1.0f, 0.8f, 0.2f};
    robot_pose_t pose;
    robot_cartesian_trajectory_t trajectory;
    float output[6];

    TEST_ASSERT_EQUAL(ROBOT_KINEMATICS_OK, robot_kinematics_fk(joint, &pose));
    TEST_ASSERT_EQUAL(ROBOT_CARTESIAN_INVALID_PATH, robot_cartesian_plan_line(
        &trajectory, &pose, &pose, joint, 1.0f, 0.0f));
    pose.value[0][3] = 10.0f;
    TEST_ASSERT_EQUAL(ROBOT_CARTESIAN_OK, robot_cartesian_plan_line(
        &trajectory, &pose, &pose, joint, 0.1f, 0.01f));
    TEST_ASSERT_EQUAL(ROBOT_CARTESIAN_IK_FAILED,
        robot_cartesian_update(&trajectory, 0.01f, output));
}

void test_joint_pid_tracks_error_and_resets(void)
{
    const robot_pid_config_t config = {
        .kp = 2.0f, .ki = 0.5f, .kd = 0.1f,
        .integral_limit = 1.0f, .output_limit = 10.0f,
        .deadband = 0.0f, .sample_time_s = 0.01f
    };
    robot_joint_pid_t controller;
    float output = 0.0f;

    TEST_ASSERT_EQUAL(ROBOT_PID_OK, robot_joint_pid_init(&controller, &config));
    TEST_ASSERT_EQUAL(ROBOT_PID_OK,
        robot_joint_pid_update(&controller, 1.0f, 0.0f, &output));
    TEST_ASSERT_TRUE(output > 0.0f);
    robot_joint_pid_reset(&controller);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, controller.output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, controller.integral);
}

void test_joint_pid_applies_output_and_integral_limits(void)
{
    const robot_pid_config_t config = {
        .kp = 10.0f, .ki = 5.0f, .kd = 0.0f,
        .integral_limit = 0.05f, .output_limit = 0.2f,
        .deadband = 0.0f, .sample_time_s = 0.01f
    };
    robot_joint_pid_t controller;
    float output = 0.0f;

    TEST_ASSERT_EQUAL(ROBOT_PID_OK, robot_joint_pid_init(&controller, &config));
    for (uint8_t index = 0U; index < 100U; index++) {
        TEST_ASSERT_EQUAL(ROBOT_PID_OK,
            robot_joint_pid_update(&controller, 1.0f, 0.0f, &output));
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.2f, output);
    TEST_ASSERT_TRUE(fabsf(controller.integral) <= config.integral_limit + 1.0e-6f);
}

void test_joint_pid_deadband_suppresses_small_error(void)
{
    const robot_pid_config_t config = {
        .kp = 4.0f, .ki = 1.0f, .kd = 0.0f,
        .integral_limit = 1.0f, .output_limit = 2.0f,
        .deadband = 0.02f, .sample_time_s = 0.01f
    };
    robot_joint_pid_t controller;
    float output = 0.0f;

    TEST_ASSERT_EQUAL(ROBOT_PID_OK, robot_joint_pid_init(&controller, &config));
    TEST_ASSERT_EQUAL(ROBOT_PID_OK,
        robot_joint_pid_update(&controller, 0.01f, 0.0f, &output));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, output);
}

void test_joint_pid_rejects_invalid_configuration_and_feedback(void)
{
    robot_pid_config_t config = {
        .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f, .output_limit = 1.0f,
        .deadband = 0.0f, .sample_time_s = 0.01f
    };
    robot_joint_pid_t controller;
    float output = 0.0f;

    config.sample_time_s = 0.0f;
    TEST_ASSERT_EQUAL(ROBOT_PID_INVALID_CONFIG,
        robot_joint_pid_init(&controller, &config));
    config.sample_time_s = 0.01f;
    TEST_ASSERT_EQUAL(ROBOT_PID_OK, robot_joint_pid_init(&controller, &config));
    TEST_ASSERT_EQUAL(ROBOT_PID_INVALID_ARGUMENT,
        robot_joint_pid_update(&controller, NAN, 0.0f, &output));
}

void test_artificial_potential_field_repels_from_box(void)
{
    robot_apf_config_t config;
    robot_apf_obstacle_t obstacle = {
        .minimum = {-0.1f, -0.1f, -0.1f},
        .maximum = {0.1f, 0.1f, 0.1f},
        .clearance_m = 0.0f,
        .influence_radius = 0.5f,
        .repulsive_gain = 0.02f
    };
    const float current[3] = {-0.2f, 0.0f, 0.0f};
    const float target[3] = {0.2f, 0.0f, 0.0f};
    float adjusted[3];

    robot_apf_config_init(&config);
    TEST_ASSERT_EQUAL_INT(0, robot_apf_set_obstacles(&config, &obstacle, 1U));
    TEST_ASSERT_EQUAL_INT(0, robot_apf_adjust_target(
        &config, current, target, adjusted));
    TEST_ASSERT_TRUE(adjusted[0] < current[0]);
}

int robot_unity_run_all(void)
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
    RUN_TEST(test_kinematics_fk_zero_pose);
    RUN_TEST(test_kinematics_ik_returns_fk_valid_solutions);
    RUN_TEST(test_kinematics_rejects_joint_limits);
    RUN_TEST(test_kinematics_selects_shortest_safe_solution);
    RUN_TEST(test_kinematics_rejects_invalid_pose);
    RUN_TEST(test_kinematics_rejects_all_singular_solutions);
    RUN_TEST(test_trajectory_cubic_meets_boundary_conditions);
    RUN_TEST(test_trajectory_quintic_meets_acceleration_conditions);
    RUN_TEST(test_trajectory_trapezoid_respects_limits_and_syncs_axes);
    RUN_TEST(test_trajectory_rejects_invalid_constraints);
    RUN_TEST(test_cartesian_line_outputs_periodic_joint_commands);
    RUN_TEST(test_cartesian_arc_interpolates_about_center);
    RUN_TEST(test_cartesian_rejects_invalid_period_and_ik_failure);
    RUN_TEST(test_joint_pid_tracks_error_and_resets);
    RUN_TEST(test_joint_pid_applies_output_and_integral_limits);
    RUN_TEST(test_joint_pid_deadband_suppresses_small_error);
    RUN_TEST(test_joint_pid_rejects_invalid_configuration_and_feedback);
    RUN_TEST(test_artificial_potential_field_repels_from_box);
    return UnityEnd();
}

#ifndef ROBOT_QEMU_UNITY_TEST
int main(void)
{
    return robot_unity_run_all();
}
#endif
