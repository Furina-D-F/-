#include <assert.h>

#include "joint_motor.h"

int main(void)
{
    robot_joint_state_t state;
    int32_t encoder_count;

    for (uint8_t id = 0U; id < ROBOT_JOINT_COUNT; id++) {
        robot_joint_init(id);
        assert(robot_joint_set_target(id, 1.0f, 2.0f, 4.0f) == ROBOT_JOINT_OK);
    }
    robot_joint_update(0.1f);

    for (uint8_t id = 0U; id < ROBOT_JOINT_COUNT; id++) {
        assert(robot_joint_read_encoder(id, &encoder_count) == ROBOT_JOINT_OK);
        assert(encoder_count > 0);
        assert(robot_joint_get_position(id, &state.position_rad) == ROBOT_JOINT_OK);
        assert(state.position_rad > 0.0f && state.position_rad < 1.0f);
        assert(robot_joint_get_velocity(id, &state.velocity_rad_s) == ROBOT_JOINT_OK);
        assert(state.velocity_rad_s > 0.0f);
        assert(robot_joint_get_state(id, &state) == ROBOT_JOINT_OK);
        assert(state.target_position_rad == 1.0f);
    }

    assert(robot_joint_read_encoder(ROBOT_JOINT_COUNT, &encoder_count)
        == ROBOT_JOINT_INVALID_ID);
    assert(robot_joint_get_velocity(0U, 0) == ROBOT_JOINT_INVALID_ARGUMENT);
    return 0;
}