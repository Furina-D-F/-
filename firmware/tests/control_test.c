#include <assert.h>
#include <string.h>

#include "control.h"

static motion_command_t valid_command(void)
{
    motion_command_t command;
    memset(&command, 0, sizeof(command));
    command.mode = 0U;
    command.joint_mask = 0x01U;
    command.target_position_rad[0] = 1.0f;
    command.max_velocity_rad_s = 1.0f;
    command.max_acceleration_rad_s2 = 1.0f;
    return command;
}

int main(void)
{
    motion_command_t command = valid_command();
    robot_control_status_t status;

    robot_control_init();
    robot_control_get_status(&status);
    assert(status.state == ROBOT_CONTROL_IDLE);
    assert(robot_control_handle_motion(&command) == ROBOT_APP_OK);
    robot_control_update(0.1f);
    robot_control_get_status(&status);
    assert(status.state == ROBOT_CONTROL_RUNNING);
    assert(status.target_position_rad[0] == 1.0f);
    assert(status.position_rad[0] > 0.0f);
    assert(status.position_rad[0] < 1.0f);
    assert(status.velocity_rad_s[0] > 0.0f);

    command.target_position_rad[0] = 7.0f;
    assert(robot_control_handle_motion(&command) == ROBOT_APP_LIMIT);
    robot_control_get_status(&status);
    assert(status.target_position_rad[0] == 1.0f);

    assert(robot_control_stop() == ROBOT_APP_OK);
    robot_control_get_status(&status);
    assert(status.state == ROBOT_CONTROL_STOPPED);
    command = valid_command();
    assert(robot_control_handle_motion(&command) == ROBOT_APP_INVALID_STATE);
    return 0;
}