#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

#define ROBOT_CONTROL_JOINT_COUNT 6U

typedef enum {
    ROBOT_CONTROL_INIT = 0,
    ROBOT_CONTROL_IDLE,
    ROBOT_CONTROL_RUNNING,
    ROBOT_CONTROL_STOPPED,
    ROBOT_CONTROL_ERROR
} robot_control_state_t;

typedef enum {
    ROBOT_APP_OK = 0,
    ROBOT_APP_INVALID_ARGUMENT = 1,
    ROBOT_APP_INVALID_STATE = 2,
    ROBOT_APP_LIMIT = 3
} robot_app_result_t;

typedef struct {
    uint8_t mode;
    uint8_t joint_mask;
    float target_position_rad[ROBOT_CONTROL_JOINT_COUNT];
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
} motion_command_t;

typedef struct {
    robot_control_state_t state;
    uint8_t error_code;
    float target_position_rad[ROBOT_CONTROL_JOINT_COUNT];
    float position_rad[ROBOT_CONTROL_JOINT_COUNT];
    float velocity_rad_s[ROBOT_CONTROL_JOINT_COUNT];
} robot_control_status_t;

void robot_control_init(void);
robot_app_result_t robot_control_handle_motion(const motion_command_t *command);
robot_app_result_t robot_control_start_velocity_control(void);
void robot_control_report_error(robot_app_result_t error_code);
robot_app_result_t robot_control_stop(void);
void robot_control_update(float dt_s);
robot_app_result_t robot_control_set_velocity_command(
    uint8_t joint_id,
    float velocity_rad_s
);
void robot_control_update_velocity_control(float dt_s);
void robot_control_set_simulation_feedback(
    const float position_rad[ROBOT_CONTROL_JOINT_COUNT],
    const float velocity_rad_s[ROBOT_CONTROL_JOINT_COUNT]
);
void robot_control_get_velocity_commands(
    float velocity_rad_s[ROBOT_CONTROL_JOINT_COUNT]
);
void robot_control_get_status(robot_control_status_t *status);

#endif