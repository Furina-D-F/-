#ifndef ROBOT_JOINT_PID_H
#define ROBOT_JOINT_PID_H

#include <stdint.h>

typedef enum {
    ROBOT_PID_OK = 0,
    ROBOT_PID_INVALID_ARGUMENT = -1,
    ROBOT_PID_INVALID_CONFIG = -2
} robot_pid_status_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float deadband;
    float sample_time_s;
} robot_pid_config_t;

typedef struct {
    robot_pid_config_t config;
    float previous_error;
    float previous_previous_error;
    float integral;
    float output;
    uint8_t initialized;
} robot_joint_pid_t;

robot_pid_status_t robot_joint_pid_init(
    robot_joint_pid_t *controller,
    const robot_pid_config_t *config
);

robot_pid_status_t robot_joint_pid_update(
    robot_joint_pid_t *controller,
    float target_position_rad,
    float feedback_position_rad,
    float *output
);

void robot_joint_pid_reset(robot_joint_pid_t *controller);

#endif