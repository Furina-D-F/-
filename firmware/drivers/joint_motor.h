#ifndef ROBOT_JOINT_MOTOR_H
#define ROBOT_JOINT_MOTOR_H

#include <stdint.h>

#define ROBOT_JOINT_COUNT 6U
#define ROBOT_JOINT_ENCODER_COUNTS_PER_REV 4096L

typedef enum {
    ROBOT_JOINT_OK = 0,
    ROBOT_JOINT_INVALID_ID = -1,
    ROBOT_JOINT_INVALID_ARGUMENT = -2
} robot_joint_result_t;

typedef struct {
    int32_t encoder_count;
    float position_rad;
    float velocity_rad_s;
    float target_position_rad;
} robot_joint_state_t;

void robot_joint_init(uint8_t id);
robot_joint_result_t robot_joint_set_target(
    uint8_t id,
    float position_rad,
    float max_velocity_rad_s,
    float max_acceleration_rad_s2
);
robot_joint_result_t robot_joint_read_encoder(uint8_t id, int32_t *encoder_count);
robot_joint_result_t robot_joint_get_position(uint8_t id, float *position_rad);
robot_joint_result_t robot_joint_get_velocity(uint8_t id, float *velocity_rad_s);
robot_joint_result_t robot_joint_get_state(uint8_t id, robot_joint_state_t *state);
robot_joint_result_t robot_joint_stop(uint8_t id);
void robot_joint_update(float dt_s);

#endif