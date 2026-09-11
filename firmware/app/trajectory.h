#ifndef ROBOT_TRAJECTORY_H
#define ROBOT_TRAJECTORY_H

#include <stdint.h>

#define ROBOT_TRAJECTORY_JOINT_COUNT 6U

typedef enum {
    ROBOT_TRAJECTORY_OK = 0,
    ROBOT_TRAJECTORY_INVALID_ARGUMENT = -1,
    ROBOT_TRAJECTORY_INVALID_CONSTRAINT = -2
} robot_trajectory_status_t;

typedef struct {
    float position[ROBOT_TRAJECTORY_JOINT_COUNT];
    float velocity[ROBOT_TRAJECTORY_JOINT_COUNT];
    float acceleration[ROBOT_TRAJECTORY_JOINT_COUNT];
} robot_trajectory_point_t;

typedef struct {
    float coefficient[ROBOT_TRAJECTORY_JOINT_COUNT][6];
    float duration_s;
} robot_trajectory_polynomial_t;

typedef struct {
    float start_position[ROBOT_TRAJECTORY_JOINT_COUNT];
    float distance[ROBOT_TRAJECTORY_JOINT_COUNT];
    float direction[ROBOT_TRAJECTORY_JOINT_COUNT];
    float peak_velocity[ROBOT_TRAJECTORY_JOINT_COUNT];
    float acceleration_time[ROBOT_TRAJECTORY_JOINT_COUNT];
    float cruise_time[ROBOT_TRAJECTORY_JOINT_COUNT];
    float profile_duration[ROBOT_TRAJECTORY_JOINT_COUNT];
    float duration_s;
    float max_velocity[ROBOT_TRAJECTORY_JOINT_COUNT];
    float max_acceleration[ROBOT_TRAJECTORY_JOINT_COUNT];
} robot_trajectory_trapezoid_t;

robot_trajectory_status_t robot_trajectory_plan_cubic(
    robot_trajectory_polynomial_t *trajectory,
    const float start_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float start_velocity[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_velocity[ROBOT_TRAJECTORY_JOINT_COUNT],
    float duration_s
);

robot_trajectory_status_t robot_trajectory_plan_quintic(
    robot_trajectory_polynomial_t *trajectory,
    const float start_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float start_velocity[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float start_acceleration[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_velocity[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_acceleration[ROBOT_TRAJECTORY_JOINT_COUNT],
    float duration_s
);

robot_trajectory_status_t robot_trajectory_sample_polynomial(
    const robot_trajectory_polynomial_t *trajectory,
    float time_s,
    robot_trajectory_point_t *point
);

robot_trajectory_status_t robot_trajectory_plan_trapezoid(
    robot_trajectory_trapezoid_t *trajectory,
    const float start_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float end_position[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float max_velocity[ROBOT_TRAJECTORY_JOINT_COUNT],
    const float max_acceleration[ROBOT_TRAJECTORY_JOINT_COUNT]
);

robot_trajectory_status_t robot_trajectory_sample_trapezoid(
    const robot_trajectory_trapezoid_t *trajectory,
    float time_s,
    robot_trajectory_point_t *point
);

#endif