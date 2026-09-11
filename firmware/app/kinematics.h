#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>

#define ROBOT_KINEMATICS_JOINT_COUNT 6U
#define ROBOT_KINEMATICS_MAX_SOLUTIONS 8U
#define ROBOT_KINEMATICS_PI 3.14159265358979323846f
#define ROBOT_KINEMATICS_TWO_PI (2.0f * ROBOT_KINEMATICS_PI)

typedef struct {
    float value[4][4];
} robot_pose_t;

typedef struct {
    float joint[ROBOT_KINEMATICS_JOINT_COUNT];
} robot_kinematics_solution_t;

typedef struct {
    float travel_cost;
    float limit_cost;
    float singularity_cost;
    float total_cost;
    uint8_t singular;
} robot_kinematics_solution_score_t;

typedef enum {
    ROBOT_KINEMATICS_OK = 0,
    ROBOT_KINEMATICS_INVALID_ARGUMENT = -1,
    ROBOT_KINEMATICS_NO_SOLUTION = -2,
    ROBOT_KINEMATICS_SINGULAR = -3,
    ROBOT_KINEMATICS_JOINT_LIMIT = -4,
    ROBOT_KINEMATICS_INVALID_POSE = -5
} robot_kinematics_status_t;

robot_kinematics_status_t robot_kinematics_fk(
    const float joint[ROBOT_KINEMATICS_JOINT_COUNT],
    robot_pose_t *pose
);

robot_kinematics_status_t robot_kinematics_ik(
    const robot_pose_t *pose,
    const float current_joint[ROBOT_KINEMATICS_JOINT_COUNT],
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS],
    uint8_t *count
);

robot_kinematics_status_t robot_kinematics_select_best(
    const robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS],
    uint8_t count,
    const float current_joint[ROBOT_KINEMATICS_JOINT_COUNT],
    uint8_t *best_index,
    robot_kinematics_solution_score_t *score
);

robot_kinematics_status_t robot_kinematics_pose_validate(
    const robot_pose_t *pose
);

int robot_kinematics_joint_limits_ok(
    const float joint[ROBOT_KINEMATICS_JOINT_COUNT]
);

#endif
