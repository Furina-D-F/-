#ifndef ROBOT_CARTESIAN_TRAJECTORY_H
#define ROBOT_CARTESIAN_TRAJECTORY_H

#include <stdint.h>

#include "artificial_potential_field.h"
#include "kinematics.h"

#define ROBOT_CARTESIAN_JOINT_COUNT ROBOT_KINEMATICS_JOINT_COUNT

typedef enum {
    ROBOT_CARTESIAN_LINE = 0,
    ROBOT_CARTESIAN_ARC = 1
} robot_cartesian_path_type_t;

typedef enum {
    ROBOT_CARTESIAN_OK = 0,
    ROBOT_CARTESIAN_INVALID_ARGUMENT = -1,
    ROBOT_CARTESIAN_INVALID_PATH = -2,
    ROBOT_CARTESIAN_IK_FAILED = -3,
    ROBOT_CARTESIAN_COMPLETE = 1
} robot_cartesian_status_t;

typedef struct {
    robot_pose_t start_pose;
    robot_pose_t end_pose;
    robot_pose_t center_pose;
    float duration_s;
    float elapsed_s;
    float period_s;
    float current_joint[ROBOT_CARTESIAN_JOINT_COUNT];
    float output_joint[ROBOT_CARTESIAN_JOINT_COUNT];
    float previous_nominal[3];
    float previous_deviation[3];
    uint8_t previous_nominal_valid;
    uint8_t previous_deviation_valid;
    float arc_angle_rad;
    uint8_t direction;
    uint8_t active;
    robot_cartesian_path_type_t type;
    robot_apf_config_t apf;
} robot_cartesian_trajectory_t;

robot_cartesian_status_t robot_cartesian_plan_line(
    robot_cartesian_trajectory_t *trajectory,
    const robot_pose_t *start_pose,
    const robot_pose_t *end_pose,
    const float start_joint[ROBOT_CARTESIAN_JOINT_COUNT],
    float duration_s,
    float period_s
);

robot_cartesian_status_t robot_cartesian_plan_arc(
    robot_cartesian_trajectory_t *trajectory,
    const robot_pose_t *start_pose,
    const robot_pose_t *end_pose,
    const robot_pose_t *center_pose,
    uint8_t direction,
    const float start_joint[ROBOT_CARTESIAN_JOINT_COUNT],
    float duration_s,
    float period_s
);

robot_cartesian_status_t robot_cartesian_update(
    robot_cartesian_trajectory_t *trajectory,
    float dt_s,
    float output_joint[ROBOT_CARTESIAN_JOINT_COUNT]
);

int robot_cartesian_set_obstacles(
    robot_cartesian_trajectory_t *trajectory,
    const robot_apf_obstacle_t *obstacles,
    uint8_t count
);

void robot_cartesian_stop(robot_cartesian_trajectory_t *trajectory);

#endif