#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include <stdint.h>

#include "control.h"
#include "cartesian_trajectory.h"

typedef enum {
    ROBOT_TASKS_OK = 0,
    ROBOT_TASKS_INVALID_ARGUMENT = -1,
    ROBOT_TASKS_QUEUE_FULL = -2,
    ROBOT_TASKS_CREATE_FAILED = -3
} robot_tasks_status_t;

typedef struct {
    uint32_t pid_cycles;
    uint32_t path_commands;
    uint32_t status_samples;
    uint32_t health_faults;
} robot_tasks_health_t;

typedef enum {
    ROBOT_PATH_MOTION = 0,
    ROBOT_PATH_CARTESIAN_LINE = 1,
    ROBOT_PATH_CARTESIAN_ARC = 2
} robot_path_command_type_t;

typedef struct {
    robot_path_command_type_t type;
    motion_command_t motion;
    robot_pose_t start_pose;
    robot_pose_t end_pose;
    robot_pose_t center_pose;
    uint8_t direction;
    float duration_s;
    float period_s;
} robot_path_command_t;

robot_tasks_status_t robot_tasks_start(void);
void robot_tasks_bootstrap_task(void *argument);
robot_tasks_status_t robot_tasks_submit_motion(const motion_command_t *command);
robot_tasks_status_t robot_tasks_submit_cartesian(
    const robot_path_command_t *command);
robot_tasks_status_t robot_tasks_set_obstacles(
    const robot_apf_obstacle_t *obstacles, uint8_t count);
void robot_tasks_run_simulation_cycle(void);
void robot_tasks_get_health(robot_tasks_health_t *health);

#endif