#ifndef ROBOT_ARTIFICIAL_POTENTIAL_FIELD_H
#define ROBOT_ARTIFICIAL_POTENTIAL_FIELD_H

#include <stdint.h>

#define ROBOT_APF_MAX_OBSTACLES 4U

typedef struct {
    float minimum[3];
    float maximum[3];
    float clearance_m;
    float influence_radius;
    float repulsive_gain;
} robot_apf_obstacle_t;

typedef struct {
    float attractive_gain;
    float step_limit_m;
    uint8_t obstacle_count;
    robot_apf_obstacle_t obstacles[ROBOT_APF_MAX_OBSTACLES];
} robot_apf_config_t;

void robot_apf_config_init(robot_apf_config_t *config);
int robot_apf_set_obstacles(
    robot_apf_config_t *config,
    const robot_apf_obstacle_t *obstacles,
    uint8_t count
);
int robot_apf_adjust_target(
    const robot_apf_config_t *config,
    const float current_position[3],
    const float nominal_target[3],
    float adjusted_target[3]
);

#endif
