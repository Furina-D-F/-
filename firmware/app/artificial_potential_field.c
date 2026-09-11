#include "artificial_potential_field.h"

#include <math.h>

#define ROBOT_APF_EPSILON 1.0e-5f
#define ROBOT_APF_DEFAULT_ATTRACTIVE_GAIN 4.0f
#define ROBOT_APF_DEFAULT_STEP_LIMIT 0.025f

static float dot3(const float left[3], const float right[3])
{
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

static float norm3(const float value[3])
{
    return sqrtf(dot3(value, value));
}

static int finite_vector(const float value[3])
{
    return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

void robot_apf_config_init(robot_apf_config_t *config)
{
    if (config == 0) {
        return;
    }
    config->attractive_gain = ROBOT_APF_DEFAULT_ATTRACTIVE_GAIN;
    config->step_limit_m = ROBOT_APF_DEFAULT_STEP_LIMIT;
    config->obstacle_count = 0U;
}

int robot_apf_set_obstacles(
    robot_apf_config_t *config,
    const robot_apf_obstacle_t *obstacles,
    uint8_t count
)
{
    if (config == 0 || (count != 0U && obstacles == 0)
        || count > ROBOT_APF_MAX_OBSTACLES) {
        return -1;
    }
    for (uint8_t index = 0U; index < count; index++) {
        const robot_apf_obstacle_t *obstacle = &obstacles[index];
        if (!finite_vector(obstacle->minimum) || !finite_vector(obstacle->maximum)
            || obstacle->minimum[0] >= obstacle->maximum[0]
            || obstacle->minimum[1] >= obstacle->maximum[1]
            || obstacle->minimum[2] >= obstacle->maximum[2]
            || !isfinite(obstacle->clearance_m)
            || obstacle->clearance_m < 0.0f
            || !isfinite(obstacle->influence_radius)
            || !isfinite(obstacle->repulsive_gain)
            || obstacle->influence_radius <= ROBOT_APF_EPSILON
            || obstacle->repulsive_gain < 0.0f) {
            return -1;
        }
        config->obstacles[index] = *obstacle;
    }
    config->obstacle_count = count;
    return 0;
}

int robot_apf_adjust_target(
    const robot_apf_config_t *config,
    const float current_position[3],
    const float nominal_target[3],
    float adjusted_target[3]
)
{
    float force[3];

    if (config == 0 || current_position == 0 || nominal_target == 0
        || adjusted_target == 0 || config->obstacle_count > ROBOT_APF_MAX_OBSTACLES
        || !finite_vector(current_position) || !finite_vector(nominal_target)
        || !isfinite(config->attractive_gain) || config->attractive_gain <= 0.0f
        || !isfinite(config->step_limit_m) || config->step_limit_m <= 0.0f) {
        return -1;
    }
    if (config->obstacle_count == 0U) {
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            adjusted_target[axis] = nominal_target[axis];
        }
        return 0;
    }
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        force[axis] = config->attractive_gain
            * (nominal_target[axis] - current_position[axis]);
    }

    for (uint8_t index = 0U; index < config->obstacle_count; index++) {
        const robot_apf_obstacle_t *obstacle = &config->obstacles[index];
        float closest[3];
        float minimum[3];
        float maximum[3];
        float direction[3];
        float distance;
        float coefficient;
        int inside = 1;
        uint8_t nearest_axis = 0U;
        float nearest_distance = 1.0e30f;

        for (uint8_t axis = 0U; axis < 3U; axis++) {
            minimum[axis] = obstacle->minimum[axis] - obstacle->clearance_m;
            maximum[axis] = obstacle->maximum[axis] + obstacle->clearance_m;
            closest[axis] = fminf(maximum[axis],
                fmaxf(minimum[axis], current_position[axis]));
            if (current_position[axis] < minimum[axis]
                || current_position[axis] > maximum[axis]) {
                inside = 0;
            }
        }
        if (inside != 0) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                float distance_to_min = current_position[axis] - minimum[axis];
                float distance_to_max = maximum[axis] - current_position[axis];
                if (distance_to_min < nearest_distance) {
                    nearest_distance = distance_to_min;
                    nearest_axis = axis;
                    direction[0] = 0.0f;
                    direction[1] = 0.0f;
                    direction[2] = 0.0f;
                    direction[axis] = -1.0f;
                }
                if (distance_to_max < nearest_distance) {
                    nearest_distance = distance_to_max;
                    nearest_axis = axis;
                    direction[0] = 0.0f;
                    direction[1] = 0.0f;
                    direction[2] = 0.0f;
                    direction[axis] = 1.0f;
                }
            }
            (void) nearest_axis;
            distance = ROBOT_APF_EPSILON;
        } else {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                direction[axis] = current_position[axis] - closest[axis];
            }
            distance = norm3(direction);
            if (distance <= ROBOT_APF_EPSILON
                || distance >= obstacle->influence_radius) {
                continue;
            }
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                direction[axis] /= distance;
            }
        }
        coefficient = obstacle->repulsive_gain
            * (1.0f / distance - 1.0f / obstacle->influence_radius)
            / (distance * distance);
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            force[axis] += coefficient * direction[axis];
        }
    }

    {
        float force_norm = norm3(force);
        float step = fminf(config->step_limit_m, force_norm);
        if (force_norm <= ROBOT_APF_EPSILON) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                adjusted_target[axis] = current_position[axis];
            }
        } else {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                adjusted_target[axis] = current_position[axis]
                    + force[axis] * step / force_norm;
            }
        }
    }
    return 0;
}
