#include "trajectory.h"

#include <math.h>

#define TRAJECTORY_EPSILON 1.0e-6f

static int finite_array(const float values[ROBOT_TRAJECTORY_JOINT_COUNT])
{
    for (uint8_t index = 0U; index < ROBOT_TRAJECTORY_JOINT_COUNT; index++) {
        if (!isfinite(values[index])) {
            return 0;
        }
    }
    return 1;
}

static int valid_time(float time_s)
{
    return isfinite(time_s) && time_s > TRAJECTORY_EPSILON;
}

static float clamp_time(float time_s, float duration_s)
{
    if (time_s < 0.0f) {
        return 0.0f;
    }
    if (time_s > duration_s) {
        return duration_s;
    }
    return time_s;
}

robot_trajectory_status_t robot_trajectory_plan_cubic(
    robot_trajectory_polynomial_t *trajectory,
    const float start_position[6], const float start_velocity[6],
    const float end_position[6], const float end_velocity[6], float duration_s)
{
    if (trajectory == 0 || start_position == 0 || start_velocity == 0
        || end_position == 0 || end_velocity == 0) {
        return ROBOT_TRAJECTORY_INVALID_ARGUMENT;
    }
    if (!valid_time(duration_s) || !finite_array(start_position)
        || !finite_array(start_velocity) || !finite_array(end_position)
        || !finite_array(end_velocity)) {
        return ROBOT_TRAJECTORY_INVALID_CONSTRAINT;
    }
    trajectory->duration_s = duration_s;
    for (uint8_t index = 0U; index < 6U; index++) {
        float delta = end_position[index] - start_position[index];
        float t2 = duration_s * duration_s;
        float t3 = t2 * duration_s;
        trajectory->coefficient[index][0] = start_position[index];
        trajectory->coefficient[index][1] = start_velocity[index];
        trajectory->coefficient[index][2] = 3.0f * delta / t2
            - (2.0f * start_velocity[index] + end_velocity[index]) / duration_s;
        trajectory->coefficient[index][3] = -2.0f * delta / t3
            + (start_velocity[index] + end_velocity[index]) / t2;
        trajectory->coefficient[index][4] = 0.0f;
        trajectory->coefficient[index][5] = 0.0f;
    }
    return ROBOT_TRAJECTORY_OK;
}

robot_trajectory_status_t robot_trajectory_plan_quintic(
    robot_trajectory_polynomial_t *trajectory,
    const float start_position[6], const float start_velocity[6],
    const float start_acceleration[6], const float end_position[6],
    const float end_velocity[6], const float end_acceleration[6], float duration_s)
{
    if (trajectory == 0 || start_position == 0 || start_velocity == 0
        || start_acceleration == 0 || end_position == 0 || end_velocity == 0
        || end_acceleration == 0) {
        return ROBOT_TRAJECTORY_INVALID_ARGUMENT;
    }
    if (!valid_time(duration_s) || !finite_array(start_position)
        || !finite_array(start_velocity) || !finite_array(start_acceleration)
        || !finite_array(end_position) || !finite_array(end_velocity)
        || !finite_array(end_acceleration)) {
        return ROBOT_TRAJECTORY_INVALID_CONSTRAINT;
    }
    trajectory->duration_s = duration_s;
    for (uint8_t index = 0U; index < 6U; index++) {
        float t = duration_s;
        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t3 * t;
        float t5 = t4 * t;
        float delta = end_position[index] - start_position[index];
        float v0 = start_velocity[index];
        float vf = end_velocity[index];
        float a0 = start_acceleration[index];
        float af = end_acceleration[index];
        trajectory->coefficient[index][0] = start_position[index];
        trajectory->coefficient[index][1] = v0;
        trajectory->coefficient[index][2] = a0 * 0.5f;
        trajectory->coefficient[index][3] =
            (20.0f * delta - (8.0f * vf + 12.0f * v0) * t
             - (3.0f * a0 - af) * t2) / (2.0f * t3);
        trajectory->coefficient[index][4] =
            (-30.0f * delta + (14.0f * vf + 16.0f * v0) * t
             + (3.0f * a0 - 2.0f * af) * t2) / (2.0f * t4);
        trajectory->coefficient[index][5] =
            (12.0f * delta - 6.0f * (vf + v0) * t
             - (a0 - af) * t2) / (2.0f * t5);
    }
    return ROBOT_TRAJECTORY_OK;
}

robot_trajectory_status_t robot_trajectory_sample_polynomial(
    const robot_trajectory_polynomial_t *trajectory, float time_s,
    robot_trajectory_point_t *point)
{
    if (trajectory == 0 || point == 0 || !isfinite(time_s)
        || !valid_time(trajectory->duration_s)) {
        return ROBOT_TRAJECTORY_INVALID_ARGUMENT;
    }
    time_s = clamp_time(time_s, trajectory->duration_s);
    for (uint8_t index = 0U; index < 6U; index++) {
        float t = time_s;
        const float *c = trajectory->coefficient[index];
        point->position[index] = c[0] + t * (c[1] + t * (c[2]
            + t * (c[3] + t * (c[4] + t * c[5]))));
        point->velocity[index] = c[1] + t * (2.0f * c[2] + t * (3.0f * c[3]
            + t * (4.0f * c[4] + t * 5.0f * c[5])));
        point->acceleration[index] = 2.0f * c[2] + t * (6.0f * c[3]
            + t * (12.0f * c[4] + t * 20.0f * c[5]));
    }
    return ROBOT_TRAJECTORY_OK;
}

robot_trajectory_status_t robot_trajectory_plan_trapezoid(
    robot_trajectory_trapezoid_t *trajectory, const float start_position[6],
    const float end_position[6], const float max_velocity[6],
    const float max_acceleration[6])
{
    float duration = 0.0f;
    if (trajectory == 0 || start_position == 0 || end_position == 0
        || max_velocity == 0 || max_acceleration == 0) {
        return ROBOT_TRAJECTORY_INVALID_ARGUMENT;
    }
    if (!finite_array(start_position) || !finite_array(end_position)
        || !finite_array(max_velocity) || !finite_array(max_acceleration)) {
        return ROBOT_TRAJECTORY_INVALID_CONSTRAINT;
    }
    for (uint8_t index = 0U; index < 6U; index++) {
        float distance = fabsf(end_position[index] - start_position[index]);
        float peak_velocity;
        float acceleration_time;
        float cruise_time;
        if (max_velocity[index] <= 0.0f || max_acceleration[index] <= 0.0f) {
            return ROBOT_TRAJECTORY_INVALID_CONSTRAINT;
        }
        trajectory->start_position[index] = start_position[index];
        trajectory->distance[index] = distance;
        trajectory->direction[index] = end_position[index] >= start_position[index]
            ? 1.0f : -1.0f;
        trajectory->max_velocity[index] = max_velocity[index];
        trajectory->max_acceleration[index] = max_acceleration[index];
        if (distance <= TRAJECTORY_EPSILON) {
            peak_velocity = 0.0f;
            acceleration_time = 0.0f;
            cruise_time = 0.0f;
        } else {
            peak_velocity = fminf(max_velocity[index],
                sqrtf(distance * max_acceleration[index]));
            acceleration_time = peak_velocity / max_acceleration[index];
            cruise_time = (distance - peak_velocity * acceleration_time)
                / peak_velocity;
        }
        trajectory->peak_velocity[index] = peak_velocity;
        trajectory->acceleration_time[index] = acceleration_time;
        trajectory->cruise_time[index] = fmaxf(0.0f, cruise_time);
        trajectory->profile_duration[index] = 2.0f * acceleration_time
            + trajectory->cruise_time[index];
        duration = fmaxf(duration, trajectory->profile_duration[index]);
    }
    trajectory->duration_s = duration;
    return ROBOT_TRAJECTORY_OK;
}

robot_trajectory_status_t robot_trajectory_sample_trapezoid(
    const robot_trajectory_trapezoid_t *trajectory, float time_s,
    robot_trajectory_point_t *point)
{
    if (trajectory == 0 || point == 0 || !isfinite(time_s)
        || !isfinite(trajectory->duration_s)) {
        return ROBOT_TRAJECTORY_INVALID_ARGUMENT;
    }
    time_s = clamp_time(time_s, trajectory->duration_s);
    for (uint8_t index = 0U; index < 6U; index++) {
        if (trajectory->distance[index] <= TRAJECTORY_EPSILON) {
            point->position[index] = trajectory->start_position[index];
            point->velocity[index] = 0.0f;
            point->acceleration[index] = 0.0f;
            continue;
        }
        float scale = trajectory->duration_s > TRAJECTORY_EPSILON
            ? trajectory->duration_s / trajectory->profile_duration[index] : 1.0f;
        float local_time = scale > 1.0f ? time_s / scale : time_s;
        float ta = trajectory->acceleration_time[index];
        float tc = trajectory->cruise_time[index];
        float original_acceleration = trajectory->max_acceleration[index];
        float original_velocity = trajectory->peak_velocity[index];
        float distance;
        if (local_time < ta) {
            distance = 0.5f * original_acceleration * local_time * local_time;
            point->velocity[index] = original_acceleration * local_time / scale;
            point->acceleration[index] = original_acceleration / (scale * scale);
        } else if (local_time < ta + tc) {
            distance = 0.5f * original_acceleration * ta * ta
                + original_velocity * (local_time - ta);
            point->velocity[index] = original_velocity / scale;
            point->acceleration[index] = 0.0f;
        } else {
            float deceleration_time = fminf(local_time - ta - tc, ta);
            distance = 0.5f * original_acceleration * ta * ta
                + original_velocity * tc + original_velocity * deceleration_time
                - 0.5f * original_acceleration * deceleration_time
                * deceleration_time;
            point->velocity[index] = (original_velocity
                - original_acceleration * deceleration_time) / scale;
            point->acceleration[index] = -original_acceleration / (scale * scale);
        }
        point->position[index] = trajectory->start_position[index]
            + trajectory->direction[index] * distance;
        point->velocity[index] *= trajectory->direction[index];
        point->acceleration[index] *= trajectory->direction[index];
    }
    return ROBOT_TRAJECTORY_OK;
}