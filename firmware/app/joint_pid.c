#include "joint_pid.h"

#include <math.h>

#define PID_EPSILON 1.0e-6f

static float clamp_value(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int finite_config(const robot_pid_config_t *config)
{
    return isfinite(config->kp) && isfinite(config->ki) && isfinite(config->kd)
        && isfinite(config->integral_limit) && isfinite(config->output_limit)
        && isfinite(config->deadband) && isfinite(config->sample_time_s);
}

robot_pid_status_t robot_joint_pid_init(
    robot_joint_pid_t *controller,
    const robot_pid_config_t *config
)
{
    if (controller == 0 || config == 0) {
        return ROBOT_PID_INVALID_ARGUMENT;
    }
    if (!finite_config(config) || config->kp < 0.0f || config->ki < 0.0f
        || config->kd < 0.0f || config->integral_limit <= 0.0f
        || config->output_limit <= 0.0f || config->deadband < 0.0f
        || config->sample_time_s <= PID_EPSILON) {
        return ROBOT_PID_INVALID_CONFIG;
    }
    controller->config = *config;
    robot_joint_pid_reset(controller);
    return ROBOT_PID_OK;
}

robot_pid_status_t robot_joint_pid_update(
    robot_joint_pid_t *controller,
    float target_position_rad,
    float feedback_position_rad,
    float *output
)
{
    float error;
    float delta_output;
    float next_integral;
    float unclamped_output;

    if (controller == 0 || output == 0) {
        return ROBOT_PID_INVALID_ARGUMENT;
    }
    if (!isfinite(target_position_rad) || !isfinite(feedback_position_rad)) {
        return ROBOT_PID_INVALID_ARGUMENT;
    }
    error = target_position_rad - feedback_position_rad;
    if (fabsf(error) <= controller->config.deadband) {
        error = 0.0f;
    }
    next_integral = clamp_value(
        controller->integral + error * controller->config.sample_time_s,
        controller->config.integral_limit);
    delta_output = controller->config.kp
        * (error - controller->previous_error)
        + controller->config.ki * controller->config.sample_time_s * error;
    if (controller->initialized != 0U) {
        delta_output += controller->config.kd / controller->config.sample_time_s
            * (error - 2.0f * controller->previous_error
            + controller->previous_previous_error);
    }
    unclamped_output = controller->output + delta_output;
    if ((unclamped_output > controller->config.output_limit && error > 0.0f)
        || (unclamped_output < -controller->config.output_limit && error < 0.0f)) {
        next_integral = controller->integral;
    }
    controller->integral = next_integral;
    controller->output = clamp_value(controller->output + delta_output,
        controller->config.output_limit);
    if (controller->initialized == 0U) {
        controller->previous_previous_error = error;
    } else {
        controller->previous_previous_error = controller->previous_error;
    }
    controller->previous_error = error;
    controller->initialized = 1U;
    *output = controller->output;
    return ROBOT_PID_OK;
}

void robot_joint_pid_reset(robot_joint_pid_t *controller)
{
    if (controller != 0) {
        controller->previous_error = 0.0f;
        controller->previous_previous_error = 0.0f;
        controller->integral = 0.0f;
        controller->output = 0.0f;
        controller->initialized = 0U;
    }
}