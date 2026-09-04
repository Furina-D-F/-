#include "joint_motor.h"

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float target_position_rad;
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    int32_t encoder_count;
} robot_joint_instance_t;

#define ROBOT_PI 3.141592654f
#define ROBOT_TWO_PI (2.0f * ROBOT_PI)

static robot_joint_instance_t joints[ROBOT_JOINT_COUNT];

static int valid_id(uint8_t id)
{
    return id < ROBOT_JOINT_COUNT;
}

void robot_joint_init(uint8_t id)
{
    if (valid_id(id)) {
        joints[id].position_rad = 0.0f;
        joints[id].velocity_rad_s = 0.0f;
        joints[id].target_position_rad = 0.0f;
        joints[id].max_velocity_rad_s = 1.0f;
        joints[id].max_acceleration_rad_s2 = 1.0f;
        joints[id].encoder_count = 0;
    }
}

robot_joint_result_t robot_joint_set_target(
    uint8_t id,
    float position_rad,
    float max_velocity_rad_s,
    float max_acceleration_rad_s2
)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    if (position_rad != position_rad || max_velocity_rad_s != max_velocity_rad_s
        || max_acceleration_rad_s2 != max_acceleration_rad_s2
        || max_velocity_rad_s <= 0.0f || max_acceleration_rad_s2 <= 0.0f) {
        return ROBOT_JOINT_INVALID_ARGUMENT;
    }
    joints[id].target_position_rad = position_rad;
    joints[id].max_velocity_rad_s = max_velocity_rad_s;
    joints[id].max_acceleration_rad_s2 = max_acceleration_rad_s2;
    return ROBOT_JOINT_OK;
}

robot_joint_result_t robot_joint_read_encoder(uint8_t id, int32_t *encoder_count)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    if (encoder_count == 0) {
        return ROBOT_JOINT_INVALID_ARGUMENT;
    }
    *encoder_count = joints[id].encoder_count;
    return ROBOT_JOINT_OK;
}

robot_joint_result_t robot_joint_get_position(uint8_t id, float *position_rad)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    if (position_rad == 0) {
        return ROBOT_JOINT_INVALID_ARGUMENT;
    }
    *position_rad = ((float) joints[id].encoder_count * ROBOT_TWO_PI)
        / (float) ROBOT_JOINT_ENCODER_COUNTS_PER_REV;
    return ROBOT_JOINT_OK;
}

robot_joint_result_t robot_joint_get_velocity(uint8_t id, float *velocity_rad_s)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    if (velocity_rad_s == 0) {
        return ROBOT_JOINT_INVALID_ARGUMENT;
    }
    *velocity_rad_s = joints[id].velocity_rad_s;
    return ROBOT_JOINT_OK;
}

robot_joint_result_t robot_joint_get_state(uint8_t id, robot_joint_state_t *state)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    if (state == 0) {
        return ROBOT_JOINT_INVALID_ARGUMENT;
    }
    state->position_rad = joints[id].position_rad;
    state->velocity_rad_s = joints[id].velocity_rad_s;
    state->target_position_rad = joints[id].target_position_rad;
    return ROBOT_JOINT_OK;
}

robot_joint_result_t robot_joint_stop(uint8_t id)
{
    if (!valid_id(id)) {
        return ROBOT_JOINT_INVALID_ID;
    }
    joints[id].target_position_rad = joints[id].position_rad;
    joints[id].velocity_rad_s = 0.0f;
    return ROBOT_JOINT_OK;
}

void robot_joint_update(float dt_s)
{
    if (dt_s <= 0.0f || dt_s != dt_s) {
        return;
    }
    for (uint8_t id = 0U; id < ROBOT_JOINT_COUNT; id++) {
        float error = joints[id].target_position_rad - joints[id].position_rad;
        float desired_velocity = error / dt_s;
        float velocity_delta;
        float maximum_velocity_delta = joints[id].max_acceleration_rad_s2 * dt_s;

        if (desired_velocity > joints[id].max_velocity_rad_s) {
            desired_velocity = joints[id].max_velocity_rad_s;
        } else if (desired_velocity < -joints[id].max_velocity_rad_s) {
            desired_velocity = -joints[id].max_velocity_rad_s;
        }
        velocity_delta = desired_velocity - joints[id].velocity_rad_s;
        if (velocity_delta > maximum_velocity_delta) {
            velocity_delta = maximum_velocity_delta;
        } else if (velocity_delta < -maximum_velocity_delta) {
            velocity_delta = -maximum_velocity_delta;
        }
        joints[id].velocity_rad_s += velocity_delta;
        joints[id].position_rad += joints[id].velocity_rad_s * dt_s;
        if ((error > 0.0f && joints[id].position_rad > joints[id].target_position_rad)
            || (error < 0.0f && joints[id].position_rad < joints[id].target_position_rad)) {
            joints[id].position_rad = joints[id].target_position_rad;
            joints[id].velocity_rad_s = 0.0f;
        }
        joints[id].encoder_count = (int32_t) ((joints[id].position_rad
            * (float) ROBOT_JOINT_ENCODER_COUNTS_PER_REV) / ROBOT_TWO_PI);
    }
}