#include "cartesian_trajectory.h"

#include <math.h>

#define CARTESIAN_EPSILON 1.0e-6f
#define CARTESIAN_APF_STEP_MARGIN 1.0e-3f
#define CARTESIAN_APF_RETURN_FRACTION 0.20f
#define CARTESIAN_APF_RETURN_STEP_M 0.01f
#define CARTESIAN_PI ROBOT_KINEMATICS_PI

typedef struct {
    float x;
    float y;
    float z;
    float w;
} quaternion_t;

static int finite_array(const float values[ROBOT_CARTESIAN_JOINT_COUNT])
{
    for (uint8_t index = 0U; index < ROBOT_CARTESIAN_JOINT_COUNT; index++) {
        if (!isfinite(values[index])) {
            return 0;
        }
    }
    return 1;
}

static float dot3(const float left[3], const float right[3])
{
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

static void cross3(const float left[3], const float right[3], float result[3])
{
    result[0] = left[1] * right[2] - left[2] * right[1];
    result[1] = left[2] * right[0] - left[0] * right[2];
    result[2] = left[0] * right[1] - left[1] * right[0];
}

static float norm3(const float value[3])
{
    return sqrtf(dot3(value, value));
}

static void normalize3(float value[3])
{
    float length = norm3(value);
    if (length > CARTESIAN_EPSILON) {
        value[0] /= length;
        value[1] /= length;
        value[2] /= length;
    }
}

static quaternion_t quaternion_from_pose(const robot_pose_t *pose)
{
    quaternion_t result;
    float trace = pose->value[0][0] + pose->value[1][1] + pose->value[2][2];
    if (trace > 0.0f) {
        float scale = sqrtf(trace + 1.0f) * 2.0f;
        result.w = 0.25f * scale;
        result.x = (pose->value[2][1] - pose->value[1][2]) / scale;
        result.y = (pose->value[0][2] - pose->value[2][0]) / scale;
        result.z = (pose->value[1][0] - pose->value[0][1]) / scale;
    } else if (pose->value[0][0] > pose->value[1][1]
        && pose->value[0][0] > pose->value[2][2]) {
        float scale = sqrtf(1.0f + pose->value[0][0]
            - pose->value[1][1] - pose->value[2][2]) * 2.0f;
        result.w = (pose->value[2][1] - pose->value[1][2]) / scale;
        result.x = 0.25f * scale;
        result.y = (pose->value[0][1] + pose->value[1][0]) / scale;
        result.z = (pose->value[0][2] + pose->value[2][0]) / scale;
    } else if (pose->value[1][1] > pose->value[2][2]) {
        float scale = sqrtf(1.0f + pose->value[1][1]
            - pose->value[0][0] - pose->value[2][2]) * 2.0f;
        result.w = (pose->value[0][2] - pose->value[2][0]) / scale;
        result.x = (pose->value[0][1] + pose->value[1][0]) / scale;
        result.y = 0.25f * scale;
        result.z = (pose->value[1][2] + pose->value[2][1]) / scale;
    } else {
        float scale = sqrtf(1.0f + pose->value[2][2]
            - pose->value[0][0] - pose->value[1][1]) * 2.0f;
        result.w = (pose->value[1][0] - pose->value[0][1]) / scale;
        result.x = (pose->value[0][2] + pose->value[2][0]) / scale;
        result.y = (pose->value[1][2] + pose->value[2][1]) / scale;
        result.z = 0.25f * scale;
    }
    return result;
}

static robot_pose_t pose_interpolate(
    const robot_pose_t *start, const robot_pose_t *end, float ratio)
{
    robot_pose_t result = *start;
    quaternion_t first = quaternion_from_pose(start);
    quaternion_t second = quaternion_from_pose(end);
    float dot = first.x * second.x + first.y * second.y
        + first.z * second.z + first.w * second.w;
    float weight_first;
    float weight_second;
    float angle;
    if (dot < 0.0f) {
        second.x = -second.x;
        second.y = -second.y;
        second.z = -second.z;
        second.w = -second.w;
        dot = -dot;
    }
    dot = fminf(1.0f, fmaxf(-1.0f, dot));
    if (dot > 0.9995f) {
        weight_first = 1.0f - ratio;
        weight_second = ratio;
    } else {
        angle = acosf(dot);
        weight_first = sinf((1.0f - ratio) * angle) / sinf(angle);
        weight_second = sinf(ratio * angle) / sinf(angle);
    }
    quaternion_t interpolated = {
        weight_first * first.x + weight_second * second.x,
        weight_first * first.y + weight_second * second.y,
        weight_first * first.z + weight_second * second.z,
        weight_first * first.w + weight_second * second.w
    };
    float norm = sqrtf(interpolated.x * interpolated.x
        + interpolated.y * interpolated.y + interpolated.z * interpolated.z
        + interpolated.w * interpolated.w);
    float xx = interpolated.x / norm;
    float yy = interpolated.y / norm;
    float zz = interpolated.z / norm;
    float ww = interpolated.w / norm;
    result.value[0][0] = 1.0f - 2.0f * (yy * yy + zz * zz);
    result.value[0][1] = 2.0f * (xx * yy - zz * ww);
    result.value[0][2] = 2.0f * (xx * zz + yy * ww);
    result.value[1][0] = 2.0f * (xx * yy + zz * ww);
    result.value[1][1] = 1.0f - 2.0f * (xx * xx + zz * zz);
    result.value[1][2] = 2.0f * (yy * zz - xx * ww);
    result.value[2][0] = 2.0f * (xx * zz - yy * ww);
    result.value[2][1] = 2.0f * (yy * zz + xx * ww);
    result.value[2][2] = 1.0f - 2.0f * (xx * xx + yy * yy);
    return result;
}

static robot_pose_t line_pose(const robot_cartesian_trajectory_t *trajectory, float ratio)
{
    robot_pose_t result = pose_interpolate(&trajectory->start_pose,
        &trajectory->end_pose, ratio);
    for (uint8_t index = 0U; index < 3U; index++) {
        result.value[index][3] = trajectory->start_pose.value[index][3]
            + ratio * (trajectory->end_pose.value[index][3]
            - trajectory->start_pose.value[index][3]);
    }
    return result;
}

static robot_pose_t arc_pose(const robot_cartesian_trajectory_t *trajectory, float ratio)
{
    robot_pose_t result = pose_interpolate(&trajectory->start_pose,
        &trajectory->end_pose, ratio);
    float center[3];
    float start_radius[3];
    float axis[3] = {trajectory->center_pose.value[0][2],
        trajectory->center_pose.value[1][2], trajectory->center_pose.value[2][2]};
    float radius;
    float angle = trajectory->arc_angle_rad * ratio;
    for (uint8_t index = 0U; index < 3U; index++) {
        center[index] = trajectory->center_pose.value[index][3];
        start_radius[index] = trajectory->start_pose.value[index][3] - center[index];
    }
    normalize3(axis);
    radius = norm3(start_radius);
    {
        float tangent[3];
        float radial[3];
        float rotated[3];
        float cosine = cosf(angle);
        float sine = sinf(angle);
        cross3(axis, start_radius, tangent);
        for (uint8_t index = 0U; index < 3U; index++) {
            radial[index] = start_radius[index] * cosine;
            rotated[index] = radial[index] + tangent[index] * sine
                + axis[index] * dot3(axis, start_radius) * (1.0f - cosine);
            result.value[index][3] = center[index] + rotated[index];
        }
    }
    (void) radius;
    return result;
}

static robot_cartesian_status_t plan_common(
    robot_cartesian_trajectory_t *trajectory,
    const robot_pose_t *start_pose, const robot_pose_t *end_pose,
    const float start_joint[6], float duration_s, float period_s)
{
    if (trajectory == 0 || start_pose == 0 || end_pose == 0 || start_joint == 0) {
        return ROBOT_CARTESIAN_INVALID_ARGUMENT;
    }
    if (robot_kinematics_pose_validate(start_pose) != ROBOT_KINEMATICS_OK
        || robot_kinematics_pose_validate(end_pose) != ROBOT_KINEMATICS_OK
        || !finite_array(start_joint) || !isfinite(duration_s)
        || !isfinite(period_s) || duration_s <= CARTESIAN_EPSILON
        || period_s <= CARTESIAN_EPSILON
        || !robot_kinematics_joint_limits_ok(start_joint)) {
        return ROBOT_CARTESIAN_INVALID_PATH;
    }
    trajectory->start_pose = *start_pose;
    trajectory->end_pose = *end_pose;
    trajectory->duration_s = duration_s;
    trajectory->period_s = period_s;
    trajectory->elapsed_s = 0.0f;
    trajectory->active = 1U;
    robot_apf_config_init(&trajectory->apf);
    for (uint8_t index = 0U; index < 6U; index++) {
        trajectory->current_joint[index] = start_joint[index];
        trajectory->output_joint[index] = start_joint[index];
    }
    trajectory->previous_nominal_valid = 0U;
    trajectory->previous_deviation_valid = 0U;
    return ROBOT_CARTESIAN_OK;
}

int robot_cartesian_set_obstacles(
    robot_cartesian_trajectory_t *trajectory,
    const robot_apf_obstacle_t *obstacles,
    uint8_t count
)
{
    if (trajectory == 0) {
        return -1;
    }
    return robot_apf_set_obstacles(&trajectory->apf, obstacles, count);
}

robot_cartesian_status_t robot_cartesian_plan_line(
    robot_cartesian_trajectory_t *trajectory, const robot_pose_t *start_pose,
    const robot_pose_t *end_pose, const float start_joint[6], float duration_s,
    float period_s)
{
    robot_cartesian_status_t status = plan_common(trajectory, start_pose, end_pose,
        start_joint, duration_s, period_s);
    if (status != ROBOT_CARTESIAN_OK) {
        return status;
    }
    trajectory->type = ROBOT_CARTESIAN_LINE;
    return ROBOT_CARTESIAN_OK;
}

robot_cartesian_status_t robot_cartesian_plan_arc(
    robot_cartesian_trajectory_t *trajectory, const robot_pose_t *start_pose,
    const robot_pose_t *end_pose, const robot_pose_t *center_pose, uint8_t direction,
    const float start_joint[6], float duration_s, float period_s)
{
    float start_radius[3];
    float end_radius[3];
    float axis[3];
    float cross[3];
    float radius_start;
    float radius_end;
    float angle;
    robot_cartesian_status_t status;
    if (center_pose == 0 || (direction != 0U && direction != 1U)) {
        return ROBOT_CARTESIAN_INVALID_ARGUMENT;
    }
    status = plan_common(trajectory, start_pose, end_pose, start_joint,
        duration_s, period_s);
    if (status != ROBOT_CARTESIAN_OK
        || robot_kinematics_pose_validate(center_pose) != ROBOT_KINEMATICS_OK) {
        return ROBOT_CARTESIAN_INVALID_PATH;
    }
    for (uint8_t index = 0U; index < 3U; index++) {
        start_radius[index] = start_pose->value[index][3] - center_pose->value[index][3];
        end_radius[index] = end_pose->value[index][3] - center_pose->value[index][3];
        axis[index] = center_pose->value[index][2];
    }
    radius_start = norm3(start_radius);
    radius_end = norm3(end_radius);
    if (radius_start <= CARTESIAN_EPSILON
        || fabsf(radius_start - radius_end) > 1.0e-4f) {
        return ROBOT_CARTESIAN_INVALID_PATH;
    }
    normalize3(axis);
    cross3(start_radius, end_radius, cross);
    angle = atan2f(norm3(cross), dot3(start_radius, end_radius));
    if (dot3(axis, cross) < 0.0f) {
        angle = -angle;
    }
    if (direction == 0U && angle > 0.0f) {
        angle -= 2.0f * CARTESIAN_PI;
    } else if (direction == 1U && angle < 0.0f) {
        angle += 2.0f * CARTESIAN_PI;
    }
    if (fabsf(angle) <= CARTESIAN_EPSILON) {
        return ROBOT_CARTESIAN_INVALID_PATH;
    }
    trajectory->center_pose = *center_pose;
    trajectory->arc_angle_rad = angle;
    trajectory->direction = direction;
    trajectory->type = ROBOT_CARTESIAN_ARC;
    return ROBOT_CARTESIAN_OK;
}

robot_cartesian_status_t robot_cartesian_update(
    robot_cartesian_trajectory_t *trajectory, float dt_s, float output_joint[6])
{
    robot_pose_t target;
    robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS];
    robot_kinematics_solution_score_t score;
    uint8_t count = 0U;
    uint8_t best_index = 0U;
    float ratio;
    robot_kinematics_status_t status;
    robot_pose_t current_pose;
    float current_position[3];
    float nominal_position[3];
    float adjusted_position[3];
    float apf_step_limit = 0.0f;
    if (trajectory == 0 || output_joint == 0 || !isfinite(dt_s)
        || dt_s <= 0.0f) {
        return ROBOT_CARTESIAN_INVALID_ARGUMENT;
    }
    if (trajectory->active == 0U) {
        for (uint8_t index = 0U; index < 6U; index++) {
            output_joint[index] = trajectory->output_joint[index];
        }
        return ROBOT_CARTESIAN_COMPLETE;
    }
    trajectory->elapsed_s = fminf(trajectory->duration_s,
        trajectory->elapsed_s + dt_s);
    ratio = trajectory->elapsed_s / trajectory->duration_s;
    target = trajectory->type == ROBOT_CARTESIAN_LINE
        ? line_pose(trajectory, ratio) : arc_pose(trajectory, ratio);
    if (robot_kinematics_fk(trajectory->current_joint, &current_pose)
        != ROBOT_KINEMATICS_OK) {
        trajectory->active = 0U;
        return ROBOT_CARTESIAN_IK_FAILED;
    }
    for (uint8_t index = 0U; index < 3U; index++) {
        current_position[index] = current_pose.value[index][3];
        nominal_position[index] = target.value[index][3];
    }
    {
        float advance = 0.0f;
        if (trajectory->previous_nominal_valid != 0U) {
            for (uint8_t index = 0U; index < 3U; index++) {
                float delta = nominal_position[index]
                    - trajectory->previous_nominal[index];
                advance = sqrtf(advance * advance + delta * delta);
            }
        }
        for (uint8_t index = 0U; index < 3U; index++) {
            trajectory->previous_nominal[index] = nominal_position[index];
        }
        trajectory->previous_nominal_valid = 1U;
        apf_step_limit = advance + CARTESIAN_APF_STEP_MARGIN;
    }
    if (robot_apf_adjust_target(&trajectory->apf,
        current_position, nominal_position, adjusted_position) != 0) {
        trajectory->active = 0U;
        return ROBOT_CARTESIAN_IK_FAILED;
    }
    {
        float deviation[3];
        float change[3];
        float change_norm = 0.0f;
        for (uint8_t index = 0U; index < 3U; index++) {
            float base = trajectory->previous_deviation_valid != 0U
                ? trajectory->previous_deviation[index] : 0.0f;
            deviation[index] = adjusted_position[index]
                - nominal_position[index];
            change[index] = deviation[index] - base;
            change_norm = sqrtf(change_norm * change_norm
                + change[index] * change[index]);
        }

        {
            float previous_norm = 0.0f;
            float deviation_norm = 0.0f;
            float limit = apf_step_limit;

            for (uint8_t index = 0U; index < 3U; index++) {
                float base = trajectory->previous_deviation_valid != 0U
                    ? trajectory->previous_deviation[index] : 0.0f;
                previous_norm = sqrtf(previous_norm * previous_norm
                    + base * base);
                deviation_norm = sqrtf(deviation_norm * deviation_norm
                    + deviation[index] * deviation[index]);
            }
            if (deviation_norm < previous_norm
                && apf_step_limit < CARTESIAN_APF_RETURN_STEP_M) {
                limit = CARTESIAN_APF_RETURN_STEP_M;
            }
            if (change_norm > limit && change_norm > 0.0f) {
                for (uint8_t index = 0U; index < 3U; index++) {
                    float base = trajectory->previous_deviation_valid != 0U
                        ? trajectory->previous_deviation[index] : 0.0f;
                    deviation[index] = base
                        + change[index] * limit / change_norm;
                }
            }
        }

        if (ratio > 1.0f - CARTESIAN_APF_RETURN_FRACTION) {
            float fade = (1.0f - ratio) / CARTESIAN_APF_RETURN_FRACTION;
            for (uint8_t index = 0U; index < 3U; index++) {
                deviation[index] *= fade;
            }
        }
        for (uint8_t index = 0U; index < 3U; index++) {
            trajectory->previous_deviation[index] = deviation[index];
            target.value[index][3] = nominal_position[index] + deviation[index];
        }
        trajectory->previous_deviation_valid = 1U;
    }
    status = robot_kinematics_ik(&target, trajectory->current_joint,
        solutions, &count);
    if (status != ROBOT_KINEMATICS_OK || count == 0U) {
        trajectory->active = 0U;
        return ROBOT_CARTESIAN_IK_FAILED;
    }
    status = robot_kinematics_select_best(solutions, count,
        trajectory->current_joint, &best_index, &score);
    if (status != ROBOT_KINEMATICS_OK) {
        trajectory->active = 0U;
        return ROBOT_CARTESIAN_IK_FAILED;
    }
    for (uint8_t index = 0U; index < 6U; index++) {
        trajectory->output_joint[index] = solutions[best_index].joint[index];
        trajectory->current_joint[index] = trajectory->output_joint[index];
        output_joint[index] = trajectory->output_joint[index];
    }
    if (trajectory->elapsed_s >= trajectory->duration_s - CARTESIAN_EPSILON) {
        trajectory->active = 0U;
        return ROBOT_CARTESIAN_COMPLETE;
    }
    return ROBOT_CARTESIAN_OK;
}

void robot_cartesian_stop(robot_cartesian_trajectory_t *trajectory)
{
    if (trajectory != 0) {
        trajectory->active = 0U;
    }
}