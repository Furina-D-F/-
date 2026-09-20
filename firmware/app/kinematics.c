#include "kinematics.h"

#include <float.h>
#include <math.h>

#define UR5_D1 0.089159f
#define UR5_A2 (-0.425f)
#define UR5_A3 (-0.39225f)
#define UR5_D4 0.10915f
#define UR5_D5 0.09465f
#define UR5_D6 0.0823f
#define KINEMATICS_EPSILON 1.0e-5f
#define KINEMATICS_POSITION_TOLERANCE 2.0e-4f
#define KINEMATICS_ROTATION_TOLERANCE 2.0e-3f
#define KINEMATICS_ORTHOGONAL_TOLERANCE 3.0e-3f
#define KINEMATICS_WRIST_SINGULAR_THRESHOLD 1.0e-3f
#define KINEMATICS_ELBOW_SINGULAR_THRESHOLD 1.0e-3f
#define KINEMATICS_LIMIT_MARGIN 0.15f
#define KINEMATICS_TRAVEL_WEIGHT 1.0f
#define KINEMATICS_LIMIT_WEIGHT 4.0f
#define KINEMATICS_SINGULARITY_WEIGHT 8.0f

static const float joint_min[ROBOT_KINEMATICS_JOINT_COUNT] = {
    -ROBOT_KINEMATICS_TWO_PI, -ROBOT_KINEMATICS_TWO_PI, -ROBOT_KINEMATICS_PI,
    -ROBOT_KINEMATICS_TWO_PI, -ROBOT_KINEMATICS_TWO_PI, -ROBOT_KINEMATICS_TWO_PI
};

static const float joint_max[ROBOT_KINEMATICS_JOINT_COUNT] = {
    ROBOT_KINEMATICS_TWO_PI, ROBOT_KINEMATICS_TWO_PI, ROBOT_KINEMATICS_PI,
    ROBOT_KINEMATICS_TWO_PI, ROBOT_KINEMATICS_TWO_PI, ROBOT_KINEMATICS_TWO_PI
};

static void matrix_identity(float matrix[4][4])
{
    for (uint8_t row = 0U; row < 4U; row++) {
        for (uint8_t column = 0U; column < 4U; column++) {
            matrix[row][column] = row == column ? 1.0f : 0.0f;
        }
    }
}

static void matrix_copy(float destination[4][4], const float source[4][4])
{
    for (uint8_t row = 0U; row < 4U; row++) {
        for (uint8_t column = 0U; column < 4U; column++) {
            destination[row][column] = source[row][column];
        }
    }
}

static void matrix_multiply(
    float result[4][4],
    const float left[4][4],
    const float right[4][4]
)
{
    float product[4][4];

    for (uint8_t row = 0U; row < 4U; row++) {
        for (uint8_t column = 0U; column < 4U; column++) {
            product[row][column] = 0.0f;
            for (uint8_t index = 0U; index < 4U; index++) {
                product[row][column] += left[row][index] * right[index][column];
            }
        }
    }
    matrix_copy(result, product);
}

static void dh_matrix(float a, float d, float alpha, float theta, float matrix[4][4])
{
    float cosine_theta = cosf(theta);
    float sine_theta = sinf(theta);
    float cosine_alpha = cosf(alpha);
    float sine_alpha = sinf(alpha);

    matrix[0][0] = cosine_theta;
    matrix[0][1] = -sine_theta * cosine_alpha;
    matrix[0][2] = sine_theta * sine_alpha;
    matrix[0][3] = a * cosine_theta;
    matrix[1][0] = sine_theta;
    matrix[1][1] = cosine_theta * cosine_alpha;
    matrix[1][2] = -cosine_theta * sine_alpha;
    matrix[1][3] = a * sine_theta;
    matrix[2][0] = 0.0f;
    matrix[2][1] = sine_alpha;
    matrix[2][2] = cosine_alpha;
    matrix[2][3] = d;
    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = 0.0f;
    matrix[3][3] = 1.0f;
}

static void chain_to_index(const float joint[6], uint8_t end, float result[4][4])
{
    static const float a[6] = {0.0f, UR5_A2, UR5_A3, 0.0f, 0.0f, 0.0f};
    static const float d[6] = {UR5_D1, 0.0f, 0.0f, UR5_D4, UR5_D5, UR5_D6};
    static const float alpha[6] = {
        ROBOT_KINEMATICS_PI / 2.0f, 0.0f, 0.0f,
        ROBOT_KINEMATICS_PI / 2.0f, -ROBOT_KINEMATICS_PI / 2.0f, 0.0f
    };
    float step[4][4];
    float accumulated[4][4];

    matrix_identity(accumulated);
    for (uint8_t index = 0U; index <= end; index++) {
        dh_matrix(a[index], d[index], alpha[index], joint[index], step);
        matrix_multiply(accumulated, accumulated, step);
    }
    matrix_copy(result, accumulated);
}

static void homogeneous_inverse(const float input[4][4], float result[4][4])
{
    matrix_identity(result);
    for (uint8_t row = 0U; row < 3U; row++) {
        for (uint8_t column = 0U; column < 3U; column++) {
            result[row][column] = input[column][row];
        }
    }
    for (uint8_t row = 0U; row < 3U; row++) {
        result[row][3] = 0.0f;
        for (uint8_t index = 0U; index < 3U; index++) {
            result[row][3] -= result[row][index] * input[index][3];
        }
    }
}

static float clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static float normalize_near(float angle, float reference)
{
    while (angle - reference > ROBOT_KINEMATICS_PI) {
        angle -= ROBOT_KINEMATICS_TWO_PI;
    }
    while (angle - reference < -ROBOT_KINEMATICS_PI) {
        angle += ROBOT_KINEMATICS_TWO_PI;
    }
    return angle;
}

static int finite_joint(const float joint[6])
{
    for (uint8_t index = 0U; index < 6U; index++) {
        if (!isfinite(joint[index])) {
            return 0;
        }
    }
    return 1;
}

robot_kinematics_status_t robot_kinematics_pose_validate(const robot_pose_t *pose)
{
    float determinant;

    if (pose == 0) {
        return ROBOT_KINEMATICS_INVALID_ARGUMENT;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        for (uint8_t column = 0U; column < 4U; column++) {
            if (!isfinite(pose->value[row][column])) {
                return ROBOT_KINEMATICS_INVALID_POSE;
            }
        }
    }
    if (fabsf(pose->value[3][0]) > KINEMATICS_EPSILON
        || fabsf(pose->value[3][1]) > KINEMATICS_EPSILON
        || fabsf(pose->value[3][2]) > KINEMATICS_EPSILON
        || fabsf(pose->value[3][3] - 1.0f) > KINEMATICS_EPSILON) {
        return ROBOT_KINEMATICS_INVALID_POSE;
    }
    for (uint8_t row = 0U; row < 3U; row++) {
        for (uint8_t column = row; column < 3U; column++) {
            float dot = 0.0f;
            for (uint8_t index = 0U; index < 3U; index++) {
                dot += pose->value[index][row] * pose->value[index][column];
            }
            if (fabsf(dot - (row == column ? 1.0f : 0.0f))
                > KINEMATICS_ORTHOGONAL_TOLERANCE) {
                return ROBOT_KINEMATICS_INVALID_POSE;
            }
        }
    }
    determinant = pose->value[0][0] * (pose->value[1][1] * pose->value[2][2]
        - pose->value[1][2] * pose->value[2][1])
        - pose->value[0][1] * (pose->value[1][0] * pose->value[2][2]
        - pose->value[1][2] * pose->value[2][0])
        + pose->value[0][2] * (pose->value[1][0] * pose->value[2][1]
        - pose->value[1][1] * pose->value[2][0]);
    if (fabsf(determinant - 1.0f) > KINEMATICS_ORTHOGONAL_TOLERANCE) {
        return ROBOT_KINEMATICS_INVALID_POSE;
    }
    return ROBOT_KINEMATICS_OK;
}

int robot_kinematics_joint_limits_ok(const float joint[6])
{
    if (joint == 0 || !finite_joint(joint)) {
        return 0;
    }
    for (uint8_t index = 0U; index < 6U; index++) {
        if (joint[index] < joint_min[index] - KINEMATICS_EPSILON
            || joint[index] > joint_max[index] + KINEMATICS_EPSILON) {
            return 0;
        }
    }
    return 1;
}

robot_kinematics_status_t robot_kinematics_fk(const float joint[6], robot_pose_t *pose)
{
    float result[4][4];

    if (pose == 0 || !robot_kinematics_joint_limits_ok(joint)) {
        return ROBOT_KINEMATICS_INVALID_ARGUMENT;
    }
    chain_to_index(joint, 5U, result);
    matrix_copy(pose->value, result);
    return ROBOT_KINEMATICS_OK;
}

static float position_error(const float left[4][4], const float right[4][4])
{
    float sum = 0.0f;
    for (uint8_t index = 0U; index < 3U; index++) {
        float difference = left[index][3] - right[index][3];
        sum += difference * difference;
    }
    return sqrtf(sum);
}

static float rotation_error(const float left[4][4], const float right[4][4])
{
    float trace = 0.0f;
    float relative[3][3];

    for (uint8_t row = 0U; row < 3U; row++) {
        for (uint8_t column = 0U; column < 3U; column++) {
            relative[row][column] = 0.0f;
            for (uint8_t index = 0U; index < 3U; index++) {
                relative[row][column] += left[index][row] * right[index][column];
            }
        }
        trace += relative[row][row];
    }
    return acosf(clamp_unit((trace - 1.0f) * 0.5f));
}

static int duplicate_solution(
    const robot_kinematics_solution_t solutions[8],
    uint8_t count,
    const float joint[6]
)
{
    for (uint8_t item = 0U; item < count; item++) {
        float distance = 0.0f;
        for (uint8_t index = 0U; index < 6U; index++) {
            float difference = solutions[item].joint[index] - joint[index];
            distance += difference * difference;
        }
        if (distance < 1.0e-6f) {
            return 1;
        }
    }
    return 0;
}

robot_kinematics_status_t robot_kinematics_ik(
    const robot_pose_t *pose,
    const float current_joint[6],
    robot_kinematics_solution_t solutions[8],
    uint8_t *count
)
{
    float q1_base;
    float radial;
    float c5;
    uint8_t solution_count = 0U;
    float target[4][4];

    if (pose == 0 || solutions == 0 || count == 0) {
        return ROBOT_KINEMATICS_INVALID_ARGUMENT;
    }
    *count = 0U;
    if (robot_kinematics_pose_validate(pose) != ROBOT_KINEMATICS_OK) {
        return ROBOT_KINEMATICS_INVALID_POSE;
    }
    matrix_copy(target, pose->value);
    radial = hypotf(UR5_D6 * target[0][2] - target[0][3],
        UR5_D6 * target[1][2] - target[1][3]);
    if (!isfinite(radial) || radial < fabsf(UR5_D4) - KINEMATICS_EPSILON) {
        return ROBOT_KINEMATICS_NO_SOLUTION;
    }

    for (uint8_t shoulder = 0U; shoulder < 4U; shoulder++) {
        float shoulder_sign = (shoulder & 1U) == 0U ? 1.0f : -1.0f;
        float shoulder_offset = shoulder >= 2U ? ROBOT_KINEMATICS_PI : 0.0f;
        q1_base = atan2f(
            UR5_D6 * target[1][2] - target[1][3],
            UR5_D6 * target[0][2] - target[0][3]
        ) + shoulder_offset + shoulder_sign * asinf(clamp_unit(UR5_D4 / radial));
        if (shoulder >= 2U) {
            q1_base = normalize_near(
                q1_base, current_joint != 0 ? current_joint[0] : q1_base);
        }
        c5 = (target[0][3] * sinf(q1_base)
            - target[1][3] * cosf(q1_base) - UR5_D4) / UR5_D6;
        if (c5 < -1.0f - KINEMATICS_EPSILON || c5 > 1.0f + KINEMATICS_EPSILON) {
            continue;
        }
        c5 = clamp_unit(c5);

        for (uint8_t wrist = 0U; wrist < 2U; wrist++) {
            float q[6];
            float q5 = (wrist == 0U ? 1.0f : -1.0f) * acosf(c5);
            float q6 = atan2f(
                -target[0][1] * sinf(q1_base) + target[1][1] * cosf(q1_base),
                target[0][0] * sinf(q1_base) - target[1][0] * cosf(q1_base)
            );
            float a6[4][4];
            float a5[4][4];
            float inverse_a6[4][4];
            float inverse_a5[4][4];
            float t04[4][4];
            float planar_length;
            float planar_height;
            float cosine_q3;

            if (wrist != 0U) {
                q6 += ROBOT_KINEMATICS_PI;
            }
            q[0] = q1_base;
            q[4] = q5;
            q[5] = normalize_near(q6, current_joint != 0 ? current_joint[5] : q6);
            dh_matrix(0.0f, UR5_D6, 0.0f, q[5], a6);
            dh_matrix(0.0f, UR5_D5, -ROBOT_KINEMATICS_PI / 2.0f, q[4], a5);
            homogeneous_inverse(a6, inverse_a6);
            homogeneous_inverse(a5, inverse_a5);
            matrix_multiply(t04, target, inverse_a6);
            matrix_multiply(t04, t04, inverse_a5);

            planar_length = cosf(q[0]) * t04[0][3] + sinf(q[0]) * t04[1][3];
            planar_height = t04[2][3] - UR5_D1;
            cosine_q3 = (planar_length * planar_length + planar_height * planar_height
                - UR5_A2 * UR5_A2 - UR5_A3 * UR5_A3)
                / (2.0f * UR5_A2 * UR5_A3);
            if (cosine_q3 < -1.0f - KINEMATICS_EPSILON
                || cosine_q3 > 1.0f + KINEMATICS_EPSILON) {
                continue;
            }
            cosine_q3 = clamp_unit(cosine_q3);

            for (uint8_t elbow = 0U; elbow < 2U; elbow++) {
                float sine_q3 = (elbow == 0U ? 1.0f : -1.0f)
                    * sqrtf(fmaxf(0.0f, 1.0f - cosine_q3 * cosine_q3));
                float q3 = atan2f(sine_q3, cosine_q3);
                float q2 = atan2f(planar_height, planar_length)
                    - atan2f(UR5_A3 * sine_q3, UR5_A2 + UR5_A3 * cosine_q3);
                float r03[4][4];
                float inverse_r03[4][4];
                float relative[4][4];
                float q4;

                q[1] = normalize_near(q2, current_joint != 0 ? current_joint[1] : q2);
                q[2] = normalize_near(q3, current_joint != 0 ? current_joint[2] : q3);
                chain_to_index(q, 2U, r03);
                homogeneous_inverse(r03, inverse_r03);
                matrix_multiply(relative, inverse_r03, t04);
                q4 = atan2f(relative[1][0], relative[0][0]);
                q[3] = normalize_near(q4, current_joint != 0 ? current_joint[3] : q4);
                if (!robot_kinematics_joint_limits_ok(q)) {
                    continue;
                }
                {
                    robot_pose_t candidate_pose;
                    if (robot_kinematics_fk(q, &candidate_pose) != ROBOT_KINEMATICS_OK
                        || position_error(candidate_pose.value, target)
                            > KINEMATICS_POSITION_TOLERANCE
                        || rotation_error(candidate_pose.value, target)
                            > KINEMATICS_ROTATION_TOLERANCE
                        || duplicate_solution(solutions, solution_count, q)) {
                        continue;
                    }
                }
                if (solution_count >= ROBOT_KINEMATICS_MAX_SOLUTIONS) {
                    continue;
                }
                for (uint8_t index = 0U; index < 6U; index++) {
                    solutions[solution_count].joint[index] = q[index];
                }
                solution_count++;
            }
        }
    }

    *count = solution_count;
    if (solution_count == 0U) {
        return ROBOT_KINEMATICS_NO_SOLUTION;
    }
    return ROBOT_KINEMATICS_OK;
}

static float joint_distance(float left, float right)
{
    return fabsf(normalize_near(left, right) - right);
}

static float limit_penalty(float angle, uint8_t index)
{
    float range = joint_max[index] - joint_min[index];
    float distance = fminf(angle - joint_min[index], joint_max[index] - angle);
    float normalized = distance / range;

    if (normalized >= KINEMATICS_LIMIT_MARGIN) {
        return 0.0f;
    }
    normalized = fmaxf(0.0f, normalized / KINEMATICS_LIMIT_MARGIN);
    return (1.0f - normalized) * (1.0f - normalized);
}

static float solution_singularity_penalty(
    const float joint[ROBOT_KINEMATICS_JOINT_COUNT],
    uint8_t *singular
)
{
    float wrist_margin = fabsf(sinf(joint[4]));
    float elbow_margin = fabsf(sinf(joint[2]));
    float wrist_penalty = 1.0f / fmaxf(wrist_margin, KINEMATICS_WRIST_SINGULAR_THRESHOLD);
    float elbow_penalty = 1.0f
        / fmaxf(elbow_margin, KINEMATICS_ELBOW_SINGULAR_THRESHOLD);

    *singular = wrist_margin < KINEMATICS_WRIST_SINGULAR_THRESHOLD
        || elbow_margin < KINEMATICS_ELBOW_SINGULAR_THRESHOLD;
    return 0.5f * (wrist_penalty + elbow_penalty);
}

robot_kinematics_status_t robot_kinematics_select_best(
    const robot_kinematics_solution_t solutions[ROBOT_KINEMATICS_MAX_SOLUTIONS],
    uint8_t count,
    const float current_joint[ROBOT_KINEMATICS_JOINT_COUNT],
    uint8_t *best_index,
    robot_kinematics_solution_score_t *score
)
{
    float best_cost = FLT_MAX;
    uint8_t selected = 0U;
    robot_kinematics_solution_score_t selected_score = {0};

    if (solutions == 0 || current_joint == 0 || best_index == 0
        || score == 0 || count == 0U || count > ROBOT_KINEMATICS_MAX_SOLUTIONS
        || !finite_joint(current_joint)) {
        return ROBOT_KINEMATICS_INVALID_ARGUMENT;
    }
    for (uint8_t item = 0U; item < count; item++) {
        robot_kinematics_solution_score_t candidate = {0};
        for (uint8_t index = 0U; index < ROBOT_KINEMATICS_JOINT_COUNT; index++) {
            float travel = joint_distance(solutions[item].joint[index], current_joint[index]);
            candidate.travel_cost += travel * travel;
            candidate.limit_cost += limit_penalty(solutions[item].joint[index], index);
        }
        candidate.singularity_cost = solution_singularity_penalty(
            solutions[item].joint, &candidate.singular);
        candidate.total_cost = KINEMATICS_TRAVEL_WEIGHT * candidate.travel_cost
            + KINEMATICS_LIMIT_WEIGHT * candidate.limit_cost
            + KINEMATICS_SINGULARITY_WEIGHT * candidate.singularity_cost;
        if (candidate.singular != 0U) {
            continue;
        }
        if (candidate.total_cost < best_cost) {
            best_cost = candidate.total_cost;
            selected = item;
            selected_score = candidate;
        }
    }
    if (best_cost == FLT_MAX) {
        return ROBOT_KINEMATICS_SINGULAR;
    }
    *best_index = selected;
    *score = selected_score;
    return ROBOT_KINEMATICS_OK;
}
