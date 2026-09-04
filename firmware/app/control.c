#include "control.h"

#include "joint_motor.h"

#ifdef ROBOT_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#define ROBOT_MOTION_MODE_POSITION 0U
#define ROBOT_MOTION_MODE_STOP 1U

static const float joint_min[ROBOT_CONTROL_JOINT_COUNT] = {
    -6.283185307f, -6.283185307f, -3.141592654f,
    -6.283185307f, -6.283185307f, -6.283185307f
};
static const float joint_max[ROBOT_CONTROL_JOINT_COUNT] = {
    6.283185307f, 6.283185307f, 3.141592654f,
    6.283185307f, 6.283185307f, 6.283185307f
};
static robot_control_status_t control_status;
#ifdef ROBOT_FREERTOS
static SemaphoreHandle_t control_mutex;
#endif

#ifdef ROBOT_FREERTOS
static void control_lock(void)
{
    configASSERT(control_mutex != NULL);
    (void) xSemaphoreTake(control_mutex, portMAX_DELAY);
}

static void control_unlock(void)
{
    (void) xSemaphoreGive(control_mutex);
}
#else
static void control_lock(void)
{
}

static void control_unlock(void)
{
}
#endif

static robot_app_result_t robot_control_stop_internal(void)
{
    if (control_status.state == ROBOT_CONTROL_ERROR) {
        control_status.error_code = ROBOT_APP_INVALID_STATE;
        return ROBOT_APP_INVALID_STATE;
    }
    for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        (void) robot_joint_stop(index);
    }
    control_status.state = ROBOT_CONTROL_STOPPED;
    control_status.error_code = ROBOT_APP_OK;
    return ROBOT_APP_OK;
}

void robot_control_init(void)
{
#ifdef ROBOT_FREERTOS
    if (control_mutex == NULL) {
        control_mutex = xSemaphoreCreateMutex();
        configASSERT(control_mutex != NULL);
    }
#endif
    control_status.state = ROBOT_CONTROL_IDLE;
    control_status.error_code = ROBOT_APP_OK;
    for (uint32_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        robot_joint_init((uint8_t) index);
        control_status.target_position_rad[index] = 0.0f;
        control_status.position_rad[index] = 0.0f;
        control_status.velocity_rad_s[index] = 0.0f;
    }
}

robot_app_result_t robot_control_handle_motion(const motion_command_t *command)
{
    robot_app_result_t result;

    control_lock();
    if (command == 0 || command->joint_mask == 0U) {
        control_status.error_code = ROBOT_APP_INVALID_ARGUMENT;
        control_unlock();
        return ROBOT_APP_INVALID_ARGUMENT;
    }

    if (command->mode == ROBOT_MOTION_MODE_STOP) {
        result = robot_control_stop_internal();
        control_unlock();
        return result;
    }
    if (command->mode != ROBOT_MOTION_MODE_POSITION
        || (control_status.state != ROBOT_CONTROL_IDLE
            && control_status.state != ROBOT_CONTROL_RUNNING)) {
        control_status.error_code = ROBOT_APP_INVALID_STATE;
        control_unlock();
        return ROBOT_APP_INVALID_STATE;
    }
    if (command->max_velocity_rad_s <= 0.0f
        || command->max_acceleration_rad_s2 <= 0.0f) {
        control_status.error_code = ROBOT_APP_INVALID_ARGUMENT;
        control_unlock();
        return ROBOT_APP_INVALID_ARGUMENT;
    }

    for (uint32_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        if ((command->joint_mask & (uint8_t) (1U << index)) != 0U
            && (command->target_position_rad[index] < joint_min[index]
                || command->target_position_rad[index] > joint_max[index])) {
            control_status.error_code = ROBOT_APP_LIMIT;
            control_unlock();
            return ROBOT_APP_LIMIT;
        }
    }

    for (uint32_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        if ((command->joint_mask & (uint8_t) (1U << index)) != 0U) {
            if (robot_joint_set_target((uint8_t) index,
                command->target_position_rad[index], command->max_velocity_rad_s,
                command->max_acceleration_rad_s2)
                != ROBOT_JOINT_OK) {
                control_status.error_code = ROBOT_APP_INVALID_ARGUMENT;
                control_unlock();
                return ROBOT_APP_INVALID_ARGUMENT;
            }
            control_status.target_position_rad[index] = command->target_position_rad[index];
        }
    }
    control_status.state = ROBOT_CONTROL_RUNNING;
    control_status.error_code = ROBOT_APP_OK;
    control_unlock();
    return ROBOT_APP_OK;
}

robot_app_result_t robot_control_stop(void)
{
    robot_app_result_t result;

    control_lock();
    result = robot_control_stop_internal();
    control_unlock();
    return result;
}

void robot_control_update(float dt_s)
{
    robot_joint_state_t joint_state;

    control_lock();
    robot_joint_update(dt_s);
    for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        if (robot_joint_get_state(index, &joint_state) == ROBOT_JOINT_OK) {
            control_status.position_rad[index] = joint_state.position_rad;
            control_status.velocity_rad_s[index] = joint_state.velocity_rad_s;
        }
    }
    control_unlock();
}

void robot_control_get_status(robot_control_status_t *status)
{
    if (status == 0) {
        return;
    }
    control_lock();
    status->state = control_status.state;
    status->error_code = control_status.error_code;
    for (uint32_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        status->target_position_rad[index] = control_status.target_position_rad[index];
        status->position_rad[index] = control_status.position_rad[index];
        status->velocity_rad_s[index] = control_status.velocity_rad_s[index];
    }
    control_unlock();
}