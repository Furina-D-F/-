#include "robot_tasks.h"

#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "joint_motor.h"
#include "trajectory.h"
#include "joint_pid.h"

#define ROBOT_PATH_TASK_PRIORITY 3U
#define ROBOT_PID_TASK_PRIORITY 2U
#define ROBOT_STATUS_TASK_PRIORITY 1U
#define ROBOT_PATH_TASK_STACK 384U
#define ROBOT_PID_TASK_STACK 384U
#define ROBOT_STATUS_TASK_STACK 256U
#define ROBOT_PATH_QUEUE_LENGTH 4U
#define ROBOT_STATUS_QUEUE_LENGTH 1U
#define ROBOT_CONTROL_PERIOD_TICKS pdMS_TO_TICKS(10U)
#define ROBOT_TRAJECTORY_NONE 0U
#define ROBOT_TRAJECTORY_JOINT 1U
#define ROBOT_TRAJECTORY_CARTESIAN 2U

static QueueHandle_t path_queue;
static QueueHandle_t status_queue;
static SemaphoreHandle_t health_mutex;
static SemaphoreHandle_t trajectory_mutex;
static TaskHandle_t pid_task_handle;
static robot_tasks_health_t health;
static robot_trajectory_trapezoid_t active_trajectory;
static robot_cartesian_trajectory_t active_cartesian;
static robot_joint_pid_t joint_pid[ROBOT_CONTROL_JOINT_COUNT];
static float trajectory_time_s;
static int trajectory_active;
static uint8_t trajectory_kind;

static const robot_apf_obstacle_t industrial_obstacles[] = {
    {
        .minimum = {-0.64f, -0.46f, 0.30f},
        .maximum = {-0.58f, -0.40f, 0.60f},
        .clearance_m = 0.04f,
        .influence_radius = 0.20f,
        .repulsive_gain = 0.02f
    }
};

static const robot_pid_config_t pid_config = {
    .kp = 2.5f,
    .ki = 0.3f,
    .kd = 0.025f,
    .integral_limit = 1.0f,
    .output_limit = 2.0f,
    .deadband = 0.0f,
    .sample_time_s = 0.01f
};

static void increment_health(uint32_t *counter)
{
    if (health_mutex != NULL) {
        (void) xSemaphoreTake(health_mutex, portMAX_DELAY);
        (*counter)++;
        (void) xSemaphoreGive(health_mutex);
    }
}

static void path_planning_task(void *argument)
{
    robot_path_command_t command;

    (void) argument;
    for (;;) {
        if (xQueueReceive(path_queue, &command, portMAX_DELAY) == pdPASS) {
            robot_control_status_t status;
            robot_app_result_t control_result = ROBOT_APP_OK;

            robot_control_get_status(&status);
            (void) xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
            if (command.type == ROBOT_PATH_MOTION) {
                float max_velocity[ROBOT_CONTROL_JOINT_COUNT];
                float max_acceleration[ROBOT_CONTROL_JOINT_COUNT];

                for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                    if ((command.motion.joint_mask & (uint8_t) (1U << index)) == 0U) {
                        command.motion.target_position_rad[index] = status.position_rad[index];
                    }
                    max_velocity[index] = command.motion.max_velocity_rad_s;
                    max_acceleration[index] = command.motion.max_acceleration_rad_s2;
                }
                if (command.motion.mode == 1U) {
                    control_result = robot_control_stop();
                    trajectory_active = 0;
                    trajectory_kind = ROBOT_TRAJECTORY_NONE;
                    robot_cartesian_stop(&active_cartesian);
                } else if (robot_trajectory_plan_trapezoid(&active_trajectory,
                    status.position_rad, command.motion.target_position_rad,
                    max_velocity, max_acceleration) == ROBOT_TRAJECTORY_OK) {
                    control_result = robot_control_handle_motion(&command.motion);
                    if (control_result == ROBOT_APP_OK) {
                        trajectory_time_s = 0.0f;
                        trajectory_active = 1;
                        trajectory_kind = ROBOT_TRAJECTORY_JOINT;
                    }
                } else {
                    control_result = ROBOT_APP_INVALID_ARGUMENT;
                }
            } else {
                robot_cartesian_status_t cartesian_result;
                if (command.type == ROBOT_PATH_CARTESIAN_LINE) {
                    cartesian_result = robot_cartesian_plan_line(&active_cartesian,
                        &command.start_pose, &command.end_pose, status.position_rad,
                        command.duration_s, command.period_s);
                } else {
                    cartesian_result = robot_cartesian_plan_arc(&active_cartesian,
                        &command.start_pose, &command.end_pose, &command.center_pose,
                        command.direction, status.position_rad, command.duration_s,
                        command.period_s);
                }
                if (cartesian_result == ROBOT_CARTESIAN_OK) {
                    (void) robot_cartesian_set_obstacles(&active_cartesian,
                        industrial_obstacles,
                        (uint8_t) (sizeof(industrial_obstacles)
                            / sizeof(industrial_obstacles[0])));
                    control_result = robot_control_start_velocity_control();
                    if (control_result == ROBOT_APP_OK) {
                        trajectory_active = 1;
                        trajectory_kind = ROBOT_TRAJECTORY_CARTESIAN;
                    } else {
                        robot_cartesian_stop(&active_cartesian);
                    }
                } else {
                    control_result = ROBOT_APP_INVALID_ARGUMENT;
                }
            }
            (void) xSemaphoreGive(trajectory_mutex);
            if (control_result != ROBOT_APP_OK) {
                robot_control_report_error(control_result);
            } else {
                for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                    robot_joint_pid_reset(&joint_pid[index]);
                }
            }
            increment_health(&health.path_commands);
        }
    }
}

static void pid_control_task(void *argument)
{
    robot_control_status_t snapshot;

    (void) argument;
    for (;;) {
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        robot_cartesian_status_t cartesian_result = ROBOT_CARTESIAN_OK;
        (void) xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
        if (trajectory_active != 0) {
            robot_trajectory_point_t point;
            robot_control_status_t status;

            robot_control_get_status(&status);
            if (trajectory_kind == ROBOT_TRAJECTORY_CARTESIAN) {
                float target[ROBOT_CONTROL_JOINT_COUNT];
                cartesian_result = robot_cartesian_update(&active_cartesian,
                    0.01f, target);
                for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                    point.position[index] = target[index];
                }
                if (cartesian_result == ROBOT_CARTESIAN_COMPLETE) {
                    trajectory_active = 0;
                    trajectory_kind = ROBOT_TRAJECTORY_NONE;
                } else if (cartesian_result == ROBOT_CARTESIAN_IK_FAILED) {
                    trajectory_active = 0;
                    trajectory_kind = ROBOT_TRAJECTORY_NONE;
                }
            } else {
                (void) robot_trajectory_sample_trapezoid(
                    &active_trajectory, trajectory_time_s, &point);
            }
            if (cartesian_result != ROBOT_CARTESIAN_IK_FAILED) {
                for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                    float velocity_command;
                    if (robot_joint_pid_update(&joint_pid[index], point.position[index],
                        status.position_rad[index], &velocity_command) == ROBOT_PID_OK) {
                        (void) robot_control_set_velocity_command(index, velocity_command);
                    }
                }
                robot_control_update_velocity_control(0.01f);
                if (cartesian_result == ROBOT_CARTESIAN_COMPLETE) {
                    motion_command_t hold_command = {
                        .mode = 0U,
                        .joint_mask = 0x3FU,
                        .max_velocity_rad_s = 1.0f,
                        .max_acceleration_rad_s2 = 1.0f
                    };
                    for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                        hold_command.target_position_rad[index] = point.position[index];
                    }
                    (void) robot_control_handle_motion(&hold_command);
                }
            }
            trajectory_time_s += 0.01f;
            if (trajectory_kind == ROBOT_TRAJECTORY_JOINT
                && trajectory_time_s >= active_trajectory.duration_s) {
                trajectory_active = 0;
                trajectory_kind = ROBOT_TRAJECTORY_NONE;
            }
        } else {
            robot_control_update(0.01f);
        }
        (void) xSemaphoreGive(trajectory_mutex);
        if (trajectory_active == 0 && cartesian_result == ROBOT_CARTESIAN_IK_FAILED) {
            robot_control_report_error(ROBOT_APP_INVALID_ARGUMENT);
        }
        robot_control_get_status(&snapshot);
        if (status_queue != NULL) {
            (void) xQueueOverwrite(status_queue, &snapshot);
        }
        increment_health(&health.pid_cycles);
    }
}

void robot_tasks_tick_isr(void)
{
    static uint32_t tick_divider;
    BaseType_t higher_priority_task_woken = pdFALSE;

    tick_divider++;
    if (tick_divider < ROBOT_CONTROL_PERIOD_TICKS) {
        return;
    }
    tick_divider = 0U;
    if (pid_task_handle != NULL) {
        vTaskNotifyGiveFromISR(pid_task_handle, &higher_priority_task_woken);
    }
}

static void status_monitor_task(void *argument)
{
    robot_control_status_t snapshot;

    (void) argument;
    for (;;) {
        if (xQueueReceive(status_queue, &snapshot, portMAX_DELAY) == pdPASS) {
            int valid = snapshot.state <= ROBOT_CONTROL_ERROR;
            for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
                valid = valid && isfinite(snapshot.position_rad[index])
                    && isfinite(snapshot.velocity_rad_s[index]);
            }
            if (!valid) {
                increment_health(&health.health_faults);
            }
            increment_health(&health.status_samples);
        }
    }
}

robot_tasks_status_t robot_tasks_start(void)
{
    if (path_queue != NULL || status_queue != NULL) {
        return ROBOT_TASKS_OK;
    }
    path_queue = xQueueCreate(ROBOT_PATH_QUEUE_LENGTH, sizeof(robot_path_command_t));
    status_queue = xQueueCreate(ROBOT_STATUS_QUEUE_LENGTH,
        sizeof(robot_control_status_t));
    health_mutex = xSemaphoreCreateMutex();
    trajectory_mutex = xSemaphoreCreateMutex();
    if (path_queue == NULL || status_queue == NULL || health_mutex == NULL
        || trajectory_mutex == NULL) {
        return ROBOT_TASKS_CREATE_FAILED;
    }
    health.pid_cycles = 0U;
    health.path_commands = 0U;
    health.status_samples = 0U;
    health.health_faults = 0U;
    trajectory_time_s = 0.0f;
    trajectory_active = 0;
    trajectory_kind = ROBOT_TRAJECTORY_NONE;
    for (uint8_t index = 0U; index < ROBOT_CONTROL_JOINT_COUNT; index++) {
        if (robot_joint_pid_init(&joint_pid[index], &pid_config) != ROBOT_PID_OK) {
            return ROBOT_TASKS_CREATE_FAILED;
        }
    }
    if (xTaskCreate(path_planning_task, "path", ROBOT_PATH_TASK_STACK, NULL,
        ROBOT_PATH_TASK_PRIORITY, NULL) != pdPASS
        || xTaskCreate(pid_control_task, "pid", ROBOT_PID_TASK_STACK, NULL,
        ROBOT_PID_TASK_PRIORITY, &pid_task_handle) != pdPASS
        || xTaskCreate(status_monitor_task, "status", ROBOT_STATUS_TASK_STACK, NULL,
        ROBOT_STATUS_TASK_PRIORITY, NULL) != pdPASS) {
        return ROBOT_TASKS_CREATE_FAILED;
    }
    return ROBOT_TASKS_OK;
}

void robot_tasks_bootstrap_task(void *argument)
{
    (void) argument;
    if (robot_tasks_start() != ROBOT_TASKS_OK) {
        vTaskSuspend(NULL);
    }
    vTaskDelete(NULL);
}

robot_tasks_status_t robot_tasks_submit_motion(const motion_command_t *command)
{
    robot_path_command_t path_command;

    if (command == NULL || path_queue == NULL) {
        return ROBOT_TASKS_INVALID_ARGUMENT;
    }
    path_command.type = ROBOT_PATH_MOTION;
    path_command.motion = *command;
    return xQueueSend(path_queue, &path_command, 0U) == pdPASS
        ? ROBOT_TASKS_OK : ROBOT_TASKS_QUEUE_FULL;
}

robot_tasks_status_t robot_tasks_submit_cartesian(const robot_path_command_t *command)
{
    if (command == NULL || path_queue == NULL
        || (command->type != ROBOT_PATH_CARTESIAN_LINE
            && command->type != ROBOT_PATH_CARTESIAN_ARC)) {
        return ROBOT_TASKS_INVALID_ARGUMENT;
    }
    return xQueueSend(path_queue, command, 0U) == pdPASS
        ? ROBOT_TASKS_OK : ROBOT_TASKS_QUEUE_FULL;
}

void robot_tasks_get_health(robot_tasks_health_t *output)
{
    if (output == NULL || health_mutex == NULL) {
        return;
    }
    (void) xSemaphoreTake(health_mutex, portMAX_DELAY);
    *output = health;
    (void) xSemaphoreGive(health_mutex);
}