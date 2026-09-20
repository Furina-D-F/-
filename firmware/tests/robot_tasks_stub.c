/**
 * @file robot_tasks_stub.c
 * @brief host 进程内通信层测试用的 robot_tasks 空实现。
 *
 * `communication.c` 把命令处理委托给 `robot_tasks_*`，而这一族实现依赖 FreeRTOS
 * 队列与任务。host 侧的两个测试
 *   - `firmware/tests/communication_link_host.c`（由 simulation/scripts/link_test.py 驱动）
 *   - `firmware/tests/communication_test.c`
 * 只验证"请求帧 -> 响应帧"的收发、解析与响应码，不需要 FreeRTOS，因此用本文件替代
 * 真实实现，使它们可以独立链接。
 *
 * 提交语义：真实固件里 `robot_tasks_submit_motion()` 只把命令入队（异步），校验由路径
 * 任务完成并把细节写进 STATUS 的 `error_code`。host 侧没有路径任务，若只返回 OK，
 * 调用方将观察不到任何状态变化，测试也就失去意义。因此这里同步调用控制层入口，
 * 把结果映射回任务状态：`ROBOT_TASKS_OK` / `ROBOT_TASKS_INVALID_ARGUMENT` 能一一对应；
 * 任务状态枚举没有 LIMIT 与 INVALID_STATE，统一用 `ROBOT_TASKS_CREATE_FAILED`
 * （通信层映射为 `ROBOT_STATUS_INVALID_STATE`）表示"已成功解析但被拒绝"。
 */

#include <stddef.h>

#include "robot_tasks.h"

robot_tasks_status_t robot_tasks_start(void)
{
    return ROBOT_TASKS_OK;
}

robot_tasks_status_t robot_tasks_submit_motion(const motion_command_t *command)
{
    robot_app_result_t result;

    if (command == NULL) {
        return ROBOT_TASKS_INVALID_ARGUMENT;
    }
    result = robot_control_handle_motion(command);
    if (result == ROBOT_APP_OK) {
        return ROBOT_TASKS_OK;
    }
    if (result == ROBOT_APP_INVALID_ARGUMENT) {
        return ROBOT_TASKS_INVALID_ARGUMENT;
    }
    return ROBOT_TASKS_CREATE_FAILED;
}

robot_tasks_status_t robot_tasks_submit_cartesian(
    const robot_path_command_t *command)
{
    (void) command;
    return ROBOT_TASKS_OK;
}

robot_tasks_status_t robot_tasks_set_obstacles(
    const robot_apf_obstacle_t *obstacles, uint8_t count)
{
    (void) obstacles;
    (void) count;
    return ROBOT_TASKS_OK;
}

void robot_tasks_run_simulation_cycle(void)
{
}

void robot_tasks_get_health(robot_tasks_health_t *output)
{
    if (output != NULL) {
        output->pid_cycles = 0U;
        output->path_commands = 0U;
        output->status_samples = 0U;
        output->health_faults = 0U;
    }
}
