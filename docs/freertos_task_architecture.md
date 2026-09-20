# FreeRTOS 多任务架构

## 任务划分

| 任务 | 优先级 | 栈深度 | 周期/阻塞 | 职责 |
|---|---:|---:|---|---|
| `task_init` | 4 | 512 | 启动阶段执行一次 | 创建队列、互斥量与业务任务，完成后自删除 |
| `communication` | 3 | 2048 | 10 ms | UART 服务、协议解析、响应发送、运动命令入队 |
| `path` | 3 | 1024 | 阻塞等待运动队列 | 消费关节/笛卡尔命令，建立梯形或笛卡尔路径 |
| `pid` | 2 | 2048 | 由 tick ISR 每 10 ms 通知唤醒 | 独占 `robot_control_update(0.01f)`，推进位置闭环/电机控制 |
| `status` | 1 | 384 | 阻塞等待状态邮箱 | 检查状态快照有限性和控制状态健康 |

## 任务间数据流

```text
UART ISR/ring buffer
        |
        v
communication task -- motion_command_t --> path queue --> path task
                                                          |
                                                          v
                                                   control state/motor target
                                                          ^
                                                          |
             status mailbox <--- pid task <--- tick ISR notification
                    |
                    v
              status task / health counters
```

- 路径命令队列：长度 4，元素为 `robot_path_command_t`，同时承载关节 MOTION、笛卡尔直线和圆弧；通信任务使用非阻塞发送，队列满返回错误。
- 状态邮箱：长度 1，使用 `xQueueOverwrite()` 保存最新 `robot_control_status_t`，丢弃过时状态而不堆积延迟。
- `control.c` 内部 mutex 保护控制状态；路径任务写入目标，PID 任务更新状态，通信任务读取状态。
- 笛卡尔轨迹对象由路径任务规划、PID 任务周期更新，二者通过轨迹 mutex 保护；每次 IK 选出的关节目标直接进入现有增量式 PID 和速度执行器。
- 健康计数器使用独立 mutex 保护，统计 PID 周期、路径命令、状态样本和异常快照。

## 实时性约束

- PID 任务是唯一调用 `robot_control_update()` 的任务，避免原通信任务和 PID 任务重复推进同一个控制模型。
- `vApplicationTickHook()` 每经过 `pdMS_TO_TICKS(10)` 个 tick，通过 `vTaskNotifyGiveFromISR()` 通知 PID 任务；PID 任务阻塞在 `ulTaskNotifyTake()` 上，保持 100 Hz 控制周期。
- tick ISR 只做分频和任务通知，不执行 IK、PID 或电机控制计算；真正的控制逻辑仍在 PID 任务上下文运行。
- 路径任务不在 ISR 中执行 IK 或复杂插补；通信任务也只做协议解析和入队。
- 状态任务只处理最新快照，阻塞在邮箱上，不进行阻塞式通信。
- 所有任务创建失败都在启动阶段停机，避免系统以缺失实时任务的状态运行。

## 资源配置

任务和队列使用 FreeRTOS 动态分配，当前 `configTOTAL_HEAP_SIZE` 为 64 KiB，使用 `heap_4.c`。正式业务对象包含通信、路径、PID、状态和启动任务，另有调度验证任务；路径队列为 4 个运动命令槽，状态邮箱为 1 个槽，控制模块和任务健康统计各使用一个互斥量。若后续增加完整笛卡尔轨迹缓存，应改用固定静态存储或单独评估 heap 余量。

## 接口

入口位于 `firmware/app/robot_tasks.h`：

- `robot_tasks_start()`：创建队列、互斥量和三个业务任务，并保存 PID 任务句柄供 tick ISR 通知；
- `robot_tasks_submit_motion()`：非阻塞提交运动命令；
- `robot_tasks_set_obstacles()`：保存主机下发的障碍物列表（带互斥量保护），供 APF 使用；
- `robot_tasks_get_health()`：读取任务健康计数。

通信收到 `ROBOT_CMD_MOTION`、`ROBOT_CMD_CARTESIAN_LINE` 或 `ROBOT_CMD_CARTESIAN_ARC` 后立即入队并返回协议响应。命令的实际规划和 IK 校验由路径/PID 任务完成；因此响应表示“已接受入队”，状态反馈才表示后续控制执行结果。`ROBOT_CMD_SET_OBSTACLES` 不同：它只是在锁保护下替换障碍物列表，不入队、不影响正在执行的轨迹，因此响应直接反映参数是否被接受。

## 当前完成度与边界

已完成：UART 收发中断化、SysTick 周期回调、通信/路径/PID/状态任务拆分、运动命令队列、状态邮箱、PID tick ISR 通知唤醒、UR5 FK/IK、多解筛选、关节和笛卡尔轨迹规划、独立单关节增量式 PID、PyBullet 参数整定，以及 native/ARM/QEMU 回归验证。

当前闭环已接通：路径任务根据当前反馈和 MOTION 目标生成梯形轨迹，或建立笛卡尔直线/圆弧轨迹；PID 任务按轨迹采样或执行周期 IK，计算速度指令，控制层通过速度执行器更新关节模型，再读取编码器位置作为下一周期反馈。PID 输出语义为 `rad/s`；真实硬件移植时将 `robot_control_set_velocity_command()` 替换为速度环、力矩环或 PWM 执行器接口。

轨迹跟踪采用"速度前馈 + 增量式 PID"结构：增量式 PID 的输出量本身即速度指令，不含前馈
项时被控对象只能靠误差累积追上参考速度，等效跟踪滞后约 `1/kp`。因此 PID 任务把参考关节
速度叠加到 PID 输出上——关节轨迹取采样点的解析速度，笛卡尔轨迹取相邻周期 IK 目标的差分；
规划时用起始关节角预置"上一周期目标"，使第一个控制周期也有前馈。

前馈与反馈必须解耦：本周期目标是**本周期末端**的位置，而反馈是**本周期起点**的位置，两者
天然差一个周期的位移。若 PID 直接用末端目标减起点反馈，会把这一结构性超前当成误差；增量
式 PID 在误差恒定时输出不衰减，会冻结在 `kp × 一个周期位移` 上，把被控对象持续推快约 `kp`
比例，直到沿轨迹偏出约一个周期的位置。因此反馈通道使用"反馈位置 + 本周期前馈位移"作为预
测位置，只关注扣掉前馈后剩余的误差。修正后，闭环位置误差从毫米级降到 0.1 mm 以内。

`sched_high` 和 `sched_low` 仅用于调度验证，当前通过 `ROBOT_ENABLE_SCHEDULER_VALIDATION` 显式开启，正式固件默认关闭，避免占用实时任务的优先级和堆栈资源。QEMU CMSDK UART 兼容层关闭外设 IRQ，由通信任务统一服务 RX/TX FIFO，避免 QEMU 外设 FIFO 与任务服务同时消费造成竞态；真实 MCU UART 仍使用硬件 RX/TX 中断。

## QEMU 初始化回归

`simulation/scripts/qemu_startup_stability_test.py` 对固件执行 12 轮独立冷启动。每轮先执行 STATUS 握手，再执行双关节 MOTION、位置反馈校验和 STOP，串口按 20 ms 分字节发送。每个逻辑用例允许最多 3 次全新 QEMU 启动来处理仿真器冷启动期间偶发的首请求丢失；连续 3 次未完成完整闭环即失败。测试输出记录每轮实际启动次数，因此启动重试次数增加可作为稳定性退化信号。
