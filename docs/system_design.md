# 六轴机器人控制系统技术方案

## 1. 项目目标

建立 Cortex-M4 + FreeRTOS 固件和 PyBullet 仿真环境，验证机器人控制系统从通信指令到仿真状态的基本流程。

## 2. 软件分层

```text
PyBullet 仿真端
    │ 二进制协议
通信层：protocol / uart / communication
    │ robot_frame_t、robot_communication_*()
应用层：FreeRTOS 任务、状态机和控制流程
    │ motion_command_t、robot_control_*()
算法层：运动学、轨迹规划、PID
    │ joint_state_t、trajectory_point_t
驱动抽象层：clock / gpio / timer / joint_motor
```

| 层 | 当前职责 |
|---|---|
| 仿真层 | 加载 UR5、读取关节状态、提供仿真场景 |
| 通信层 | UART 缓存、帧编码、帧解析、命令响应 |
| 驱动抽象层 | 时钟、GPIO 电平、周期回调 |
| 应用层 | FreeRTOS 任务、系统状态机、命令分发和状态汇总 |
| 算法层 | 运动学、轨迹规划、PID；第一周只定义契约，后续阶段实现 |

### 2.1 依赖与调用规则

依赖方向固定为“上层调用下层”：仿真端通过通信层交互，应用层调用通信层和算法层，算法层调用驱动抽象层中的关节电机接口，驱动抽象层不得调用应用层或算法层。通信层只负责字节流、帧和协议错误，不直接修改关节目标；应用层是唯一的命令解释和系统状态机入口。

跨任务数据应通过结构体复制、FreeRTOS 队列或互斥量保护的共享对象传递，不跨层暴露 FreeRTOS 类型。当前实现尚未引入命令/状态队列或互斥量；调用者必须保证初始化对象指针有效，已实现的输出接口会检查 NULL 输出指针。接口返回值只表示调用结果，业务状态通过输出结构体或响应码返回。

### 2.2 驱动抽象层接口

驱动层接口屏蔽 QEMU 模型和后续 STM32 HAL/寄存器实现差异。当前已有接口如下，通常返回 `0` 表示成功，负值表示错误；UART 的 `EMPTY` 是非负状态码，GPIO 和 Timer 使用负值错误码。接口参数校验以当前实现为准。

| 模块 | 调用接口 | 输入/输出约束 | 返回值 |
|---|---|---|---|
| 时钟 | `bsp_clock_init()`、`bsp_clock_get_hz()`、`bsp_tick_get()`、`bsp_delay_ms(ms)` | 初始化一次；时间单位为 ms；tick 为无符号系统节拍 | 初始化和延时无返回值，频率/tick 返回当前值 |
| GPIO | `bsp_gpio_write(pin, level)`、`bsp_gpio_toggle(pin)`、`bsp_gpio_read(pin, &level)` | `pin` 必须小于 16；read 输出指针不可为 NULL；当前实现将非 `BSP_GPIO_HIGH` 的 level 按低电平处理 | `BSP_GPIO_OK`、`BSP_GPIO_INVALID_PIN` |
| 定时器 | `bsp_timer_start_periodic(period_ms, callback, context)` | 周期大于 0，回调不可为 NULL；回调不得阻塞 | `BSP_TIMER_OK`、`BSP_TIMER_ERROR` |
| UART RX | `robot_uart_init(&rx)`、`robot_uart_rx_isr_push(&rx, byte)`、`robot_uart_read(&rx, &byte)`、`robot_uart_available(&rx)` | RX 环形缓存容量为 256 字节，实际可用容量为 255 字节；ISR 只入队 | `ROBOT_UART_OK`、`ROBOT_UART_EMPTY`、`ROBOT_UART_FULL` |
| UART TX | `robot_uart_tx_init(&tx)`、`robot_uart_tx_write(&tx, byte)`、`robot_uart_tx_read(&tx, &byte)`、`robot_uart_tx_available(&tx)` | TX 发送队列容量为 256 字节，满时不覆盖旧数据 | `ROBOT_UART_OK`、`ROBOT_UART_EMPTY`、`ROBOT_UART_FULL` |

关节电机抽象固定为六路实例，当前接口为 `robot_joint_init(id)`、`robot_joint_set_target(id, position_rad, max_velocity_rad_s, max_acceleration_rad_s2)`、`robot_joint_read_encoder(id, &encoder_count)`、`robot_joint_get_position(id, &position_rad)`、`robot_joint_get_velocity(id, &velocity_rad_s)`、`robot_joint_get_state(id, &state)` 和 `robot_joint_stop(id)`；`id` 范围为 `0..5`，位置单位为 rad，速度单位为 rad/s，编码器模拟分辨率为 4096 counts/rev，状态读取不得触发硬件阻塞。当前实现使用无硬件依赖的电机模型：目标位置经过速度和加速度限制后更新，编码器计数反算位置，速度由相邻编码器位置采样计算。上层只依赖这些统一接口，不直接依赖具体电机实现。

### 2.3 算法层接口

算法层规划为只接收与输出纯 C 数据结构，不依赖 UART、FreeRTOS 或 PyBullet。角度单位统一为 rad，时间单位统一为 s，数组长度固定为 6，避免 MCU 运行时分配内存。

```c
typedef struct {
    float position_rad[6];
    float velocity_rad_s[6];
    uint32_t timestamp_tick;
} joint_state_t;

typedef struct {
    float position_rad[6];
    float velocity_rad_s[6];
    float acceleration_rad_s2[6];
    uint32_t timestamp_tick;
} trajectory_point_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float deadband;
} pid_config_t;
```

接口设计：`robot_kinematics_fk(joint, &pose)`、`robot_kinematics_ik(&pose, solutions, &count)`、`robot_trajectory_next(&trajectory, now_s, &point)`、`robot_pid_init(&pid, &config)` 和 `robot_pid_update(&pid, target, feedback, dt_s, &output)`。。

### 2.4 应用层接口与状态机

应用层把通信帧转换为控制对象，负责检查命令语义、调用算法、更新六路目标并汇总系统状态。应用层不解析 SOF、CRC，也不直接读写 UART。

当前实现使用以下控制对象：

```c
typedef struct {
    uint8_t mode;
    uint8_t joint_mask;
    float target_position_rad[6];
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
} motion_command_t;

typedef struct {
    robot_control_state_t state;
    uint8_t error_code;
    float target_position_rad[6];
    float position_rad[6];
    float velocity_rad_s[6];
} robot_control_status_t;
```

应用层当前公开 `robot_control_init()`、`robot_control_handle_motion(&command)`、`robot_control_get_status(&status)`、`robot_control_stop()` 和 `robot_control_update(dt_s)`；未实现 `robot_control_handle_config()`。通信任务收到 `ROBOT_FRAME_COMMAND` 后调用命令分发入口，应用层返回的业务结果再由通信层编码为同一 `sequence` 的响应帧。当前状态迁移主要覆盖 `IDLE -> RUNNING -> STOPPED`，并保留 `INIT`、`ERROR` 状态枚举。

当前应用层错误码为 `ROBOT_APP_OK`、`ROBOT_APP_INVALID_ARGUMENT`、`ROBOT_APP_INVALID_STATE` 和 `ROBOT_APP_LIMIT`。通信响应码仍使用协议层 `ROBOT_STATUS_*`，不得把应用错误码直接当作协议响应码；当前通信层通过映射函数转换，其中非法参数和限位错误映射为长度错误，其他非成功结果映射为未知命令。

### 2.5 通信层接口、数据结构与错误码

通信层现有公开接口为 `robot_communication_init(&communication, &rx, &tx)`、`robot_communication_poll(&communication, tick)`、`robot_communication_send_status(&communication, sequence, task_counter, timer_counter)` 和 `robot_communication_task(argument)`。帧对象使用 `robot_frame_t`，字段含义和字节序以 [通信协议](communication_protocol.md) 为准；负载最大 128 字节，整数采用 little-endian。

第一版负载约定如下：

| 命令 | 请求 payload | 响应/状态 payload |
|---|---|---|
| `MOTION` | `mode(1)`、`joint_mask(1)`、6 个 `float32` 目标角度、`float32` 最大速度、`float32` 最大加速度，共 34 字节 | 空负载；业务错误放在 response code |
| `CONFIG` | `parameter_id(1)`、`operation(1)`、参数值（按参数定义） | 查询返回参数值，设置返回空负载 |
| `STATUS` | 空负载 | 命令响应为 50 字节的状态、位置和速度数据；周期状态帧在其前追加 `task_counter(uint32)`、`timer_counter(uint32)`，总长 58 字节 |

解析器结果 `ROBOT_PROTOCOL_NEED_MORE` 和 `ROBOT_PROTOCOL_FRAME_READY` 不是错误；`ROBOT_PROTOCOL_BAD_FRAME`、`ROBOT_PROTOCOL_OVERSIZE`、`ROBOT_PROTOCOL_TIMEOUT`、`ROBOT_PROTOCOL_DUPLICATE` 分别映射到 `ROBOT_STATUS_BAD_CRC`/长度错误、`ROBOT_STATUS_BAD_LENGTH`、`ROBOT_STATUS_TIMEOUT`、`ROBOT_STATUS_DUPLICATE`。UART 满映射为 `ROBOT_STATUS_OVERFLOW`，未知命令或非法帧类型映射为 `ROBOT_STATUS_BAD_COMMAND`。通信层必须保持请求序号，重复帧只响应不重复执行。

通信层不吞掉错误：编码失败、TX 队列满和应用层处理失败都必须生成可观察的错误计数或响应；当前 `robot_communication_t` 中的 `rx_errors`、`duplicate_frames`、`handled_frames` 是第一版最小诊断计数器。

## 3. FreeRTOS 任务

| 任务 | 优先级 | 周期/行为 |
|---|---:|---|
| `communication` | 3 | 每 10 ms 读取并解析 UART 缓存 |
| `timer` | 2 | 当前配置为每 100 ms 唤醒一次，调用 GPIO 翻转和计数回调 |
| `heartbeat` | 1 | 每 100 ms 更新心跳计数 |
| `sched_high` | 4 | 调度验证任务，每 20 ms 记录一次运行后延时 |
| `sched_low` | 2 | 调度验证任务，每 50 ms 记录一次运行后延时 |
| Idle | 0 | FreeRTOS 空闲任务 |

系统节拍由 Cortex-M4 SysTick 提供，当前频率为 100 Hz，即 10 ms 一个 Tick。

通信任务拥有命令分发权；当前 `robot_communication_task()` 每 10 ms 读取并处理 UART 数据，随后推进一次关节模型。算法计算不得在 UART ISR 中执行。当前定时器由独立的 FreeRTOS `timer` 任务驱动，周期为 100 ms，回调只执行 GPIO 翻转、读取和计数。当前版本未创建命令队列、状态队列或状态互斥量；`robot_control_status_t` 在通信任务和控制更新路径中直接访问，后续引入多任务共享时必须增加互斥量或消息队列保护。

### 3.1 调度验证与日志

固件启动时通过 `scheduler_validation_start()` 创建 `sched_high`（优先级 4）和 `sched_low`（优先级 2）两个验证任务。高优先级任务先运行并调用 `vTaskDelay(20 ms)`，阻塞期间低优先级任务获得运行机会；低优先级任务调用 `vTaskDelay(50 ms)`，因此两个任务都能在不同时间点重复运行。验证任务不访问 UART，不影响通信链路；若任务创建失败，启动函数直接返回，当前没有额外错误响应。

验证日志由 `scheduler_log_entry_t` 组成，保存在固定大小的内存数组中，不使用堆和 `printf`。每条记录包含 `tick`、事件类型以及两个任务的累计运行次数；通过 `scheduler_validation_log_count()` 和 `scheduler_validation_log_get(index)` 导出给调试器或后续串口日志模块。事件值为：`1=HIGH_START`、`2=HIGH_DELAY`、`3=LOW_START`、`4=LOW_DELAY`。

验收条件：日志中必须出现 `HIGH_START -> HIGH_DELAY -> LOW_START`，且后续同时出现 `HIGH_DELAY` 和 `LOW_DELAY`；相邻事件的 tick 应体现 20 ms/50 ms 延时带来的切换。若高优先级任务不延时，低优先级任务不会获得运行机会，这可直接由日志缺少 `LOW_START` 判定。日志数组写满后停止追加，不覆盖历史记录。

一次启动后的调试器导出日志格式如下（tick 以 100 Hz 为基准，具体首个 tick 可能因启动时序不同而变化）：

```text
tick,event,high_counter,low_counter
0,HIGH_START,0,0
0,HIGH_DELAY,1,0
0,LOW_START,1,0
0,LOW_DELAY,1,1
20,HIGH_DELAY,2,1
50,LOW_DELAY,2,2
```

该日志证明高优先级任务先运行、`vTaskDelay(20 ms)` 后低优先级任务得到执行，并且两个不同延时周期都能再次唤醒。实际导出可使用 GDB 读取 `log_entries` 数组，或由后续 UART 调试命令将相同字段打印出来。

## 4. 数据流程

```text
主机/PyBullet
    ↓
UART 接收环形缓存
    ↓
协议解析器
    ↓
通信任务
    ↓
MOTION / STATUS 命令分发
    ├─ MOTION -> 应用层/关节电机 -> 响应帧
    └─ STATUS -> 应用层状态 -> 50 字节响应帧

周期状态帧由通信层主动组装，在状态数据前追加计数器后发送。
```

请求链路的具体调用为：`robot_uart_rx_isr_push()` -> `robot_communication_poll()` -> `robot_protocol_parser_feed()` -> `handle_frame()`；MOTION 继续调用 `robot_control_handle_motion()` 和关节电机接口，STATUS 调用 `robot_control_get_status()` 后生成响应帧。周期状态使用 `robot_communication_send_status()` 主动发送。

正常命令处理结果通过 response code 返回；CRC 错误、超长帧、超时和 UART 缓存满目前由解析器/通信层返回或计数，不会统一生成错误响应帧。CONFIG 尚未实现命令分发。

## 5. 接口版本与集成约束

- 本文档和协议版本 `1` 的接口以现有头文件和实现为基线；修改结构体字段、单位、数组长度或返回值时必须同步更新协议文档、仿真脚本和测试用例。
- 所有公共接口使用固定宽度整数或 `float`，不在接口内部隐式分配堆内存；Python 仿真端使用 IEEE-754 little-endian `float32`，不依赖固件内部结构体布局。
- 运动指令只有在应用状态为 `IDLE` 或 `RUNNING` 且通过关节限位检查后才可执行；MOTION 的 `mode=1` 调用 STOP，停止后状态为 `STOPPED`。
- 当前通信层已实现 MOTION 和 STATUS，CONFIG 仅保留命令值和协议位置，尚未实现参数配置或查询分发。
- 协议解析器将 CRC 错误、超长帧和超时作为解析错误返回；通信层通过 `rx_errors` 记录接收错误，重复帧通过 `duplicate_frames` 计数并返回重复响应。UART RX/TX 队列通过 `ROBOT_UART_FULL` 报告满状态。
- 双向链路分为两级验收：`firmware/tests/communication_test.c` 在 host 进程内验证请求帧到响应帧的通信层闭环；`simulation/scripts/qemu_link_test.py` 启动 ARM 固件运行于 QEMU，通过 stdin/stdout 连接 CMSDK UART，验证 MOTION、STATUS 和位置反馈。
- 驱动模块的 Unity 测试由 `firmware/tests/CMakeLists.txt` 独立使用 native 编译器构建，覆盖 UART、协议解析和关节电机接口；构建产物位于被忽略的 `firmware/tests/build/`。

## 6. 当前边界

当前目标为 ARM Cortex-M4，使用 QEMU `mps2-an386` 运行固件。QEMU 不提供 STM32 外设寄存器模型，因此 GPIO 和 Timer 仍是抽象行为模型；UART 已通过 QEMU CMSDK APB UART0 完成收发，基地址为 `0x40004000`，固件侧由 `firmware/bsp/qemu_uart.c` 负责寄存器读写，协议和通信层保持与 host bridge 共用。

当前已完成的联调范围包括：host bridge 通信、QEMU MOTION/STATUS 双向链路、QEMU 与 PyBullet 的六关节状态同步，以及 Unity native 驱动单元测试。`simulation/scripts/robot_cli.py` 默认使用 host bridge，增加 `--qemu` 后使用 QEMU 中的 ARM 固件；`--gui` 只负责将 STATUS 响应中的位置同步显示到 PyBullet，不直接生成机器人状态。


## 7. UR5 运动学

UR5 的 DH 参数、基座/关节/末端坐标系、齐次变换矩阵、位姿格式、8 组逆解流程和关节限位规则统一定义在 [kinematics.md](kinematics.md)。