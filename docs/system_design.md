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

跨任务数据必须通过结构体复制、FreeRTOS 队列或互斥量保护的共享对象传递，不跨层暴露 FreeRTOS 类型。所有接口的指针参数在入口检查，调用者负责保证输出对象可写；接口返回值只表示调用结果，业务状态通过输出结构体或响应码返回。

### 2.2 驱动抽象层接口

驱动层接口屏蔽 QEMU 模型和后续 STM32 HAL/寄存器实现差异。当前已有接口如下，返回值为 `0` 成功、负值错误；非负的枚举值表示非错误状态。

| 模块 | 调用接口 | 输入/输出约束 | 返回值 |
|---|---|---|---|
| 时钟 | `bsp_clock_init()`、`bsp_clock_get_hz()`、`bsp_tick_get()`、`bsp_delay_ms(ms)` | 初始化一次；时间单位为 ms；tick 为无符号系统节拍 | 初始化和延时无返回值，频率/tick 返回当前值 |
| GPIO | `bsp_gpio_write(pin, level)`、`bsp_gpio_toggle(pin)`、`bsp_gpio_read(pin, &level)` | `pin` 必须小于 16；`level` 只能为 `BSP_GPIO_LOW/HIGH`；read 输出指针不可为 NULL | `BSP_GPIO_OK`、`BSP_GPIO_INVALID_PIN` |
| 定时器 | `bsp_timer_start_periodic(period_ms, callback, context)` | 周期大于 0，回调不可为 NULL；回调不得阻塞 | `BSP_TIMER_OK`、`BSP_TIMER_ERROR` |
| UART RX | `robot_uart_init(&rx)`、`robot_uart_rx_isr_push(&rx, byte)`、`robot_uart_read(&rx, &byte)`、`robot_uart_available(&rx)` | RX 环形缓存容量为 256 字节，实际可用容量为 255 字节；ISR 只入队 | `ROBOT_UART_OK`、`ROBOT_UART_EMPTY`、`ROBOT_UART_FULL` |
| UART TX | `robot_uart_tx_init(&tx)`、`robot_uart_tx_write(&tx, byte)`、`robot_uart_tx_read(&tx, &byte)`、`robot_uart_tx_available(&tx)` | TX 发送队列容量为 256 字节，满时不覆盖旧数据 | `ROBOT_UART_OK`、`ROBOT_UART_EMPTY`、`ROBOT_UART_FULL` |

后续关节电机抽象沿用同一规则，固定为六路实例。接口暂定为 `robot_joint_init(id)`、`robot_joint_set_target(id, position_rad)`、`robot_joint_get_state(id, &state)` 和 `robot_joint_stop(id)`；`id` 范围为 `0..5`，位置单位为 rad，状态读取不得触发硬件阻塞。该接口在第二阶段实现前，算法层不得直接依赖具体电机驱动符号。

### 2.3 算法层接口

算法层只接收与输出纯 C 数据结构，不依赖 UART、FreeRTOS 或 PyBullet。角度单位统一为 rad，时间单位统一为 s，数组长度固定为 6，避免 MCU 运行时分配内存。

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

目标接口为 `robot_kinematics_fk(joint, &pose)`、`robot_kinematics_ik(&pose, solutions, &count)`、`robot_trajectory_next(&trajectory, now_s, &point)`、`robot_pid_init(&pid, &config)` 和 `robot_pid_update(&pid, target, feedback, dt_s, &output)`。正逆运动学、轨迹规划和 PID 的具体实现分别在第三、四阶段完成；接口错误统一使用 `ROBOT_ALGO_*`，至少包括 `ROBOT_ALGO_OK`、`ROBOT_ALGO_INVALID_ARGUMENT`、`ROBOT_ALGO_JOINT_LIMIT`、`ROBOT_ALGO_INVALID_POSE`、`ROBOT_ALGO_NO_SOLUTION`、`ROBOT_ALGO_SINGULAR` 和 `ROBOT_ALGO_OUT_OF_RANGE`。

### 2.4 应用层接口与状态机

应用层把通信帧转换为控制对象，负责检查命令语义、调用算法、更新六路目标并汇总系统状态。应用层不解析 SOF、CRC，也不直接读写 UART。

```c
typedef enum {
    ROBOT_CONTROL_INIT = 0,
    ROBOT_CONTROL_IDLE,
    ROBOT_CONTROL_RUNNING,
    ROBOT_CONTROL_STOPPED,
    ROBOT_CONTROL_ERROR
} robot_control_state_t;

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
    joint_state_t joints;
    uint32_t task_counter;
    uint32_t timer_counter;
} robot_system_status_t;
```

应用层公开 `robot_control_init()`、`robot_control_handle_motion(&command)`、`robot_control_handle_config(&config)`、`robot_control_get_status(&status)` 和 `robot_control_stop()`。通信任务收到 `ROBOT_FRAME_COMMAND` 后调用命令分发入口，应用层返回的业务结果再由通信层编码为同一 `sequence` 的响应帧。状态迁移只允许 `INIT -> IDLE -> RUNNING -> STOPPED`，异常时任意工作态可进入 `ERROR`，停止或复位后回到 `IDLE`。

应用层错误码分为 `ROBOT_APP_INVALID_ARGUMENT`、`ROBOT_APP_INVALID_STATE`、`ROBOT_APP_BUSY`、`ROBOT_APP_LIMIT`、`ROBOT_APP_ALGORITHM` 和 `ROBOT_APP_DRIVER`。通信响应码仍使用协议层 `ROBOT_STATUS_*`，不得把算法或应用错误码直接当作协议响应码；二者通过映射表转换。

### 2.5 通信层接口、数据结构与错误码

通信层现有公开接口为 `robot_communication_init(&communication, &rx, &tx)`、`robot_communication_poll(&communication, tick)`、`robot_communication_send_status(&communication, sequence, task_counter, timer_counter)` 和 `robot_communication_task(argument)`。帧对象使用 `robot_frame_t`，字段含义和字节序以 [通信协议](communication_protocol.md) 为准；负载最大 128 字节，整数采用 little-endian。

第一版负载约定如下：

| 命令 | 请求 payload | 响应/状态 payload |
|---|---|---|
| `MOTION` | `mode(1)`、`joint_mask(1)`、6 个 `float32` 目标角度、`float32` 最大速度、`float32` 最大加速度，共 34 字节 | 空负载；业务错误放在 response code |
| `CONFIG` | `parameter_id(1)`、`operation(1)`、参数值（按参数定义） | 查询返回参数值，设置返回空负载 |
| `STATUS` | 空负载 | `task_counter(uint32)`、`timer_counter(uint32)`，后续追加系统状态和关节反馈 |

解析器结果 `ROBOT_PROTOCOL_NEED_MORE` 和 `ROBOT_PROTOCOL_FRAME_READY` 不是错误；`ROBOT_PROTOCOL_BAD_FRAME`、`ROBOT_PROTOCOL_OVERSIZE`、`ROBOT_PROTOCOL_TIMEOUT`、`ROBOT_PROTOCOL_DUPLICATE` 分别映射到 `ROBOT_STATUS_BAD_CRC`/长度错误、`ROBOT_STATUS_BAD_LENGTH`、`ROBOT_STATUS_TIMEOUT`、`ROBOT_STATUS_DUPLICATE`。UART 满映射为 `ROBOT_STATUS_OVERFLOW`，未知命令或非法帧类型映射为 `ROBOT_STATUS_BAD_COMMAND`。通信层必须保持请求序号，重复帧只响应不重复执行。

通信层不吞掉错误：编码失败、TX 队列满和应用层处理失败都必须生成可观察的错误计数或响应；当前 `robot_communication_t` 中的 `rx_errors`、`duplicate_frames`、`handled_frames` 是第一版最小诊断计数器。

## 3. FreeRTOS 任务

| 任务 | 优先级 | 周期/行为 |
|---|---:|---|
| `communication` | 3 | 每 10 ms 读取并解析 UART 缓存 |
| `timer` | 2 | 按设定周期调用回调 |
| `heartbeat` | 1 | 每 100 ms 更新心跳计数 |
| `sched_high` | 4 | 调度验证任务，每 20 ms 记录一次运行后延时 |
| `sched_low` | 2 | 调度验证任务，每 50 ms 记录一次运行后延时 |
| Idle | 0 | FreeRTOS 空闲任务 |

系统节拍由 Cortex-M4 SysTick 提供，当前频率为 100 Hz，即 10 ms 一个 Tick。

通信任务拥有命令分发权；算法计算不得在 UART ISR 中执行。定时器回调只更新驱动/采样数据或发送轻量通知，耗时控制计算放入后续控制任务。任务间共享的 `robot_system_status_t` 必须由互斥量保护，命令队列和状态队列的队列项分别使用 `motion_command_t` 和 `robot_system_status_t`。

### 3.1 调度验证与日志

固件启动时创建 `sched_high`（优先级 4）和 `sched_low`（优先级 2）两个验证任务。高优先级任务先运行并调用 `vTaskDelay(20 ms)`，阻塞期间低优先级任务获得运行机会；低优先级任务调用 `vTaskDelay(50 ms)`，因此两个任务都能在不同时间点重复运行。验证任务不访问 UART，不影响通信链路。

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
MOTION / CONFIG / STATUS 命令
    ↓
响应帧或状态帧
```

具体调用链为：`robot_uart_rx_isr_push()` -> `robot_communication_poll()` -> `robot_protocol_parser_feed()` -> `robot_control_handle_*()` -> 算法/关节驱动接口 -> `robot_communication_send_status()` 或响应帧。任何一层失败都在本层转换为本层错误码，并由应用/通信边界映射为协议响应码。

## 5. 接口版本与集成约束

- 本文档和协议版本 `1` 的接口以现有头文件为基线；修改结构体字段、单位、数组长度或返回值时必须同步更新协议文档、仿真脚本和测试用例。
- 所有公共接口使用固定宽度整数或 `float`，不返回裸指针，不在接口内部隐式分配堆内存。
- 所有层的错误码必须可追溯到调用层；禁止用 `-1` 跨层传递未分类业务错误。现有 `robot_protocol_encode()` 和 `queue_frame()` 的 `-1` 在第二阶段应细分为参数错误、容量不足和队列满。
- 运动指令只有在应用状态为 `IDLE` 或 `RUNNING` 且参数通过关节限位检查后才可执行；`STOPPED` 和 `ERROR` 状态拒绝普通运动指令。
- Python 仿真端只依赖通信协议和 payload 定义，不依赖固件内部结构体布局；`float32` 使用 IEEE-754 little-endian 编码。
- 双向链路验收使用 `simulation/scripts/link_test.py`：脚本临时编译 `firmware/tests/communication_link_host.c`，由 Python 写入命令帧，C 端通过 UART RX 环形缓存和协议解析器处理后，从 stdout 返回响应帧，Python 再校验响应。该测试桥只用于主机验证，不属于目标固件运行时。

## 6. 当前边界

QEMU 使用 `mps2-an386` Cortex-M4 平台。该平台不提供 STM32 外设寄存器模型，因此 GPIO 和 Timer 当前是抽象行为模型，UART 已完成缓存和协议层，但尚未连接真实 QEMU UART 设备。当前双向链路由 C 协议桥验证，迁移到具体 MCU 时将 bridge 的 stdin/stdout 替换为 UART ISR 收发即可。

## 7. UR5 模型基线

仿真使用的 [ur5.urdf](../simulation/models/ur5/ur5.urdf) 必须与标准 UR5 的运动学基线一致。标准 DH 参数采用米和弧度，参数顺序为 `a、d、alpha`：

| 关节 | `a` (m) | `d` (m) | `alpha` |
|---|---:|---:|---:|
| 1 | 0 | 0.089159 | `+pi/2` |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.39225 | 0 | 0 |
| 4 | 0 | 0.10915 | `+pi/2` |
| 5 | 0 | 0.09465 | `-pi/2` |
| 6 | 0 | 0.0823 | 0 |

URDF 使用关节原点 `xyz/rpy` 表达同一条链，运动学实现采用统一的变换约定，并使用上述基线进行转换和验证。六个活动关节顺序固定为 `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、`wrist_2`、`wrist_3`，旋转轴均为局部 `z` 轴。

关节位置限位基线为：`q1=[-2pi, 2pi]`、`q2=[-2pi, 2pi]`、`q3=[-pi, pi]`、`q4=[-2pi, 2pi]`、`q5=[-2pi, 2pi]`、`q6=[-2pi, 2pi]`。已对当前固定 URDF 完成一核对，确认模型关节链、原点、轴向、DH 几何参数和位置限位与上述标准一致。后续运动学推导直接以本节参数为准；若未来替换模型，必须重新进行人工核对后才能继续生成运动学测试数据。