# 六轴机器人控制系统技术方案

## 1. 项目目标

建立 Cortex-M4 + FreeRTOS 固件和 PyBullet 仿真环境，验证机器人控制系统从通信指令到仿真状态的基本流程。

## 2. 软件分层

```text
PyBullet 仿真端
    │ 二进制协议
通信层：protocol / uart / communication
    │
驱动抽象层：clock / gpio / timer
    │
应用层：FreeRTOS 任务、状态和控制逻辑
    │
算法层：运动学、轨迹规划、PID（后续实现）
```

| 层 | 当前职责 |
|---|---|
| 仿真层 | 加载 UR5、读取关节状态、提供仿真场景 |
| 通信层 | UART 缓存、帧编码、帧解析、命令响应 |
| 驱动抽象层 | 时钟、GPIO 电平、周期回调 |
| 应用层 | FreeRTOS 任务创建和任务调度 |
| 算法层 | 预留运动学、轨迹规划和 PID 接口 |

## 3. FreeRTOS 任务

| 任务 | 优先级 | 周期/行为 |
|---|---:|---|
| `communication` | 3 | 每 10 ms 读取并解析 UART 缓存 |
| `timer` | 2 | 按设定周期调用回调 |
| `heartbeat` | 1 | 每 100 ms 更新心跳计数 |
| Idle | 0 | FreeRTOS 空闲任务 |

系统节拍由 Cortex-M4 SysTick 提供，当前频率为 100 Hz，即 10 ms 一个 Tick。

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

## 5. 当前边界

QEMU 使用 `mps2-an386` Cortex-M4 平台。该平台不提供 STM32 外设寄存器模型，因此 GPIO 和 Timer 当前是抽象行为模型，UART 已完成缓存和协议层，但尚未连接真实 QEMU UART 设备。后续迁移到具体 MCU 时替换 BSP 实现即可。