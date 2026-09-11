# 驱动模块测试报告

## 1. 测试概况

测试设计代码：firmware/tests/unity_driver_test.c

| 项目 | 内容 |
|---|---|
| 测试日期 | 2026-09-11 |
| 测试对象 | UART 收发、协议解析、六轴关节电机、运动学、轨迹规划、笛卡尔插补、单关节 PID、人工势场和 PyBullet 静态场景 |
| 单元测试框架 | Unity 2.7.2 |
| 编译环境 | Linux native `cc`，C11，`-Wall -Wextra -Werror` |
| 测试构建 | `firmware/tests/CMakeLists.txt`，CTest |
| 目标环境测试 | ARM Cortex-M4 + FreeRTOS + QEMU `mps2-an386` |
| 测试结果 | 通过 |

## 2. Unity 单元测试覆盖

### 2.1 UART 收发

共 5 个用例，覆盖：

- RX 初始化后的空队列状态。
- RX 单字节和多字节接收顺序。
- RX 环形缓冲满边界，验证 255 字节有效容量和满状态返回值。
- RX 空队列读取及 NULL 输出指针输入。
- TX 初始化、单字节发送、读取顺序和空队列状态。

### 2.2 协议解析

共 5 个用例，覆盖：

- 帧编码后分片输入，并验证解析结果与原始帧一致。
- NULL 参数、输出缓冲区容量不足和超过最大负载长度的编码请求。
- CRC 错误帧。
- 非法协议版本和超过 128 字节的 payload。
- 接收超时和重复 sequence 检测。

通信层集成测试另外验证了协议错误和业务错误的分层返回：帧长度错误返回
`ROBOT_STATUS_BAD_LENGTH`，未知命令返回 `ROBOT_STATUS_BAD_COMMAND`；应用参数非法、
控制状态不允许和目标超限分别返回 `ROBOT_STATUS_INVALID_ARGUMENT`、
`ROBOT_STATUS_INVALID_STATE` 和 `ROBOT_STATUS_LIMIT`。

### 2.3 关节电机接口

共 6 个用例，覆盖：

- 六个关节初始化和零状态。
- 正向运动、加速度限制和最大速度限制。
- 编码器反馈及量化输出。
- 到达目标位置后停止，以及停止时速度清零。
- 非法关节 ID、NULL 输出指针。
- NaN 位置、零/负速度、零/负加速度，以及零/负/NaN 时间步长。

### 2.4 运动学、轨迹、PID 和人工势场

Unity 回归还覆盖 UR5 FK/IK、关节限位、最优逆解筛选、奇异解拒绝、三次/五次/梯形
轨迹、笛卡尔直线/圆弧插补，以及增量式 PID 的复位、积分/输出限幅、死区和非法参数。
新增人工势场测试验证末端目标接近静态长方体 AABB 时，修正方向远离障碍物。当前 QEMU Unity 输出为 `34 Tests 0 Failures 0 Ignored`。

### 2.5 静态障碍物 PyBullet 场景

场景加载 UR5、地面、长方体障碍物、起点/终点标记，并通过 QEMU 发送笛卡尔直线命令。场景周期读取 STATUS，同步固件关节反馈到 PyBullet，使用 `getClosestPoints()` 检查机器人与障碍物的接触。已验证状态保持 `RUNNING`、`error=0`，全过程 `collision=0`，最终输出 `PyBullet APF static-obstacle scene: PASS`。GUI 和 `--headless` 模式均可运行。

## 3. 测试结果统计

### 3.1 Unity 用例级统计

| 模块 | 用例数 | 通过 | 失败 | 通过率 |
|---|---:|---:|---:|---:|
| UART | 5 | 5 | 0 | 100% |
| 协议解析 | 5 | 5 | 0 | 100% |
| 关节电机 | 6 | 6 | 0 | 100% |
| 运动学 | 6 | 6 | 0 | 100% |
| 关节/笛卡尔轨迹 | 7 | 7 | 0 | 100% |
| 单关节 PID | 4 | 4 | 0 | 100% |
| 人工势场 | 1 | 1 | 0 | 100% |
| **合计** | **34** | **34** | **0** | **100%** |

### 3.2 ARM/QEMU 目标环境统计

为避免只验证 native 行为，复用同一组 34 个 Unity 用例构建
`robot_driver_unity_qemu.elf`，在 FreeRTOS 任务中运行，并通过 QEMU CMSDK UART
输出 Unity 结果。该路径同时覆盖 ARM Cortex-M4 指令集、目标 ABI、FreeRTOS 任务栈、
SysTick 和抢占调度启动环境。

| 环境 | 用例数 | 通过 | 失败 | 通过率 |
|---|---:|---:|---:|---:|
| native host | 34 | 34 | 0 | 100% |
| ARM + FreeRTOS + QEMU | 34 | 34 | 0 | 100% |

### 3.3 构建和回归统计

| 测试项 | 结果 |
|---|---|
| CTest 聚合测试 | 1/1 通过，100% |
| 既有 protocol assert 测试 | 通过 |
| 既有 joint motor assert 测试 | 通过 |
| 既有 control assert 测试 | 通过 |
| 既有 communication assert 测试 | 通过 |
| ARM 固件构建 | 通过 |
| 控制状态 mutex 集成 | 通过，通信读取、路径命令和 PID 控制更新均经统一锁保护 |
| FreeRTOS 任务架构 | 通过，通信/路径/PID/状态任务、命令队列、状态邮箱和 tick ISR 通知已接入 |
| Python 脚本语法检查 | 通过 |
| QEMU UART 基础链路 | 通过，MOTION/STATUS 正常 |
| QEMU 冷启动稳定性 | 12/12 通过；每轮覆盖 STATUS 握手、双轴 MOTION、反馈和 STOP |
| QEMU 笛卡尔链路 | 直线/圆弧命令均通过 STATUS 握手、入队、周期 IK 和运行态检查；每个用例最多 3 次冷启动 |
| PyBullet 静态障碍物场景 | UR5、长方体障碍物、起终点标记和 QEMU STATUS 同步通过；全过程 `getClosestPoints()` 无碰撞 |
| PyBullet GUI 可视化 | GUI OpenGL 窗口成功创建，实时显示 UR5、障碍物、起点/终点和反馈运动 |
| QEMU-PyBullet 无头联动 | 通过，6 个关节状态同步 |

## 4. 执行方式

构建并运行 native Unity 测试：

```bash
cmake -S firmware/tests -B firmware/tests/build -G Ninja
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure
```

构建并运行 ARM/QEMU Unity 测试：

```bash
cmake --build firmware/build --target robot_driver_unity_qemu
python3 simulation/scripts/qemu_unity_test.py
```

直接查看 Unity 用例级统计：

```bash
./firmware/tests/build/robot_driver_unity_tests
```

ARM 固件和端到端回归：

```bash
cmake --build firmware/build
python3 -m py_compile simulation/scripts/*.py
python3 simulation/scripts/qemu_link_test.py
python3 simulation/scripts/qemu_cartesian_link_test.py
python3 simulation/scripts/pybullet_apf_scene.py --headless
python3 simulation/scripts/pybullet_apf_scene.py --duration 30
python3 simulation/scripts/qemu_startup_stability_test.py
python3 simulation/scripts/qemu_pybullet_integration_test.py --headless
```

运行通信层 host 集成测试：

```bash
cc -std=c11 -Wall -Wextra -Werror \
	-Ifirmware/config -Ifirmware/app -Ifirmware/drivers \
	-Ifirmware/third_party/FreeRTOS-Kernel/include \
	-Ifirmware/third_party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F \
	firmware/tests/communication_test.c firmware/app/control.c \
	firmware/drivers/communication.c firmware/drivers/protocol.c \
	firmware/drivers/uart.c firmware/drivers/joint_motor.c \
	-lm -o /tmp/robot_communication_assert
/tmp/robot_communication_assert
```

## 5. 结论与限制

本次 Unity 测试已覆盖驱动、运动学、轨迹、笛卡尔插补、PID 和人工势场模块的主要正常路径、容量边界、非法参数和异常输入，34 个用例全部通过，通过率为 **100%**。控制模块通过 FreeRTOS mutex 保护共享控制状态和关节模型，通信读取、路径命令、周期 IK、APF 目标修正和 PID 控制更新均已完成集成验证。ARM 固件构建、QEMU 串口链路、Cartesian 线/圆弧链路、PyBullet 静态障碍物场景及 QEMU-PyBullet 联动回归均通过。

