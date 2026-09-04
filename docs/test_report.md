# 驱动模块测试报告

## 1. 测试概况

测试设计代码：firmware/tests/unity_driver_test.c

| 项目 | 内容 |
|---|---|
| 测试日期 | 2026-09-03 |
| 测试对象 | UART 收发、协议解析、六轴关节电机接口 |
| 单元测试框架 | Unity 2.7.2 |
| 编译环境 | Linux native `cc`，C11，`-Wall -Wextra -Werror` |
| 测试构建 | `firmware/tests/CMakeLists.txt`，CTest |
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

### 2.3 关节电机接口

共 6 个用例，覆盖：

- 六个关节初始化和零状态。
- 正向运动、加速度限制和最大速度限制。
- 编码器反馈及量化输出。
- 到达目标位置后停止，以及停止时速度清零。
- 非法关节 ID、NULL 输出指针。
- NaN 位置、零/负速度、零/负加速度，以及零/负/NaN 时间步长。

## 3. 测试结果统计

### 3.1 Unity 用例级统计

| 模块 | 用例数 | 通过 | 失败 | 通过率 |
|---|---:|---:|---:|---:|
| UART | 5 | 5 | 0 | 100% |
| 协议解析 | 5 | 5 | 0 | 100% |
| 关节电机 | 6 | 6 | 0 | 100% |
| **合计** | **16** | **16** | **0** | **100%** |


### 3.2 构建和回归统计

| 测试项 | 结果 |
|---|---|
| CTest 聚合测试 | 1/1 通过，100% |
| 既有 protocol assert 测试 | 通过 |
| 既有 joint motor assert 测试 | 通过 |
| 既有 control assert 测试 | 通过 |
| 既有 communication assert 测试 | 通过 |
| ARM 固件构建 | 通过 |
| Python 脚本语法检查 | 通过 |
| QEMU UART 基础链路 | 通过，MOTION/STATUS 正常 |
| QEMU-PyBullet 无头联动 | 通过，6 个关节状态同步 |

## 4. 执行方式

构建并运行 Unity 测试：

```bash
cmake -S firmware/tests -B firmware/tests/build -G Ninja
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure
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
python3 simulation/scripts/qemu_pybullet_integration_test.py --headless
```

## 5. 结论与限制

本次 Unity 测试已覆盖驱动模块的主要正常路径、容量边界、非法参数和异常输入，16 个用例全部通过，通过率为 **100%**。现有 ARM 固件构建、QEMU 串口链路和 QEMU-PyBullet 联动回归也均通过。
