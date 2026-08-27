# 环境和依赖说明

## 1. 运行环境

| 项目 | 信息 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| Python | 3.10.12 |
| Python 环境 | 项目目录下的 `.venv` |
| 目标架构 | ARM Cortex-M4 |
| QEMU 机器 | `mps2-an386` |

## 2. 使用的软件

| 软件 | 版本 | 用途 |
|---|---|---|
| `arm-none-eabi-gcc` | 10.3.1 | Cortex-M4 交叉编译 |
| CMake | 3.22.1 | 固件工程配置 |
| Ninja | 1.10.1 | 固件构建 |
| QEMU | 6.2.0 | Cortex-M4 仿真 |
| `gdb-multiarch` | 12.1 | ARM 固件调试 |

## 3. Python 包

| 包 | 版本 | 用途 |
|---|---|---|
| PyBullet | 3.2.6 | UR5 机器人物理仿真 |
| xacro | 2.1.1 | 将 ROS 机器人描述模板转换为 URDF |

Python 包安装在项目的 `.venv` 虚拟环境中。

## 4. 项目依赖

| 依赖 | 说明 |
|---|---|
| FreeRTOS Kernel | 提供 Cortex-M4 任务调度和系统节拍 |
| Bullet3 | PyBullet 的源码和机器人仿真资源 |
| UR5 URDF 与网格 | 位于 `simulation/models/ur5/` |
| ARM GCC 运行库 | 提供裸机编译所需的 ARM 辅助函数 |
