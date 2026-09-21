# 系统功能测试

## 1. 测试概况

| 项目 | 内容 |
|---|---|
| 被测对象 | UART 驱动、协议编解码、通信层、关节电机接口、FreeRTOS 任务架构、控制状态机、人工势场避障、搬运任务全流程 |
| 用例代码 | `firmware/tests/`（Unity 2.7.2）+ `simulation/scripts/`（链路与集成） |
| 运行环境 | Linux native `cc`；ARM Cortex-M4 + FreeRTOS + QEMU `mps2-an386`；PyBullet 作为外部被控对象 |
| 编译 | C11，`-Wall -Wextra -Werror` |
| 结论 | 全部通过：Unity 34/34、集成与链路回归全绿、搬运任务 12/12 循环完成 |

## 2. 功能用例覆盖与结果

| 模块 | 用例数 | 通过 | 覆盖内容 |
|---|---:|---:|---|
| UART 收发 | 5 | 5 | 空队列、单/多字节顺序、255 字节容量边界、NULL 指针、TX 排空 |
| 协议解析 | 5 | 5 | 分片输入、长度不足、CRC 错误、非法版本、超长 payload、超时与重复序号 |
| 关节电机 | 6 | 6 | 初始化与零状态、加速度/最大速度限幅、编码器量化、到位置停、非法 ID、NaN/零/负参数 |
| 运动学 | 6 | 6 | UR5 FK/IK、关节限位、最优逆解筛选、奇异解拒绝 |
| 关节/笛卡尔轨迹 | 7 | 7 | 三次/五次/梯形规划、直线/圆弧插补 |
| 单关节 PID | 4 | 4 | 复位、积分与输出限幅、死区、非法参数 |
| 人工势场 | 1 | 1 | 接近静态 AABB 时修正方向背离障碍 |
| **合计** | **34** | **34** | 通过率 100% |

同一组用例在 ARM + FreeRTOS + QEMU 目标环境下重建运行（`robot_driver_unity_qemu.elf`），
覆盖 Cortex-M4 指令集、目标 ABI、任务栈、SysTick 与抢占调度，结果同样 34/34 通过。

## 3. 构建与回归

| 测试项 | 结果 |
|---|---|
| CTest 聚合、protocol/joint motor/communication 既有 assert 测试 | 通过 |
| ARM 固件构建（`-Werror`）、Python 脚本语法检查 | 通过 |
| FreeRTOS 任务架构与控制状态 mutex | 通过，通信/路径/PID/状态任务、命令队列、状态邮箱、tick ISR 通知均已接入 |
| QEMU UART 基础链路、笛卡尔链路 | 通过，MOTION/STATUS 与直线/圆弧命令的握手、入队、周期 IK、运行态检查正常 |
| QEMU 冷启动稳定性 | 12/12 通过，每轮含 STATUS 握手、双轴 MOTION、反馈与 STOP |
| QEMU 外部反馈压力测试 | 通过，5/20 ms 字节间隔各 200 次往返无超时，后续实测 0.2 ms 亦全部送达 |
| PyBullet 静态障碍物场景（GUI/无头） | 通过，全过程 `getClosestPoints()` 无碰撞、`error=0` |
| QEMU-PyBullet 无头联动 | 通过，6 个关节状态同步 |

## 4. 通信层行为

- **分层错误码**：帧长度错误 `ROBOT_STATUS_BAD_LENGTH`、未知命令 `BAD_COMMAND`、应用参数非法
  `INVALID_ARGUMENT`、控制状态不允许 `INVALID_STATE`。
- **幂等重试**：固件按序号缓存并重发上一次响应，重复帧不会重复生效，主机丢帧后重发是安全的；
  主机侧按空闲超时等待（首轮快速失败、逐次退避），丢帧不会把单步拖到 30 s。
- **字节间隔**：QEMU 的 CMSDK UART 模型只接收单字节，主机必须逐字节写；间隔只是各脚本的调度
  裕量、不是协议要求（轨迹性能与链路脚本 0.2 ms，搬运任务 1 ms，较早脚本 20 ms）。帧带 CRC，
  间隔偏小只会变慢，不会破坏数据。真实硬件 UART 中断路径不受该模型限制。

## 5. 工业搬运任务全流程

作业流程按"原点 → 抓取点 → 避障点… → 放置点 → 回原点"组织，作业点序列可由场景任意加长，
每段以笛卡尔直线交给固件规划（插补 + APF + IK + PID），抓取用 `createConstraint()` 固定负载，
放置前记录负载实际位置作为放置精度。障碍物由主机经 `SET_OBSTACLES`（`0x07`）下发，固件避让的
物体与 PyBullet 渲染并做碰撞检测的物体是同一个。

| 场景 | 布置 | 段数 | 到位误差 (mm) | 放置精度 (mm) | 最大避障偏移 (mm) | 碰撞 | 周期 (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| S1 | 左工位到右工位，1 个障碍在取件段外侧 | 4 | 2.6422 | 0.8209 | 121.45 | 0 | 151～193 |
| S2 | 前工位到后工位，1 个障碍在放置段外侧 | 4 | 1.1796 | 1.5285 | 136.85 | 0 | 120～154 |
| S3 | 左工位到右工位，2 个障碍分列路径两侧 | 4 | 2.6422 | 1.7522 | 125.13 | 0 | 121～138 |
| S4 | 2 个障碍一上一下，路径在水平面内呈平放的 S 型 | 5 | 2.4108 | 1.4752 | 59.02 | 0 | 162～208 |

- 判据：声明的作业段全部执行、全程 `state=RUNNING`、`error=0`、与障碍物无接触，且末端到位
  误差 ≤ 5 mm、抓取-放置精度 ≤ 5 mm。4 个场景各重复 3 次（共 12 个循环）全部通过。
- 避障偏移是 APF 主动修正目标的证据：`--no-obstacles` 关闭障碍物后偏移降到 0.66～1.96 mm
  （开启时 59～137 mm），到位误差同时降到 0.32～0.59 mm。
- S4 的偏移较小是因为障碍物离路径更远（0.131/0.140 m）、影响半径取 0.15 m；逐周期核对显示
  第 2 段末端向障碍物反侧偏 31.6 mm、第 4 段偏 26.6 mm，两处都是背离各自障碍物的方向。

### 演示：录制 + 回放

闭环每个控制周期都要与固件往返一次，跑完一个循环需 2～3 分钟。演示拆成两步：`--record` 走一次
闭环并把固件逐周期回传的数据（关节位置/速度、下发速度指令、末端与负载位姿、链路序号/响应码/
往返与重发）写入 `simulation/scenes/pick_place_demo.json`；`--replay` 脱离 QEMU 播放同一份数据，
可慢放、可换 `--view`（侧上方俯视/近俯视/低角度侧视）。界面同屏给出作业段、控制周期、
通信 `seq`/响应码、固件 `state`/`error` 与 APF 避让量，并用不同颜色标出两个障碍与按段着色的
实际轨迹。


## 6. 单周期开销、资源占用与长跑稳定性

### 6.1 单个控制周期的计算开销

`simulation/scripts/system_performance_test.py` 内置的 C 基准编译项目源码（`cartesian_trajectory.c`、
`artificial_potential_field.c`、`kinematics.c`、`joint_pid.c`），在 C 内按固件
`robot_tasks_run_simulation_cycle()` 的笛卡尔分支跑一个控制周期：插补 + APF 修正 + 周期 IK，
再对 6 个关节执行增量式 PID 与前馈。整段循环放在 C 内，避免把主机脚本的调用开销算进被测代码。

| 阶段 | 均值 | P95 | 最大值 |
|---|---:|---:|---:|
| 插补 + 避障修正 + 周期 IK | 11.5～15.9 us | 18.5～23.9 us | 0.26～2.5 ms（主机抢占尖峰） |
| 6 关节增量式 PID + 前馈 | 0.12 us | 0.18 us | 2.1～3.6 us |
| 单周期合计 | 11.6～16.1 us | 18.7～24.0 us | 同上 |
| 其中：单次 IK 参考值 | 13.6～14.3 us | 17.6～22.9 us | — |
| APF 单次修正（100 万次平均，2 个障碍） | 0.216 us | — | — |

- IK 是单周期计算量的主体（约 14 us），PID 与 APF 修正都在亚微秒量级；相对 10 ms 控制周期，
  单周期计算量占比约 0.2%，余量约 600 倍。
- 计时口径是主机 x86-64、-O2 的算法耗时；QEMU 的 `mps2-an386` 不是周期精确模型，因此本表
  不给 MCU 等效主频下的耗时。闭环中 70 ms/控制周期的挂钟时间由仿真步进与 UART 往返决定，
  不代表固件计算量。
- 最大值来自主机抢占（毫秒级），不是算法本身；看 P95 更合适。

### 6.2 栈与堆占用

FreeRTOS 堆 64 KiB（heap_4）；任务栈水位按 FreeRTOS 建栈时写入的 0xA5 填充统计，由脚本内置的
GDB 探针在长跑过程中读取。

| 任务 | 配置栈 | 长跑峰值占用 | 水位余量 |
|---|---:|---:|---:|
| IDLE | 128 words / 512 B | 152 B（29.7%） | 360 B |
| communication | 2048 words / 8 KiB | 2760 B（33.7%） | 5432 B |
| path | 1024 words / 4 KiB | 664 B（16.2%） | 3432 B |
| pid | 2048 words / 8 KiB | 608 B（7.4%） | 7584 B |
| status | 384 words / 1.5 KiB | 360 B（23.4%） | 1176 B |

- 堆：空闲稳定在 40.0 KiB（64 KiB 的 62%），历史最小剩余 37.9 KiB，6 个循环（6054 个控制
  周期）中四个采样点均为 40.0 KiB，无下降趋势。
- 栈：占用最高的是 communication 任务的 33.7%，其余任务不超过 30%，`configCHECK_FOR_STACK_OVERFLOW = 2` 全程未触发。

### 6.3 长时间运行稳定性

`system_performance_test.py` 的第二部分连续跑 6 个完整作业循环（6054 个控制周期，
挂钟 422.5 s）：

| 指标 | 首个循环 | 末个循环 | 全程 |
|---|---:|---:|---:|
| 末端到位误差 | 1.2266 mm | 1.2266 mm | 最大 1.2266 mm |
| 抓取-放置精度 | 0.6423 mm | 0.6423 mm | 最大 0.6423 mm |
| 状态 / 错误码 | RUNNING / 0 | RUNNING / 0 | 仅 (RUNNING, 0) |
| 碰撞次数 | 0 | 0 | 0 |

6 个循环的指标逐位相同，按 5 mm 到位/放置判据 6/6 通过；中位循环挂钟 57～88 s，波动来自
宿主负载引起的串口重发，与固件行为无关。

### 6.4 当前验证的覆盖边界

- 时间口径：单周期开销是主机原生基准，用于比较各阶段计算量级；未在真实 MCU 上测周期数，
  也未做最坏执行时间（WCET）分析，中断响应与任务切换开销未单独计量。
- 资源口径：堆/栈数字来自运行中采样，覆盖 5 个应用任务 + IDLE 与 64 KiB 堆；未做小时级
  内存碎片化统计、分配失败注入，也未覆盖异常嵌套时的内核栈（MSP）深度。
- 稳定性口径：长跑为 6 个循环、6054 个控制周期、约 7 分钟设备时间，覆盖稳态闭环；未覆盖
  长时间连续运行、掉电重启、通信断开重连、负载/摩擦突变等异常注入。
- 性能口径：周期开销只测了笛卡尔直线 + 2 个障碍这一条路径（圆弧与关节空间走的是同一套
  接口，量级一致但未逐一测量）；长跑只用了 S1 场景。
- 仿真边界：被控对象是 PyBullet 刚体模型，无编码器噪声、摩擦、关节柔性；控制节拍由主机
  （QEMU 闭环约 70 ms/周期）驱动，因此这些结果不构成硬件实时性证明。

## 7. 结论与限制

驱动、协议、通信、状态机与任务架构的正常路径、边界与错误输入共 34 个用例在 native 与
ARM/QEMU 两套环境下全部通过；链路、集成与搬运任务回归全绿。搬运任务在 4 个场景 12 个循环中
全部完成，末端到位误差与抓取-放置精度均在 3 mm 以内且无碰撞，说明"轨迹规划 + APF 避障 +
周期 IK + 关节 PID"这条链路在完整作业流程中可以稳定运行。

需要说明的边界：APF 的目标修正是增量式的，只适用于运动中的轨迹跟踪。把轨迹结束后的伺服保持
窗口从 30 个控制周期放大到 150 个，即使某段与障碍物毫无交互，末端也会漂离目标约 0.4 m，而
固件仍上报 `state=RUNNING`、`error=0`。任务级脚本因此保持较小的整定窗口，并使用与抓取/放置
相匹配的 5 mm 容差；0.3 mm 量级的纯跟踪精度由[运动学测试文档](kinematics_test_report.md)单独验收。

## 8. 执行方式

```bash
# native / ARM Unity 与构建回归
cmake -S firmware/tests -B firmware/tests/build -G Ninja
cmake --build firmware/tests/build && ctest --test-dir firmware/tests/build --output-on-failure
cmake --build firmware/build --target robot_driver_unity_qemu
python3 simulation/scripts/qemu_unity_test.py

# 链路与集成
python3 simulation/scripts/link_test.py
python3 simulation/scripts/qemu_link_test.py
python3 simulation/scripts/qemu_cartesian_link_test.py
python3 simulation/scripts/qemu_startup_stability_test.py
python3 simulation/scripts/pybullet_apf_scene.py --headless
python3 simulation/scripts/qemu_pybullet_integration_test.py --headless

# 工业搬运任务（验收 / 录制 / 回放）；场景名为 S1～S4
python3 simulation/scripts/qemu_pybullet_pick_place.py
python3 simulation/scripts/qemu_pybullet_pick_place.py --scenarios S2 --repeats 1
python3 simulation/scripts/qemu_pybullet_pick_place.py --record \
	--scenarios S4 --log-file /tmp/pick_place_record.log
python3 simulation/scripts/qemu_pybullet_pick_place.py --replay \
	--record-file simulation/scenes/pick_place_demo.json

# 单周期开销、栈堆占用与长跑稳定性
python3 simulation/scripts/system_performance_test.py --scenario S1 --cycles 6 \
        --cost-cycles 4000
通信层 host 集成测试（`robot_tasks_stub.c` 提供 `robot_tasks_*` 空实现，使通信层脱离 FreeRTOS
也能独立链接）：

```bash
cc -std=c11 -Wall -Wextra -Werror \
	-Ifirmware/config -Ifirmware/app -Ifirmware/drivers \
	-Ifirmware/third_party/FreeRTOS-Kernel/include \
	-Ifirmware/third_party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F \
	firmware/tests/communication_test.c firmware/tests/robot_tasks_stub.c \
	firmware/app/control.c firmware/drivers/communication.c \
	firmware/drivers/protocol.c firmware/drivers/uart.c \
	firmware/drivers/joint_motor.c -lm -o /tmp/robot_communication_assert
/tmp/robot_communication_assert
```
