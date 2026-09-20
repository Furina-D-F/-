# 通信协议

## 帧格式

| 字段 | 字节数 | 说明 |
|---|---:|---|
| SOF | 2 | 固定 `AA 55` |
| version | 1 | 协议版本，当前为 `1` |
| type | 1 | 帧类型：`01` 命令帧、`02` 响应帧、`03` 状态帧 |
| payload_length | 2 | 负载长度（little-endian），最大 128 |
| sequence | 1 | 请求序号，重复序号视为重复帧 |
| command | 1 | 命令码，取值见下表 |
| response_code | 1 | 响应码；命令帧填 `00` |
| payload | N | 命令参数或状态数据 |
| CRC16 | 2 | 对 SOF 到 payload 的全部字段计算，低字节在前 |

最小帧长度为 11 字节，最大帧长度为 139 字节。各命令的总帧长为“头 9 + 负载 + CRC 2”：`STATUS` 请求 11、`STATUS` 响应 61、`MOTION` 45、`CARTESIAN_LINE` 75、`CARTESIAN_ARC` 104、`SIMULATION_STEP` 请求 59 / 响应 35、`SET_OBSTACLES` `12 + 36 × count`。

## 命令

| 值 | 名称 | 方向 | 说明 |
|---:|---|---|---|
| `01` | MOTION | 主机到固件 | 目标关节或轨迹指令 |
| `02` | CONFIG | 双向 | 参数配置和查询 |
| `03` | STATUS | 主机到固件 | 状态查询（请求-响应）；payload 为 `state(1)`、`error_code(1)`、6 个位置 `float32`、6 个速度 `float32`，共 50 字节 |
| `04` | CARTESIAN_LINE | 主机到固件 | 笛卡尔直线轨迹，周期 IK |
| `05` | CARTESIAN_ARC | 主机到固件 | 笛卡尔圆弧轨迹，周期 IK |
| `06` | SIMULATION_STEP | 双向 | 处理器在环闭环：请求 payload 为 6 个位置和 6 个速度 `float32`（48 字节），响应用 24 字节负载回传 6 个关节速度指令 |
| `07` | SET_OBSTACLES | 主机到固件 | 下发病例障碍物列表，供 APF 使用 |

## 响应码

`00` 成功，`01` 协议负载长度错误，`02` 未知命令或帧类型，`03` CRC 错误，`04` 接收超时，`05` 重复帧，`06` 接收缓存溢出；`07` 应用参数非法，`08` 当前控制状态不允许，`09` 目标超出关节限位。

当前通信层已实现 `MOTION`、`STATUS`、`CARTESIAN_LINE`、`CARTESIAN_ARC`、`SIMULATION_STEP` 和
`SET_OBSTACLES` 六条命令。`CONFIG` 虽已预留命令值，但尚未实现参数配置或查询分发。
CRC 错误、接收超时和接收缓存溢出目前由解析器或通信层记录为接收错误，不会单独编码为响应帧。重复帧不重复执行：若其序号和命令与缓存的上次响应一致，固件直接重发该响应（主机丢帧后重发是幂等的）；只有序号或命令不匹配时才回 `ROBOT_STATUS_DUPLICATE`。
协议错误码（`01`-`06`）描述帧格式、传输或去重问题；业务错误码（`07`-`09`）仅用于已成功解析的命令，分别对应 `ROBOT_APP_INVALID_ARGUMENT`、`ROBOT_APP_INVALID_STATE` 和 `ROBOT_APP_LIMIT`。

MOTION payload 固定为 34 字节：`mode(1)`、`joint_mask(1)`、6 个 little-endian `float32` 目标角度、最大速度和最大加速度。`mode=0` 为目标运动，`mode=1` 为停止；停止模式仍需提供非零 `joint_mask`。STATUS payload 固定为 50 字节，全部为请求-响应，当前没有周期状态帧。

笛卡尔姿态使用紧凑的 7 个 little-endian `float32`：`x, y, z, qx, qy, qz, qw`，单位为米，四元数由固件归一化。`CARTESIAN_LINE` payload 为 64 字节：起点姿态 28 字节、终点姿态 28 字节、`duration_s` 和 `period_s` 各 4 字节。`CARTESIAN_ARC` payload 为 93 字节：起点、终点和圆心姿态各 28 字节，随后为 `direction(uint8)`、`duration_s` 和 `period_s`；方向 `0` 为负向、`1` 为正向。路径任务在收到命令后规划，PID 任务每 10 ms 调用一次笛卡尔插补和 IK，并将筛选后的关节目标送入增量式 PID。IK 失败会停止当前笛卡尔轨迹，并在 STATUS 的 `error_code` 中报告参数错误。

`SET_OBSTACLES` payload 为 `1 + 36 × count` 字节：首字节为障碍物个数，其后每个障碍物为 9 个 little-endian `float32`（AABB 的 `minimum` 三个分量、`maximum` 三个分量，另有 `clearance`、`influence`、`gain`）。AABB 坐标在 DH 系下给出。个数超过固件上限、负载长度不匹配、边界非有限或 `minimum >= maximum`、`clearance < 0`、`influence <= 0`、`gain <= 0` 都会被拒绝（`07` 应用参数非法或 `01` 负载长度错误）。下发成功后，路径任务优先使用这组障碍物，取代内置的示例障碍物；`count = 0` 表示清空。该命令用于让主机决定测试场景中的障碍物，使固件 APF 避让的物体与仿真环境中的物体完全一致。受单帧负载上限 128 字节限制，一次最多只能下发 3 个障碍物（负载 `1 + 3 × 36 = 109` 字节，帧长 120 字节）：固件障碍物数组容量为 4，但 4 个障碍物的负载为 145 字节，会被解析器按超长帧丢弃且不回响应，因此第 4 个不能通过单帧下发。

## 接收状态机

UART 中断只负责把字节放入环形缓存；通信任务从缓存取字节并交给协议解析器。解析器按帧头、固定头、长度、负载和 CRC 顺序工作。

QEMU CMSDK UART 兼容路径由 10 ms 通信任务服务单字节 FIFO，且当前没有 RTS/CTS 流控。因此主机必须**一字节一次 `write()`**：QEMU 的 CMSDK APB UART 模型 `uart_receive()` 只保留 `*buf`、忽略 `size`，整帧一次写入必然丢字节。字节之间的间隔下限很低，实测 0.2 ms 仍全部送达。间隔只是各脚本自己的调度裕量，不是协议要求，也不统一：轨迹性能测试与 `qemu_link_test.py` 用 0.2 ms，工业搬运任务（APF 生效、guest 计算更重）用 1 ms，较早的冷启动稳定性和笛卡尔链路脚本仍保留 20 ms。帧长含 CRC，间隔偏小只会变慢，不会破坏数据。实际硬件 UART 中断路径不受该 QEMU 模型限制。

- 帧头错误：丢弃当前字节并重新寻找 `AA`；
- 长度超过 128：立即丢弃当前帧；
- CRC 错误：丢弃当前帧并记录接收错误，不单独发送错误响应；
- 字节间隔超过 500 ms：清空半帧并记录接收错误，不单独发送超时响应。500 ms 为 QEMU/host 调度抖动预留裕量；实际硬件可按链路速率收紧该阈值；
- 序号与上一帧相同：判为重复帧，不重复执行运动命令；若序号和命令与缓存的上次响应一致，直接重发该响应，否则回 `ROBOT_STATUS_DUPLICATE`；
- 环形缓存满：`robot_uart_rx_isr_push()` 返回 `ROBOT_UART_FULL`，该字节被丢弃（缓存为 256 字节，正常运行由通信任务每 10 ms 排空）；当前 QEMU 接收路径不会为该情况单独发送错误响应。

## 当前驱动边界

`uart.c` 是不依赖具体 MCU 寄存器的环形接收驱动。真实 STM32 项目中，UART 中断服务函数读取 `USARTx->RDR` 后调用 `robot_uart_rx_isr_push()`；QEMU MPS2 AN386 没有 STM32 UART 寄存器，因此本阶段使用该抽象接口验证协议逻辑。