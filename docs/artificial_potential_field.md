# 静态障碍物人工势场避障

## 场景参数

固件和 PyBullet 场景共享同一个长方体障碍物：

| 参数 | 值 |
|---|---:|
| minimum | `(-0.64, -0.46, 0.30) m` |
| maximum | `(-0.58, -0.40, 0.60) m` |
| clearance | `0.04 m` |
| influence radius | `0.20 m` |
| repulsive gain | `0.02` |

起点关节为 `(0.30, -1.0, 1.0, -1.0, 0.8, 0.2)`，终点关节为
`(0.50, -1.0, 1.0, -1.0, 0.8, 0.2)`。脚本通过正运动学生成起点和终点位姿，再发送
`CARTESIAN_LINE` 命令。

## C 实现

`firmware/app/artificial_potential_field.c` 使用固定大小的 AABB 障碍物数组，不进行堆分配。
每个控制周期根据末端当前位置和笛卡尔名义目标计算：

- 吸引力：将末端拉向直线或圆弧的当前目标点；
- 斥力：对影响半径内的障碍物施加远离最近点的力；
- 安全余量：将 AABB 外扩 `clearance_m` 后参与距离计算；
- 步长限制：限制每周期笛卡尔目标修正量。

`robot_cartesian_update()` 先生成直线/圆弧名义位姿，再执行 APF 位置修正，姿态保持原
SLERP 结果，最后调用 IK 和解筛选。没有配置障碍物时，输出与原笛卡尔轨迹完全一致。
APF 计算运行在 PID 任务上下文，不运行在 UART 或 SysTick ISR 中。

## PyBullet 验证

运行无头场景：

```bash
cmake --build firmware/build
python3 simulation/scripts/pybullet_apf_scene.py --headless
```

打开可视化场景：

```bash
python3 simulation/scripts/pybullet_apf_scene.py
```

脚本加载 UR5、平面和橙色长方体，绿色/蓝色球分别表示起点/终点；通过 QEMU 发送起始关节
命令和笛卡尔直线命令，周期读取 STATUS，同步 PyBullet 关节状态，并使用
`getClosestPoints()` 检查机器人和障碍物的接触。只有全过程无碰撞且固件状态没有错误时才报告
`PASS`。
