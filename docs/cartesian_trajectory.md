# 笛卡尔空间轨迹规划

## 模块职责

`firmware/app/cartesian_trajectory.c` 以固定内存在线生成笛卡尔轨迹点，并在每个控制周期调用现有 IK 求解器，将当前位姿转换为六轴关节角。模块不保存整条轨迹序列，也不直接操作电机、通信或 RTOS。

位姿使用运动学接口约定的 `T_B_E`：4x4 行主序齐次矩阵，位置单位为 m，旋转矩阵位于左上角 3x3。关节输出单位为 rad。

## 直线插补

位置采用：

$$p(s)=(1-s)p_0+s p_1,\quad s=clamp(t/T,0,1)$$

姿态使用四元数 SLERP，避免欧拉角万向节锁。每次 `robot_cartesian_update()` 先推进 `elapsed_s`，再生成一个位姿并调用 IK；采样时刻超过总时长会钳制到终点。

### 直线位置与姿态的计算

设起点和终点位置为 $p_0,p_1$，以归一化时间 $s=\operatorname{clamp}(t/T,0,1)$ 作为插值参数，位移向量为 $\Delta p=p_1-p_0$，则：

$$p(s)=p_0+s\Delta p.$$

展开到 C 代码的三个平移元素就是 $p_i=p_{0,i}+s(p_{1,i}-p_{0,i})$。因此 $s=0$ 得到起点、$s=1$ 得到终点，所有中间位置位于两点连线段上。当前实现使用匀速参数化，时间推进由每次调用传入的 `dt_s` 决定。

姿态先将起点、终点旋转矩阵转换为单位四元数 $q_0,q_1$，计算 $d=q_0\cdot q_1$。若 $d<0$，将 $q_1$ 取反以选择较短旋转路径；当 $d>0.9995$ 时使用线性插值，否则令 $\theta=\arccos(d)$，并使用 SLERP：

$$w_0=\frac{\sin((1-s)\theta)}{\sin\theta},\qquad w_1=\frac{\sin(s\theta)}{\sin\theta},$$
$$q(s)=\frac{w_0q_0+w_1q_1}{\left\|w_0q_0+w_1q_1\right\|}.$$

归一化四元数 $(x,y,z,w)$ 再恢复旋转矩阵，例如 $R_{00}=1-2(y^2+z^2)$、$R_{01}=2(xy-zw)$、$R_{02}=2(xz+yw)$。`pose_interpolate()` 只更新旋转矩阵，直线和圆弧函数再分别更新平移列。

## 圆弧插补

圆弧由以下参数定义：

- 起点位姿；
- 终点位姿；
- `center_pose.value[0..2][3]` 圆心；
- `center_pose` 的局部 Z 轴作为圆弧平面法向；
- `direction=0` 选择负方向，`direction=1` 选择正方向。

起点和终点到圆心的半径必须相等，且圆心轴线必须与半径平面一致。实现使用 Rodrigues 公式旋转起点半径向量，位置沿圆弧变化，姿态仍使用四元数 SLERP。方向参数允许选择对应的短弧或补充长弧。

### 圆弧角度与 Rodrigues 公式

令圆心为 $c$，起点和终点半径向量为 $r_0=p_0-c$、$r_1=p_1-c$。合法圆弧要求：

$$\|r_0\|>\varepsilon,\qquad \left|\|r_0\|-\|r_1\|\right|\le10^{-4}.$$

圆弧法向量取 `center_pose` 的局部 Z 轴并归一化为 $u$。初始有向角由叉积和点积计算：

$$\theta_0=\operatorname{atan2}(\|r_0\times r_1\|,r_0\cdot r_1).$$

若 $u\cdot(r_0\times r_1)<0$，则将 $\theta_0$ 取负；随后依据 `direction` 加减 $2\pi$，选择正向或负向的补充弧。对比例 $s$，令 $\theta=s\theta_0$，代码使用 Rodrigues 公式：

$$R(u,\theta)r_0=r_0\cos\theta+(u\times r_0)\sin\theta+u\,[u\cdot r_0](1-\cos\theta).$$

所以位置为 $p(s)=c+R(u,s\theta_0)r_0$。固定圆心和旋转轴保证理想情况下 $\|p(s)-c\|=\|r_0\|$。代码中的 `cross3()`、`dot3()`、`cosf()` 和 `sinf()` 分别对应上述运算。

## 周期控制和 IK

规划接口的 `period_s` 明确记录期望控制周期，调用者应使用同一周期调用：

```c
robot_cartesian_update(&trajectory, 0.01f, output_joint);
```

实际推进量由 `dt_s` 提供，因此可以处理任务调度抖动；不应同时由定时器 ISR 和控制任务推进同一个实例。每次更新使用上一次关节输出作为 IK 当前状态，调用 `robot_kinematics_select_best()` 保持关节解连续并规避限位/奇异解。

### 周期更新、APF 与 IK 的数学链路

第 $k$ 次更新先推进时间：

$$t_k=\min(T,t_{k-1}+\Delta t_k),\qquad s_k=\frac{t_k}{T}.$$

根据路径类型得到名义目标位姿 $T_d(s_k)$。当前关节向量 $q_k$ 先通过正运动学得到当前末端位置：

$$T_c=FK(q_k),\qquad p_c=T_c[0:3,3].$$

APF 使用 $p_c$ 和名义位置 $p_d$ 计算修正位置 $p'_d$，然后只替换目标齐次矩阵的平移列，姿态插值结果不变。最终求解：

$$\{q^{(0)},\ldots,q^{(n-1)}\}=IK(T'_d,q_k),$$
$$q_{k+1}=\arg\min_{q^{(j)}}\operatorname{score}(q^{(j)},q_k).$$

第二式表示 `robot_kinematics_select_best()` 选择与上一周期状态最连续且满足约束的解。选中的关节向量同时保存为 `current_joint` 和 `output_joint`，供下一周期使用。若 IK 无解，轨迹停止并返回 `ROBOT_CARTESIAN_IK_FAILED`；当 $t_k\ge T-\varepsilon$ 时输出终点解并完成。

因此实际数据流为：

$$p_d\xrightarrow{\mathrm{APF}(p_c,p_d)}p'_d\xrightarrow{IK}q_{k+1}.$$

APF 修正发生在笛卡尔几何路径和 IK 之间，不直接对关节角加扰动。

除 APF 自身的两处限制（吸引项不超过剩余距离；偏移量每周期变化不超过名义推进量加余量）
外，本模块还在 $\|p'_d-p_d\|$ 上做同样的每周期变化率限制，两处共同保证修正后的目标不会
在相邻周期来回跳变。细节见 [artificial_potential_field.md](artificial_potential_field.md)。

返回值含义：

- `ROBOT_CARTESIAN_OK`：输出了一个中间关节指令；
- `ROBOT_CARTESIAN_COMPLETE`：输出终点指令并完成；
- `ROBOT_CARTESIAN_IK_FAILED`：当前笛卡尔点无可用 IK 解，轨迹停止；
- `ROBOT_CARTESIAN_INVALID_PATH`：位姿、时间、周期或起点关节非法。

