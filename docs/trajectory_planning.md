# 关节空间路径规划

## 1. 模块边界

轨迹模块位于 `firmware/app/trajectory.c`，使用固定六轴数组和 `float`，不依赖 FreeRTOS、通信或堆内存。它只生成轨迹采样点，输出位置、速度和加速度；控制层可在周期任务中调用采样接口，再将位置目标交给 `robot_joint_set_target()`。

单位统一为：位置 `rad`，速度 `rad/s`，加速度 `rad/s2`，时间 `s`。关节顺序为 `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、`wrist_2`、`wrist_3`。

## 2. 三次多项式

每个关节使用：

$$q(t)=a_0+a_1t+a_2t^2+a_3t^3$$

输入起点位置/速度、终点位置/速度和总时间 `T`，满足：

$$q(0)=q_0,\quad \dot q(0)=v_0,\quad q(T)=q_f,\quad \dot q(T)=v_f$$

实现接口为 `robot_trajectory_plan_cubic()`，通过 `robot_trajectory_sample_polynomial()` 计算任意时刻的三阶位置、二阶速度和一阶加速度。采样时间会限制在 `[0,T]`，超出时间返回终点状态，避免控制周期抖动造成越界。

### 2.1 系数推导

令单个关节的位移为 $\Delta q=q_f-q_0$。由四个边界条件：

$$q(0)=a_0=q_0,\quad \dot q(0)=a_1=v_0,$$
$$q(T)=q_f,\quad \dot q(T)=v_f.$$

消去 $a_0,a_1$ 后求得：

$$a_2=\frac{3\Delta q-(2v_0+v_f)T}{T^2},\qquad
a_3=\frac{-2\Delta q+(v_0+v_f)T}{T^3}.$$

这正对应 `robot_trajectory_plan_cubic()` 中的闭式计算；每个关节的 $a_4,a_5$ 置零。

## 3. 五次多项式

每个关节使用：

$$q(t)=a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5$$

输入起点和终点的位置、速度、加速度，共六个边界条件，因此可保证轨迹端点的加速度连续约束：

$$q(0)=q_0,\ \dot q(0)=v_0,\ \ddot q(0)=a_{start}$$

$$q(T)=q_f,\ \dot q(T)=v_f,\ \ddot q(T)=a_{end}$$

五次曲线适合需要平滑起停的工业机器人点到点运动。实现不使用矩阵或动态内存，直接计算六个系数，降低 MCU 栈和计算开销。

### 3.1 系数推导

起点约束直接给出 $a_0=q_0$、$a_1=v_0$、$a_2=\alpha_0/2$，其中 $\alpha_0$、$\alpha_f$ 是起点和终点加速度。令 $\Delta q=q_f-q_0$，将终点三个约束代入可得：

$$a_3=\frac{20\Delta q-(8v_f+12v_0)T-(3\alpha_0-\alpha_f)T^2}{2T^3},$$
$$a_4=\frac{-30\Delta q+(14v_f+16v_0)T+(3\alpha_0-2\alpha_f)T^2}{2T^4},$$
$$a_5=\frac{12\Delta q-6(v_f+v_0)T-(\alpha_0-\alpha_f)T^2}{2T^5}.$$

采样时计算：

$$
\begin{aligned}
q(t)&=a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5,\\
\dot q(t)&=a_1+2a_2t+3a_3t^2+4a_4t^3+5a_5t^4,\\
\ddot q(t)&=2a_2+6a_3t+12a_4t^2+20a_5t^3.
\end{aligned}
$$

C 实现使用 Horner 形式展开上述多项式，减少乘法次数。

## 4. 梯形速度曲线

`robot_trajectory_plan_trapezoid()` 为每个关节根据位移、最大速度和最大加速度计算：

- 加速段：速度从 0 增加到峰值；
- 匀速段：保持峰值速度；
- 减速段：速度回到 0。

短距离情况下自动退化为三角速度曲线，峰值速度为 `sqrt(distance * max_acceleration)`，不会超过配置的最大速度。长距离情况下峰值速度为最大速度。

### 4.1 峰值速度和分段公式

设单轴位移大小为 $d$，最大速度为 $v_{max}$，最大加速度为 $a_{max}$。峰值速度和加速时间为：

$$v_p=\min\left(v_{max},\sqrt{d\,a_{max}}\right),\qquad t_a=\frac{v_p}{a_{max}}.$$

匀速时间为：

$$t_c=\max\left(0,\frac{d-v_p t_a}{v_p}\right).$$

令 $\tau_d=\min(\tau-t_a-t_c,t_a)$，局部位移、速度和加速度为：

$$
(s,v,a)=
\begin{cases}
(\frac12a_{max}\tau^2,\ a_{max}\tau,\ a_{max}),&0\le\tau<t_a,\\
(\frac12a_{max}t_a^2+v_p(\tau-t_a),\ v_p,\ 0),&t_a\le\tau<t_a+t_c,\\
(\frac12a_{max}t_a^2+v_pt_c+v_p\tau_d-\frac12a_{max}\tau_d^2,\ v_p-a_{max}\tau_d,\ -a_{max}),&\tau\ge t_a+t_c.
\end{cases}
$$

最终乘以运动方向 $\sigma=\operatorname{sign}(q_f-q_0)$：

$$q=q_0+\sigma s,\qquad \dot q=\sigma v,\qquad \ddot q=\sigma a.$$

这对应 C 代码中的 `local_time`、`deceleration_time` 和 `direction`。

### 4.2 六轴时间同步

六轴首先分别计算自己的最短剖面，再以最长轴的剖面时间作为总时间。对每个关节先得到 $T_i=2t_{a,i}+t_{c,i}$，全局时间取 $T=\max_iT_i$，时间伸缩因子为 $k_i=T/T_i$。采样时 $\tau_i=t/k_i$，因此：

$$\dot q_i=\frac{\dot q_i^{local}}{k_i},\qquad
\ddot q_i=\frac{\ddot q_i^{local}}{k_i^2}.$$

由于 $k_i\ge1$，时间伸缩不会放大速度和加速度上限，并保证非零位移关节同时结束。

采样接口 `robot_trajectory_sample_trapezoid()` 对负时间取起点、超过总时间取终点；零位移关节全程保持静止。每轴速度不超过配置的最大速度，加速度不超过配置的最大加速度。

## 5. 参数和验证

所有输入数组和时间必须为有限值；多项式总时间必须大于零；梯形曲线最大速度和最大加速度必须大于零。非法参数返回 `ROBOT_TRAJECTORY_INVALID_CONSTRAINT`，空指针返回 `ROBOT_TRAJECTORY_INVALID_ARGUMENT`。

采样入口先执行 $t\leftarrow\min(\max(t,0),T)$，因此负时间取起点，超过总时间取终点。规划阶段使用 `TRAJECTORY_EPSILON=10^{-6}` 拒绝近似零时长，避免 $T^2,T^3$ 等分母退化。

当前测试覆盖：

- 三次曲线位置和速度端点约束；
- 五次曲线位置、速度和加速度端点约束；
- 梯形/三角曲线的同步终点、反向运动和速度限制；
- 非法时间、速度和加速度拒绝；
- native 与 ARM/QEMU Unity 回归。
