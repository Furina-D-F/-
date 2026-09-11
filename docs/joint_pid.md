# 单关节 PID 位置闭环

## 控制器接口

实现位于 `firmware/app/joint_pid.c`，控制器状态固定存储在
`robot_joint_pid_t` 中，不使用堆内存。输入为目标位置和编码器反馈位置，输出为单关节控制量，单位由执行器接口决定；当前 PyBullet 整定将其作为 `rad/s` 速度指令。

参数通过 `robot_pid_config_t` 独立配置：

- `kp`：比例增益；
- `ki`：积分增益；
- `kd`：微分增益；
- `sample_time_s`：固定控制周期；
- `integral_limit`：积分状态限幅；
- `output_limit`：最终输出对称限幅；
- `deadband`：误差死区。

## 增量式 PID

误差为：

$$e_k=q_{target,k}-q_{feedback,k}$$

控制增量为：

$$\Delta u_k=K_p(e_k-e_{k-1})+K_iT_se_k+\frac{K_d}{T_s}(e_k-2e_{k-1}+e_{k-2})$$

输出为：

$$u_k=clamp(u_{k-1}+\Delta u_k,-u_{max},u_{max})$$

控制器首拍同时初始化两级历史误差，避免微分启动冲击。误差绝对值小于 `deadband` 时按零误差处理，降低编码器量化噪声引起的抖动。

## 抗饱和

积分状态单独限制在 `[-integral_limit, integral_limit]`。当未限幅输出已经超过输出上限，且当前误差还会继续推动输出向饱和方向增加时，冻结本拍积分，防止饱和后的积分累积和恢复超调。输出限幅始终作用于最终控制量。

## 仿真整定

使用 `simulation/scripts/tune_joint_pid.py` 对 PyBullet UR5 第一个活动关节进行闭环整定。仿真周期为 `1/240 s`，目标位置为 `0.8 rad`，输出限幅 `2.0 rad/s`，积分限幅 `1.0`。候选参数按 RMSE、最大超调和末端误差评分。

当前整定结果见 [joint_pid_tuning_report.md](joint_pid_tuning_report.md)：

```text
Kp = 2.5
Ki = 0.3
Kd = 0.025
```
