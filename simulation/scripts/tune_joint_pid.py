#!/usr/bin/env python3
"""Tune a single joint PID against PyBullet feedback."""

import argparse
import pathlib
import time

import numpy as np
import pybullet as p
import pybullet_data


ROOT = pathlib.Path(__file__).resolve().parents[2]


def run_trial(kp, ki, kd, target, duration, dt, joint_index=0):
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(dt)
    robot = p.loadURDF(str(ROOT / "simulation/models/ur5/ur5.urdf"), useFixedBase=True)
    active_joints = [
        index for index in range(p.getNumJoints(robot))
        if p.getJointInfo(robot, index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    joint_id = active_joints[joint_index]
    p.resetJointState(robot, joint_id, 0.0, 0.0)
    error_previous = 0.0
    error_previous_previous = 0.0
    integral = 0.0
    output = 0.0
    initialized = False
    output_limit = 2.0
    integral_limit = 1.0
    positions = []
    errors = []
    for _ in range(int(duration / dt)):
        position = p.getJointState(robot, joint_id)[0]
        error = target - position
        integral = float(np.clip(integral + error * dt, -integral_limit, integral_limit))
        delta = kp * (error - error_previous) + ki * dt * error
        if initialized:
            delta += kd / dt * (error - 2.0 * error_previous + error_previous_previous)
        unclamped = output + delta
        if (unclamped > output_limit and error > 0.0) or (
            unclamped < -output_limit and error < 0.0
        ):
            integral -= error * dt
        output = float(np.clip(unclamped, -output_limit, output_limit))
        p.setJointMotorControl2(
            robot,
            joint_id,
            p.VELOCITY_CONTROL,
            targetVelocity=output,
            force=100.0,
        )
        p.stepSimulation()
        if not initialized:
            error_previous_previous = error
        else:
            error_previous_previous = error_previous
        error_previous = error
        initialized = True
        positions.append(position)
        errors.append(error)
    p.disconnect(client)
    positions = np.asarray(positions)
    errors = np.asarray(errors)
    settled = np.flatnonzero(np.abs(errors) <= 0.01)
    settling_time = duration
    if settled.size:
        for index in settled:
            if np.all(np.abs(errors[index:]) <= 0.01):
                settling_time = index * dt
                break
    return {
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "max_overshoot": float(max(0.0, np.max(positions) - target)),
        "steady_error": float(abs(errors[-1])),
        "settling_time": settling_time,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="docs/joint_pid_tuning_report.md")
    parser.add_argument("--target", type=float, default=0.8)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=1.0 / 240.0)
    args = parser.parse_args()
    candidates = [
        (1.5, 0.20, 0.015),
        (2.0, 0.25, 0.020),
        (2.5, 0.30, 0.025),
        (3.0, 0.35, 0.030),
        (3.5, 0.40, 0.035),
    ]
    results = [run_trial(*candidate, args.target, args.duration, args.dt)
               for candidate in candidates]
    best = min(results, key=lambda item: (
        item["rmse"] + 0.5 * item["max_overshoot"] + item["steady_error"],
        item["settling_time"],
    ))
    report = [
        "# 单关节 PID 参数整定报告",
        "",
        "- 仿真器：PyBullet UR5，固定基座，第 1 关节。",
        f"- 目标位置：`{args.target:.3f} rad`，采样周期：`{args.dt:.6f} s`。",
        "- 控制器：增量式 PID，输出限幅 `2.0 rad/s`，积分限幅 `1.0`，死区 `0 rad`。",
        "- 评分：`RMSE + 0.5 * max_overshoot + steady_error`，并以调节时间作次级排序。",
        "",
        "| Kp | Ki | Kd | RMSE (rad) | 最大超调 (rad) | 稳态误差 (rad) | 调节时间 (s) |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in results:
        report.append(
            f"| {item['kp']:.3f} | {item['ki']:.3f} | {item['kd']:.3f} | "
            f"{item['rmse']:.6f} | {item['max_overshoot']:.6f} | "
            f"{item['steady_error']:.6f} | {item['settling_time']:.3f} |"
        )
    report.extend([
        "",
        f"推荐参数：`Kp={best['kp']:.3f}, Ki={best['ki']:.3f}, Kd={best['kd']:.3f}`。",
        "推荐值适用于当前仿真模型、输出限幅和 240 Hz 采样条件；更换负载、摩擦或控制周期后应重新整定。",
    ])
    output = ROOT / args.output
    output.write_text("\n".join(report) + "\n", encoding="utf-8")
    print("\n".join(report))


if __name__ == "__main__":
    main()
