#!/usr/bin/env python3
"""QEMU firmware + PyBullet external actuator/sensor trajectory test."""

import argparse
import json
import logging
import math
import pathlib
import struct
import sys
import time

import numpy as np
import pybullet as p

from protocol import CMD_CARTESIAN_ARC, CMD_CARTESIAN_LINE, CMD_SIMULATION_STEP, CMD_STATUS
from protocol import cartesian_arc_payload, cartesian_line_payload
from qemu_cartesian_link_test import fk
from qemu_client import QemuRobotClient

ROOT = pathlib.Path(__file__).resolve().parents[2]
DT = 0.01
START_JOINT = np.array([-0.77, -1.04, 0.96, -1.35, 1.74, -0.58])
END_JOINT = np.array([-0.97, -0.81, 1.10, -1.63, 2.10, -0.46])

DH_FROM_URDF = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
])

# 单个用例允许的失败次数：QEMU 停顿到连重试都超时时重启仿真器重跑当前用例。
QEMU_CASE_ATTEMPTS = 3


def matrix_from_quaternion(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def _quat(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1]
                              - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0]
                              - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0]
                              - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return (x, y, z, w)


def pose_from_joint(joint):
    transform = np.asarray(fk(joint), dtype=float)
    return transform[:3, 3], _quat(transform[:3, :3])


def build_arc_center(start_position, end_position, start_quaternion):
    chord = end_position - start_position
    chord_length = np.linalg.norm(chord)
    chord_axis = chord / chord_length
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(chord_axis, reference)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    normal = np.cross(chord_axis, reference)
    normal /= np.linalg.norm(normal)
    in_plane = np.cross(normal, chord_axis)
    center = (start_position + end_position) * 0.5 + in_plane * chord_length * 0.5
    rotation = np.column_stack((chord_axis, in_plane, normal))
    return center, _quat(rotation), normal


def pose_tuple(position, quaternion):
    return tuple(position.tolist()) + tuple(quaternion)


def reference_pose(kind, ratio, start_position, start_quaternion, end_position,
                   end_quaternion, center, axis):
    quaternion = p.getQuaternionSlerp(start_quaternion, end_quaternion, ratio)
    if kind == "line":
        position = start_position + ratio * (end_position - start_position)
    else:
        radius = start_position - center
        end_radius = end_position - center
        angle = math.atan2(np.linalg.norm(np.cross(radius, end_radius)),
                           np.dot(radius, end_radius))
        if np.dot(axis, np.cross(radius, end_radius)) < 0.0:
            angle = -angle
        if angle < 0.0:
            angle += 2.0 * math.pi
        theta = ratio * angle
        position = (center + radius * math.cos(theta)
                    + np.cross(axis, radius) * math.sin(theta)
                    + axis * np.dot(axis, radius) * (1.0 - math.cos(theta)))
    return position, quaternion


def load_scene():
    if p.connect(p.DIRECT) < 0:
        raise RuntimeError("无法连接 PyBullet")
    p.setGravity(0.0, 0.0, -9.81)
    p.setTimeStep(DT)
    robot_id = p.loadURDF(str(ROOT / "simulation/models/ur5/ur5.urdf"),
                          useFixedBase=True)
    # 官方 UR5 把 base/flange/tool0 定义为无质量坐标帧，PyBullet 会回退成 1 kg，
    # 使腕部多出 2 kg；清零后总质量与官方 21.05 kg 一致。
    for index in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, index)[12].decode() in ("base", "flange", "tool0"):
            p.changeDynamics(robot_id, index, mass=0.0)
    joint_ids = [index for index in range(p.getNumJoints(robot_id))
                 if p.getJointInfo(robot_id, index)[2] in
                 (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
    tool_link = next((index for index in range(p.getNumJoints(robot_id))
                      if p.getJointInfo(robot_id, index)[12] == b"tool0"),
                     joint_ids[-1])
    return robot_id, joint_ids, tool_link


def read_sensor(robot_id, joint_ids):
    states = [p.getJointState(robot_id, index) for index in joint_ids]
    return (np.array([state[0] for state in states]),
            np.array([state[1] for state in states]))


def joint_torque_limits(robot_id, joint_ids):
    return np.array([p.getJointInfo(robot_id, index)[10] for index in joint_ids])


def reset_to_joint(robot_id, joint_ids, target):
    """Place the plant at a known configuration before closing the loop."""
    for index, value in enumerate(target):
        p.resetJointState(robot_id, joint_ids[index], float(value))


def run_case(client, robot_id, joint_ids, tool_link, kind, duration, settle_steps,
             speed, trace=False):
    torque_limits = joint_torque_limits(robot_id, joint_ids)
    reset_to_joint(robot_id, joint_ids, START_JOINT)
    initial_position, initial_velocity = read_sensor(robot_id, joint_ids)
    target_joint = initial_position + (END_JOINT - START_JOINT)
    start_position, start_quaternion = pose_from_joint(initial_position)
    end_position, end_quaternion = pose_from_joint(target_joint)
    center, center_quaternion, axis = build_arc_center(
        start_position, end_position, start_quaternion)
    start_pose = pose_tuple(start_position, start_quaternion)
    end_pose = pose_tuple(end_position, end_quaternion)
    if kind == "line":
        command = CMD_CARTESIAN_LINE
        payload = cartesian_line_payload(start_pose, end_pose, duration, DT)
    else:
        command = CMD_CARTESIAN_ARC
        center_pose = pose_tuple(center, center_quaternion)
        payload = cartesian_arc_payload(start_pose, end_pose, center_pose, 1,
                                        duration, DT)
    response = client.request(CMD_SIMULATION_STEP,
                              struct.pack("<12f", *initial_position,
                                          *initial_velocity))
    if response["response_code"] != 0:
        raise RuntimeError("初始仿真传感器反馈被拒绝")
    response = client.request(command, payload)
    if response["response_code"] != 0:
        raise RuntimeError(f"{kind} 命令被拒绝: {response['response_code']}")
    position_errors = []
    orientation_errors = []
    collision_count = 0
    started = time.monotonic()
    steps = int(math.ceil(duration / DT)) + settle_steps
    settle_start = int(math.ceil(duration / DT)) + max(settle_steps // 2, 10)
    for step in range(steps):
        sensor_position, sensor_velocity = read_sensor(robot_id, joint_ids)
        feedback = struct.pack("<12f", *sensor_position, *sensor_velocity)
        response = client.request(CMD_SIMULATION_STEP, feedback)
        if response["response_code"] != 0 or len(response["payload"]) != 24:
            raise RuntimeError("仿真步进响应无效")
        commands = struct.unpack("<6f", response["payload"])
        for index, (joint_id, command_velocity) in enumerate(zip(joint_ids, commands)):
            p.setJointMotorControl2(robot_id, joint_id, p.VELOCITY_CONTROL,
                                     targetVelocity=command_velocity,
                                     force=float(torque_limits[index]))
        p.stepSimulation()
        actual_position, actual_quaternion = p.getLinkState(
            robot_id, tool_link, computeForwardKinematics=True)[4:6]
        actual_position = DH_FROM_URDF @ np.asarray(actual_position, dtype=float)
        actual_quaternion = _quat(
            DH_FROM_URDF @ matrix_from_quaternion(actual_quaternion))
        ratio = min((step + 1) * DT / duration, 1.0)
        expected_position, expected_quaternion = reference_pose(
            kind, ratio, start_position, start_quaternion, end_position,
            end_quaternion, center, axis)
        relative = matrix_from_quaternion(actual_quaternion).T @ matrix_from_quaternion(
            expected_quaternion)
        orientation_error = math.acos(float(np.clip((np.trace(relative) - 1.0) * 0.5,
                                                     -1.0, 1.0)))
        position_errors.append(float(np.linalg.norm(
            np.asarray(actual_position) - expected_position)))
        orientation_errors.append(orientation_error)
        collision_count = max(collision_count, len(p.getClosestPoints(
            robot_id, -1, 0.0)))
        if trace and (step % 10 == 0 or step < 5):
            # 对齐剖面：实际位姿应当与哪个周期的参考对齐。
            profile = []
            for offset in range(4):
                candidate, _ = reference_pose(
                    kind, min((step + offset) * DT / duration, 1.0),
                    start_position, start_quaternion, end_position,
                    end_quaternion, center, axis)
                profile.append(float(np.linalg.norm(
                    np.asarray(actual_position) - candidate)))
            print(f"step={step:4d} " + " ".join(
                f"off{offset}={value * 1000:.3f}mm"
                for offset, value in enumerate(profile)), flush=True)
        if step >= settle_start and np.linalg.norm(sensor_velocity) < 0.01:
            break
    status = client.request(CMD_STATUS)
    final_state, final_error = struct.unpack_from("<BB", status["payload"])
    steady = max(1, len(position_errors) // 10)
    return {"kind": kind, "speed": speed, "samples": len(position_errors),
            "duration": time.monotonic() - started,
            "position_rmse": float(np.sqrt(np.mean(np.square(position_errors)))),
            "orientation_rmse": float(np.sqrt(np.mean(np.square(orientation_errors)))),
            "steady_position": float(np.sqrt(np.mean(np.square(position_errors[-steady:])))),
            "steady_orientation": float(np.sqrt(np.mean(np.square(orientation_errors[-steady:])))),
            "collision": collision_count,
            "final_state": final_state, "final_error": final_error}


def print_summary(rows):
    """把已完成用例的结果打到终端（不写文件）。"""
    print("  速度   类型   样本   位置RMSE(mm)  姿态RMSE(°)  稳态位置(mm)  "
          "稳态姿态(°)  碰撞  末态/末错误码", flush=True)
    for row in sorted(rows, key=lambda item: (item["speed"], item["kind"])):
        print(f"  {row['speed']:5.2f}  {row['kind']:>4}  {row['samples']:5d}  "
              f"{row['position_rmse'] * 1000.0:11.4f}  "
              f"{math.degrees(row['orientation_rmse']):10.4f}  "
              f"{row['steady_position'] * 1000.0:10.4f}  "
              f"{math.degrees(row['steady_orientation']):10.4f}  "
              f"{row['collision']:4d}  {row['final_state']}/{row['final_error']}",
              flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0,
                        help="速度 1.0 对应的轨迹时长 (s)")
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5, 1.0, 1.5],
                        help="速度档，时长为 duration/speed")
    parser.add_argument("--kinds", nargs="+", default=["line", "arc"],
                        choices=("line", "arc"))
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--rows-cache",
                        default="/tmp/qemu_pybullet_trajectory_rows.json",
                        help="已完成用例缓存，支持分段运行后自动合并")
    parser.add_argument("--reset-cache", action="store_true",
                        help="忽略已有缓存，从头开始")
    parser.add_argument("--log-file", default="/tmp/qemu_pybullet_trajectory.log")
    parser.add_argument("--byte-interval", type=float, default=0.0002)
    parser.add_argument("--settle-steps", type=int, default=200)
    parser.add_argument("--startup-delay", type=float, default=5.0)
    parser.add_argument("--gdb-port", type=int)
    parser.add_argument("--keep-qemu-on-failure", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(filename=args.log_file, filemode="w", level=logging.DEBUG,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    robot_id, joint_ids, tool_link = load_scene()
    cache = pathlib.Path(args.rows_cache)
    meta = {"start": START_JOINT.tolist(), "end": END_JOINT.tolist(),
            "base_duration": args.duration}
    rows = []
    if not args.reset_cache and cache.exists():
        try:
            cached = json.loads(cache.read_text(encoding="utf-8"))
            if cached.get("meta") == meta:
                rows = cached["rows"]
                print(f"复用缓存用例 {len(rows)} 个：{args.rows_cache}", flush=True)
            else:
                print("缓存条件不一致，忽略缓存", flush=True)
        except (ValueError, KeyError, OSError):
            print("缓存不可用，忽略", flush=True)
    done = {(row["speed"], row["kind"]) for row in rows}
    client = None

    def connect():
        new_client = QemuRobotClient(byte_interval_s=args.byte_interval,
                                     logger=logging.getLogger("qemu.pybullet"),
                                     gdb_port=args.gdb_port,
                                     startup_delay_s=args.startup_delay)
        handshake = new_client.request(CMD_STATUS)
        if handshake["response_code"] != 0 or len(handshake["payload"]) != 50:
            new_client.close()
            raise RuntimeError("QEMU 启动 STATUS 握手失败")
        return new_client

    try:
        for speed in args.speeds:
            for kind in args.kinds:
                if (speed, kind) in done:
                    continue
                # 出错即重启
                for attempt in range(1, QEMU_CASE_ATTEMPTS + 1):
                    try:
                        if client is None:
                            client = connect()
                        row = run_case(client, robot_id, joint_ids, tool_link,
                                       kind, args.duration / speed,
                                       args.settle_steps, speed, args.trace)
                        break
                    except RuntimeError as error:
                        print(f"用例 {kind}@{speed:g} 第 {attempt} 次失败，"
                              f"重启 QEMU 后重试：{error}", flush=True)
                        if client is not None:
                            client.close()
                            client = None
                        if attempt == QEMU_CASE_ATTEMPTS:
                            raise
                rows.append(row)
                done.add((speed, kind))
                # 每个用例结束就写回缓存
                cache.write_text(json.dumps({"meta": meta, "rows": rows}),
                                 encoding="utf-8")
                print_summary(rows)
    finally:
        if client is not None and not args.keep_qemu_on_failure:
            client.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    main()
