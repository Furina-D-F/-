#!/usr/bin/env python3
"""典型工业搬运场景设计
"""

import argparse
import json
import logging
import math
import pathlib
import struct
import time

import numpy as np
import pybullet as p
import pybullet_data

from protocol import (CMD_CARTESIAN_LINE, CMD_SET_OBSTACLES, CMD_SIMULATION_STEP,
                      CMD_STATUS, cartesian_line_payload, obstacles_payload)
from qemu_cartesian_link_test import fk, quaternion
from qemu_client import QemuRobotClient

ROOT = pathlib.Path(__file__).resolve().parents[2]
DT = 0.01
GRIPPER_OFFSET_M = 0.03
PAYLOAD_SIZE_M = 0.06
SETTLE_STEPS = 300
ARRIVAL_LIMIT_M = 0.005
ARRIVE_EXIT_LIMIT_M = 0.001
SETTLE_DRIFT_LIMIT = 60
QEMU_RESTART_ATTEMPTS = 6
PLACE_LIMIT_M = 0.005
DEFAULT_RECORD_FILE = "simulation/scenes/pick_place_demo.json"
RECORD_FORMAT = "pick_place_trajectory"
RECORD_VERSION = 1
RECORD_STEP_FIELDS = ("q0..q5_rad", "v0..v5_rad_s", "cmd0..cmd5_rad_s",
                      "tcp_dh_x_y_z_m", "payload_world_x_y_z_m",
                      "seq", "response_code", "round_trip_ms", "requests",
                      "retries")#回放数据
RECORD_FIELD_OFFSETS = {"q": 0, "v": 6, "cmd": 12, "tcp": 18, "payload": 21,
                        "seq": 24, "code": 25, "rtt_ms": 26, "requests": 27,
                        "retries": 28}#回放数据偏移
RECORD_STEP_WIDTH = 29
RECORD_DECIMALS = 6
PAYLOAD_AT_PICK = "at_pick"
PAYLOAD_ATTACHED = "attached"
PAYLOAD_AT_PLACE = "at_place"
PAYLOAD_MODE_LABELS = {PAYLOAD_AT_PICK: "payload at PICK",
                       PAYLOAD_ATTACHED: "payload held",
                       PAYLOAD_AT_PLACE: "payload placed"}

DH_FROM_URDF = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
])

OBSTACLE_CLEARANCE = 0.03
OBSTACLE_INFLUENCE = 0.20
OBSTACLE_GAIN = 0.02
#四个搬运场景
SCENARIOS = [
    {
        "name": "S1",
        "home": [0.0, -1.00, 0.65, -1.8, -math.pi / 2, 0.0],
        "pick": [-1.40, -1.10, 1.40, -1.8, -math.pi / 2, 0.0],
        "via": [0.0, -1.15, 0.90, -1.8, -math.pi / 2, 0.0],
        "place": [1.40, -1.10, 1.40, -1.8, -math.pi / 2, 0.0],
        "obstacles": [
            {"minimum": (-0.4973, 0.2094, 0.00), "maximum": (-0.3973, 0.3094, 0.26)},
        ],
    },
    {
        "name": "S2",
        "home": [0.0, -1.00, 0.65, -1.8, -math.pi / 2, 0.0],
        "pick": [-1.20, -1.05, 1.35, -1.8, -math.pi / 2, 0.0],
        "via": [0.0, -1.15, 0.90, -1.8, -math.pi / 2, 0.0],
        "place": [1.20, -1.05, 1.35, -1.8, -math.pi / 2, 0.0],
        "obstacles": [
            {"minimum": (-0.4592, -0.4388, 0.00), "maximum": (-0.3592, -0.3388, 0.26)},
        ],
    },
    {
        "name": "S3",
        "home": [0.0, -1.00, 0.65, -1.8, -math.pi / 2, 0.0],
        "pick": [-1.40, -1.10, 1.40, -1.8, -math.pi / 2, 0.0],
        "via": [0.0, -1.15, 0.90, -1.8, -math.pi / 2, 0.0],
        "place": [1.40, -1.10, 1.40, -1.8, -math.pi / 2, 0.0],
        "obstacles": [
            {"minimum": (-0.4973, 0.2094, 0.00), "maximum": (-0.3973, 0.3094, 0.26)},
            {"minimum": (-0.3898, -0.4371, 0.00), "maximum": (-0.2898, -0.3371, 0.26)},
        ],
    },
    {
        "name": "S4",
        "path": ("home", "pick", "via1", "via2", "place", "home"),
        "waypoint_labels": {"home": "HOME", "pick": "PICK (upper)",
                            "via1": "VIA1 (outer)", "via2": "VIA2 (inner)",
                            "place": "PLACE (lower)"},
        "leg_labels": ("原点 -> 抓取点（空载接近）",
                       "抓取点 -> 避障点 1（绕过上方障碍）",
                       "避障点 1 -> 避障点 2（搬运中）",
                       "避障点 2 -> 放置点（绕过下方障碍）",
                       "放置点 -> 原点（卸料返回）"),
        "home": [0.0, -1.00, 0.65, -1.8, -math.pi / 2, 0.0],
        "pick": [1.20, -1.05, 1.35, -1.8, -math.pi / 2, 0.0],
        "via1": [0.45, -1.00, 0.80, -1.8, -math.pi / 2, 0.0],
        "via2": [-0.45, -1.70, 1.60, -1.8, -math.pi / 2, 0.0],
        "place": [-1.20, -1.05, 1.35, -1.8, -math.pi / 2, 0.0],
        "obstacle_labels": ("OBS-1 upper", "OBS-2 lower"),
        "obstacles": [
            {"minimum": (-0.5412, -0.7143, 0.00), "maximum": (-0.4412, -0.6143, 0.28),
             "clearance": 0.02, "influence": 0.15, "gain": 0.02},
            {"minimum": (-0.3069, 0.2005, 0.00), "maximum": (-0.2069, 0.3005, 0.28),
             "clearance": 0.02, "influence": 0.15, "gain": 0.02},
        ],
    },
]
for _scenario in SCENARIOS:
    for _obstacle in _scenario["obstacles"]:
        _obstacle.setdefault("clearance", OBSTACLE_CLEARANCE)
        _obstacle.setdefault("influence", OBSTACLE_INFLUENCE)
        _obstacle.setdefault("gain", OBSTACLE_GAIN)


def pose_of(joint):
    """关节角 -> DH 系位姿矩阵。"""
    return np.asarray(fk(np.asarray(joint, dtype=float)), dtype=float)


def world_pose_of(joint):#坐标系转换
    transform = np.eye(4)
    transform[:3, :3] = DH_FROM_URDF
    return transform @ pose_of(joint)


def pose_tuple(matrix):
    """4x4 位姿矩阵 -> (x, y, z, qx, qy, qz, qw)。"""
    return tuple(quaternion(np.asarray(matrix, dtype=float)))


def payload_center(matrix):
    """夹持中心相对末端位姿的偏移。"""
    return matrix[:3, 3] + GRIPPER_OFFSET_M * matrix[:3, 2]


def load_scene(obstacles, gui=False):
    if p.connect(p.GUI if gui else p.DIRECT) < 0:
        raise RuntimeError("无法连接 PyBullet")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.setTimeStep(DT)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(str(ROOT / "simulation/models/ur5/ur5.urdf"),
                          useFixedBase=True)
    for index in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, index)[12].decode() in ("base", "flange", "tool0"):
            p.changeDynamics(robot_id, index, mass=0.0)
    joint_ids = [index for index in range(p.getNumJoints(robot_id))
                 if p.getJointInfo(robot_id, index)[2]
                 in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
    tool_link = next(index for index in range(p.getNumJoints(robot_id))
                     if p.getJointInfo(robot_id, index)[12] == b"tool0")
    obstacle_ids = []
    for index, obstacle in enumerate(obstacles):
        minimum = np.asarray(obstacle["minimum"], dtype=float)
        maximum = np.asarray(obstacle["maximum"], dtype=float)
        low = np.array([-maximum[0], -maximum[1], minimum[2]])
        high = np.array([-minimum[0], -minimum[1], maximum[2]])
        half = (high - low) * 0.5
        color = OBSTACLE_COLOR_PALETTE[index % len(OBSTACLE_COLOR_PALETTE)]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half.tolist())
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half.tolist(),
                                     rgbaColor=[*color, 1.0])
        body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=collision,
                                 baseVisualShapeIndex=visual,
                                 basePosition=((low + high) * 0.5).tolist())
        obstacle_ids.append(body)
    half = PAYLOAD_SIZE_M * 0.5
    payload_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[half, half, half])
    payload_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[half, half, half],
        rgbaColor=(0.20, 0.50, 0.95, 1.0))
    payload_id = p.createMultiBody(
        baseMass=0.2, baseCollisionShapeIndex=payload_collision,
        baseVisualShapeIndex=payload_visual, basePosition=[0.0, 0.0, -5.0])
    return robot_id, joint_ids, tool_link, obstacle_ids, payload_id


def reload_scene(obstacles, gui=False):
    """重建 PyBullet 场景

    """
    if p.isConnected():
        p.disconnect()
    return load_scene(obstacles, gui)


def joint_torque_limits(robot_id, joint_ids):
    return np.array([p.getJointInfo(robot_id, index)[10] for index in joint_ids])

DEFAULT_PATH = ("home", "pick", "via", "place", "home")
DEFAULT_WAYPOINT_LABELS = {"home": "HOME", "pick": "PICK", "via": "VIA",
                           "place": "PLACE"}
DEFAULT_LEG_LABELS = ("原点 -> 抓取点（空载接近）", "抓取点 -> 避障点（搬运中）",
                      "避障点 -> 放置点（搬运中）", "放置点 -> 原点（卸料返回）")
WAYPOINT_COLOR_PALETTE = ((0.72, 0.72, 0.72), (0.15, 0.85, 0.25),
                          (0.95, 0.75, 0.10), (0.35, 0.85, 0.95),
                          (0.20, 0.45, 1.00))
OBSTACLE_COLOR_PALETTE = ((0.95, 0.30, 0.10), (0.55, 0.25, 0.95),
                          (0.10, 0.75, 0.55), (0.95, 0.75, 0.10))
TRACE_FLOOR_ALPHA = 0.80     # 实际轨迹的地面投影
NOMINAL_FLOOR_ALPHA = 0.35   # 名义路径的地面投影
FLOOR_Z = 0.006
CAMERA_PRESETS = {
    "iso": ((2.60, -72.0, -60.0, (0.38, 0.00, 0.22)), "侧上方俯视"),
    "top": ((2.10, -90.0, -88.0, (0.38, 0.00, 0.05)), "近俯视（看平面轨迹）"),
    "side": ((2.75, -35.0, -35.0, (0.38, 0.00, 0.28)), "低角度侧视（看障碍高度）"),
}
TEXT_SIZE_REFERENCE_DISTANCE = 1.95
WAYPOINT_TEXT_SIZE = 1.30
OBSTACLE_TEXT_SIZE = 1.20
DEFAULT_VIEW = "iso"
ACTIVE_CAMERA = {"view": DEFAULT_VIEW, "distance": CAMERA_PRESETS[DEFAULT_VIEW][0][0],
                 "yaw": CAMERA_PRESETS[DEFAULT_VIEW][0][1],
                 "pitch": CAMERA_PRESETS[DEFAULT_VIEW][0][2],
                 "target": CAMERA_PRESETS[DEFAULT_VIEW][0][3]}


def scenario_path(scenario):
    return tuple(scenario.get("path", DEFAULT_PATH))


def scenario_waypoint_labels(scenario):
    return scenario.get("waypoint_labels", DEFAULT_WAYPOINT_LABELS)


def scenario_leg_labels(scenario):
    return tuple(scenario.get("leg_labels", DEFAULT_LEG_LABELS))


def scenario_legs(scenario):
    """返回 [(段名, 起点键, 终点键), ...]。"""
    path = scenario_path(scenario)
    return [(f"{path[index]}->{path[index + 1]}", path[index], path[index + 1])
            for index in range(len(path) - 1)]


def scenario_waypoint_color(scenario, index):
    palette = WAYPOINT_COLOR_PALETTE
    return palette[index % len(palette)]


def scenario_leg_color(scenario, index):
    palette = WAYPOINT_COLOR_PALETTE
    return palette[(index + 1) % len(palette)]


def scenario_obstacle_labels(scenario):
    labels = scenario.get("obstacle_labels")
    if labels:
        return tuple(labels)
    return tuple(f"OBS-{index + 1}" for index in range(len(scenario["obstacles"])))


def obstacle_world_bounds(obstacle):
    """AABB 由 DH 系给到世界系：x/y 取反并交换上下界。"""
    minimum = np.asarray(obstacle["minimum"], dtype=float)
    maximum = np.asarray(obstacle["maximum"], dtype=float)
    return (np.array([-maximum[0], -maximum[1], minimum[2]]),
            np.array([-minimum[0], -minimum[1], maximum[2]]))


def apply_view(name):
    """按预设摆放调试相机，并记住参数供文本排版换算。"""
    preset, note = CAMERA_PRESETS[name]
    distance, yaw, pitch, target = preset
    ACTIVE_CAMERA.update({"view": name, "distance": distance, "yaw": yaw,
                          "pitch": pitch, "target": target})
    p.resetDebugVisualizerCamera(distance, yaw, pitch, list(target))
    return note


def scaled_text_size(base):
    return base * ACTIVE_CAMERA["distance"] / TEXT_SIZE_REFERENCE_DISTANCE


def draw_box_outline(low, high, color, width=2.5, z=FLOOR_Z):
    corners = [np.array([x, y, z]) for x in (low[0], high[0]) for y in (low[1], high[1])]
    for start, end in ((0, 1), (1, 3), (3, 2), (2, 0)):
        p.addUserDebugLine(corners[start], corners[end], list(color), width, 0.0)


def setup_visualization(scenario, view=DEFAULT_VIEW):
    """GUI 模式
    """
    note = apply_view(view)
    print(f"界面视角：{view}（{note}）", flush=True)
    path = scenario_path(scenario)
    labels = scenario_waypoint_labels(scenario)
    keys = list(dict.fromkeys(path))
    waypoints = {key: world_pose_of(np.asarray(scenario[key], dtype=float))[:3, 3]
                 for key in keys}
    # 障碍物：地板轮廓 + 顶端标签
    for index, obstacle in enumerate(scenario["obstacles"]):
        low, high = obstacle_world_bounds(obstacle)
        color = OBSTACLE_COLOR_PALETTE[index % len(OBSTACLE_COLOR_PALETTE)]
        draw_box_outline(low, high, color, 3.0)
        p.addUserDebugLine([(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5, high[2]],
                           [(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5, high[2] + 0.10],
                           list(color), 3.0, 0.0)
        label = scenario_obstacle_labels(scenario)[index]
        p.addUserDebugText(
            f"{label}  {high[0] - low[0]:.2f}x{high[1] - low[1]:.2f}x{high[2] - low[2]:.2f} m",
            [(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5, high[2] + 0.13],
            textColorRGB=list(color), textSize=scaled_text_size(OBSTACLE_TEXT_SIZE),
            lifeTime=0.0)
    # 作业点：垂线 + 标签
    for index, key in enumerate(keys):
        position = waypoints[key]
        color = scenario_waypoint_color(scenario, index)
        p.addUserDebugText(labels.get(key, key),
                           position + np.array([0.0, 0.0, 0.09]),
                           textColorRGB=list(color),
                           textSize=scaled_text_size(WAYPOINT_TEXT_SIZE), lifeTime=0.0)
        p.addUserDebugLine(position,
                           position + np.array([0.0, 0.0, -position[2] + FLOOR_Z]),
                           list(color), 1.5, 0.0)
    # 各段名义路径：空中直线/地面投影一份
    for index, (_, head, tail) in enumerate(scenario_legs(scenario)):
        color = scenario_leg_color(scenario, index)
        p.addUserDebugLine(waypoints[head], waypoints[tail], list(color), 2.0, 0.0)
        floor_head = np.array([waypoints[head][0], waypoints[head][1], FLOOR_Z])
        floor_tail = np.array([waypoints[tail][0], waypoints[tail][1], FLOOR_Z])
        p.addUserDebugLine(floor_head, floor_tail,
                           [channel * NOMINAL_FLOOR_ALPHA for channel in color],
                           1.5, 0.0)
    return waypoints


class TrajectoryTrace:
    """累积已走过的末端轨迹，并批量提交绘制。

    """

    POINT_SIZE = 6.0

    def __init__(self, flush_interval=1.5):
        self.flush_interval = flush_interval
        self.pending = []
        self.floor_pending = []
        self.last_flush = None

    def add(self, point, color, floor=True, now=None):
        point = np.asarray(point, dtype=float)
        self.pending.append((point, [1.0, 1.0, 1.0]))
        if floor:
            self.floor_pending.append((np.array([point[0], point[1], FLOOR_Z]),
                                       [channel * TRACE_FLOOR_ALPHA
                                        for channel in color]))
        if now is None:
            now = time.monotonic()
        if self.last_flush is None:
            self.last_flush = now
        elif now - self.last_flush >= self.flush_interval:
            self.flush(now=now)

    def flush(self, now=None):
        for positions in (self.pending, self.floor_pending):
            if not positions:
                continue
            p.addUserDebugPoints([item[0].tolist() for item in positions],
                                 [item[1] for item in positions],
                                 self.POINT_SIZE, 0.0)
        self.pending.clear()
        self.floor_pending.clear()
        self.last_flush = time.monotonic() if now is None else now


class DemoOverlay:

    def __init__(self, scenario_name, console_interval, mode="实时",
                 status_interval=1.0, mark_rate=0.5, leg_total=4, trace=None):
        self.scenario_name = scenario_name
        self.console_interval = console_interval
        self.mode = mode
        self.status_interval = status_interval
        # 轨迹点按批提交（见 TrajectoryTrace）
        self.trace = trace
        self.leg_color = WAYPOINT_COLOR_PALETTE[0]
        self.mark_interval = 1.0 / mark_rate if mark_rate > 0.0 else 0.0
        self.last_mark = -1.0e9
        self.leg = ""
        self.leg_key = ""
        self.leg_index = 0
        self.leg_total = leg_total
        self.last_console = 0.0
        self.last_status = 0.0
        self.state = -1
        self.error = -1
        self.deviation_item = -1
        self.recorded = mode == "回放"

    def begin_leg(self, index, name, color=None, key=None):
        self.leg_index = index
        self.leg = name
        self.leg_key = key or name
        self.leg_color = list(color) if color is not None else self.leg_color
        self.last_mark = -1.0e9

    def _poll_status(self, client, now):
        if now - self.last_status < self.status_interval:
            return
        self.last_status = now
        try:#应对可能的卡顿
            status = client.request(CMD_STATUS, timeouts=(0.25, 0.5))
        except RuntimeError:
            return
        if status["response_code"] == 0 and len(status["payload"]) >= 2:
            self.state, self.error = struct.unpack_from("<BB", status["payload"])

    def update(self, step, total, sequence, response_code,
               position, velocity, actual, target, round_trip_ms,
               requests, retries, elapsed, client=None):
        now = time.monotonic()
        if client is not None:
            self._poll_status(client, now)
        if self.trace is not None:
            world = DH_FROM_URDF @ actual
            self.trace.add(world, self.leg_color, now=now)
        deviation = float(np.linalg.norm(actual - target))
        if self.mark_interval > 0.0 and now - self.last_mark >= self.mark_interval:
            self.last_mark = now
            self.deviation_item = p.addUserDebugLine(
                (DH_FROM_URDF @ actual).tolist(),
                (DH_FROM_URDF @ target).tolist(), [0.95, 0.25, 0.25], 2.0, 0.0,
                replaceItemUniqueId=self.deviation_item)
        if (self.console_interval > 0.0
                and now - self.last_console >= self.console_interval):
            self.last_console = now
            print(f"[{self.mode}] 段={self.leg} seq={sequence} "
                  f"响应码={response_code} 往返={round_trip_ms:.1f}ms"
                  f"{'（录制值）' if self.recorded else ''} "
                  f"重发={retries}/{requests} "
                  f"| state={self.state} error={self.error} "
                  f"| 位置={self._format(position)} "
                  f"| 速度={self._format(velocity)} "
                  f"| 避让量={deviation * 1000.0:.1f}mm", flush=True)

    @staticmethod
    def _format(values):
        return "[" + ", ".join(f"{value:+.3f}" for value in values) + "]"


class TrajectoryRecorder:
    """把一次 QEMU 闭环的固件逐周期输出收集成可回放的数据。

    """

    def __init__(self, scenario, leg_duration):
        self.scenario = scenario
        self.leg_duration = leg_duration
        self.legs = []
        self.current = None

    def begin_leg(self, name, label, payload_mode, head=None, tail=None):
        self.current = {"name": name, "label": label, "payload_mode": payload_mode,
                        "head": head, "tail": tail, "steps": []}

    def add(self, position, velocity, command, tcp, payload, response,
            round_trip_ms, client):
        sample = [None] * RECORD_STEP_WIDTH
        offset = RECORD_FIELD_OFFSETS
        for key, values in (("q", position), ("v", velocity), ("cmd", command),
                            ("tcp", tcp), ("payload", payload)):
            for index, value in enumerate(values):
                sample[offset[key] + index] = round(float(value), RECORD_DECIMALS)
        sample[offset["seq"]] = int(response["sequence"])
        sample[offset["code"]] = int(response["response_code"])
        sample[offset["rtt_ms"]] = round(float(round_trip_ms), 1)
        sample[offset["requests"]] = int(client.request_count)
        sample[offset["retries"]] = int(client.retry_count)
        self.current["steps"].append(sample)

    def end_leg(self, metrics):
        self.current["metrics"] = {
            key: round(float(metrics[key]), 6)
            for key in ("arrival", "max_deviation", "duration")}
        self.current["metrics"]["steps"] = int(metrics["steps"])
        self.current["metrics"]["collision"] = int(metrics["collision"])
        self.current["metrics"]["state"] = int(metrics["state"])
        self.current["metrics"]["error"] = int(metrics["error"])
        self.legs.append(self.current)
        self.current = None


def save_recording(path, recorder, result, byte_interval):
    """写出录制文件；同一路径再次录制会直接覆盖。"""
    payload = {
        "format": RECORD_FORMAT,
        "version": RECORD_VERSION,
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "scenario": recorder.scenario["name"],
        "leg_duration_s": recorder.leg_duration,
        "control_period_s": DT,
        "settle_steps": SETTLE_STEPS,
        "byte_interval_s": byte_interval,
        "step_fields": list(RECORD_STEP_FIELDS),
        "path": list(scenario_path(recorder.scenario)),
        "waypoint_labels": scenario_waypoint_labels(recorder.scenario),
        "waypoints": {key: list(recorder.scenario[key])
                      for key in dict.fromkeys(scenario_path(recorder.scenario))},
        "obstacles": recorder.scenario["obstacles"],
        "legs": recorder.legs,
        "result": {key: result[key] for key in
                   ("arrival", "place_error", "deviation", "collision", "state",
                    "error", "cycle_time")},
    }
    target = pathlib.Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return target


def load_recording(path):
    data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    if data.get("format") != RECORD_FORMAT:
        raise RuntimeError(f"非搬运任务录制文件: {path}")
    if data.get("step_fields") != list(RECORD_STEP_FIELDS):
        raise RuntimeError(f"录制文件字段与当前版本不一致，请重新录制: {path}")
    if not data.get("path") or any(leg.get("head") is None or leg.get("tail") is None
                                   for leg in data["legs"]):
        raise RuntimeError(f"录制文件损坏: {path}")
    for leg in data["legs"]:
        for sample in leg["steps"]:
            if len(sample) != RECORD_STEP_WIDTH:
                raise RuntimeError(f"录制文件步长不是 {RECORD_STEP_WIDTH}，"
                                   f"请重新录制: {path}")
    sample = data["legs"][0]["steps"][0]
    q_start = RECORD_FIELD_OFFSETS["q"]
    tcp_start = RECORD_FIELD_OFFSETS["tcp"]
    expected = pose_of(sample[q_start:q_start + 6])[:3, 3]
    recorded = np.asarray(sample[tcp_start:tcp_start + 3], dtype=float)
    if float(np.linalg.norm(expected - recorded)) > 0.05:
        raise RuntimeError(f"录制文件字段错位，请重新录制: {path}")
    return data


def reset_to_joint(robot_id, joint_ids, target):
    for index, value in enumerate(target):
        p.resetJointState(robot_id, joint_ids[index], float(value))


def read_sensor(robot_id, joint_ids):
    states = [p.getJointState(robot_id, index) for index in joint_ids]
    return (np.array([state[0] for state in states]),
            np.array([state[1] for state in states]))


def run_leg(client, robot_id, joint_ids, tool_link, torque_limits, obstacle_ids,
            start_joint, end_joint, duration, payload_id=None, hold_payload=None,
            overlay=None, recorder=None):
    """执行一段笛卡尔直线，返回本段指标。"""
    reset_to_joint(robot_id, joint_ids, start_joint)
    position, velocity = read_sensor(robot_id, joint_ids)
    client.request(CMD_SIMULATION_STEP,
                   struct.pack("<12f", *position, *velocity))
    start_matrix = pose_of(start_joint)
    end_matrix = pose_of(end_joint)
    start_pose = pose_tuple(start_matrix)
    end_pose = pose_tuple(end_matrix)
    response = client.request(CMD_CARTESIAN_LINE,
                              cartesian_line_payload(start_pose, end_pose,
                                                     duration, DT))
    if response["response_code"] != 0:
        raise RuntimeError(f"笛卡尔直线被拒绝: {response['response_code']}")

    steps = int(math.ceil(duration / DT))
    maximum_collision = 0
    collision_links = set()
    any_contact = 0
    states = set()
    tracking = []
    best_arrival = float("inf")
    drift_steps = 0
    distance_to_end = float("inf")
    started = time.monotonic()
    for step in range(steps + SETTLE_STEPS):
        position, velocity = read_sensor(robot_id, joint_ids)
        if hold_payload is not None:
            p.resetBasePositionAndOrientation(
                payload_id, hold_payload["position"], hold_payload["orientation"])
        request_started = time.monotonic()
        response = client.request(CMD_SIMULATION_STEP,
                                  struct.pack("<12f", *position, *velocity))
        round_trip_ms = (time.monotonic() - request_started) * 1000.0
        if response["response_code"] != 0 or len(response["payload"]) != 24:
            raise RuntimeError("仿真步进响应无效")
        commands = struct.unpack("<6f", response["payload"])
        for index, (joint_id, command) in enumerate(zip(joint_ids, commands)):
            p.setJointMotorControl2(robot_id, joint_id, p.VELOCITY_CONTROL,
                                    targetVelocity=command,
                                    force=float(torque_limits[index]))
        p.stepSimulation()
        actual = DH_FROM_URDF @ np.asarray(
            p.getLinkState(robot_id, tool_link,
                           computeForwardKinematics=True)[4], dtype=float)
        ratio = min((step + 1) * DT / duration, 1.0)
        reference = start_matrix[:3, 3] + ratio * (end_matrix[:3, 3]
                                                  - start_matrix[:3, 3])
        tracking.append(float(np.linalg.norm(actual - reference)))
        for obstacle_id in obstacle_ids:
            contacts = p.getClosestPoints(robot_id, obstacle_id, 0.0)
            maximum_collision = max(maximum_collision, len(contacts))
            collision_links.update(int(contact[3]) for contact in contacts)
        any_contact = max(any_contact, len(p.getClosestPoints(robot_id, -1, 0.0)))
        if overlay is not None:
            overlay.update(step + 1, steps + SETTLE_STEPS, response["sequence"],
                           response["response_code"], position, velocity, actual,
                           reference, round_trip_ms, client.request_count,
                           client.retry_count, time.monotonic() - started, client)
        if recorder is not None:
            recorded_position, recorded_velocity = read_sensor(robot_id, joint_ids)
            recorder.add(recorded_position, recorded_velocity, commands, actual,
                         p.getBasePositionAndOrientation(payload_id)[0],
                         response, round_trip_ms, client)
        if step + 1 >= steps:
            distance_to_end = float(np.linalg.norm(actual - end_matrix[:3, 3]))
            if distance_to_end < best_arrival:
                best_arrival = distance_to_end
                drift_steps = 0
            else:
                drift_steps += 1
            if (best_arrival <= ARRIVE_EXIT_LIMIT_M
                    or drift_steps >= SETTLE_DRIFT_LIMIT):
                break
    arrival = float(best_arrival) if np.isfinite(best_arrival) else distance_to_end
    status = client.request(CMD_STATUS)
    state, error = struct.unpack_from("<BB", status["payload"])
    states.add((state, error))
    return {"steps": len(tracking), "collision": maximum_collision,
            "collision_links": sorted(collision_links),
            "any_contact": any_contact,
            "state": state, "error": error, "arrival": arrival,
            "max_deviation": float(np.max(tracking)),
            "duration": time.monotonic() - started}


def run_cycle(client, robot_id, joint_ids, tool_link, torque_limits, obstacle_ids,
              payload_id, scenario, leg_duration, overlay=None, recorder=None):
    """执行一次完整作业循环

    """
    path = scenario_path(scenario)
    label_list = scenario_leg_labels(scenario)
    joints = {key: np.asarray(scenario[key], dtype=float) for key in set(path)}
    # 抓取点/放置点都要以世界系交给 PyBullet；固件下发仍用 DH 系。
    pick_joint = joints[path[1]]
    place_joint = joints[path[-2]]
    pick_center = payload_center(world_pose_of(pick_joint))
    place_center = payload_center(world_pose_of(place_joint))
    hold_at_pick = {"position": pick_center.tolist(),
                    "orientation": [0.0, 0.0, 0.0, 1.0]}
    hold_at_place = {"position": place_center.tolist(),
                     "orientation": [0.0, 0.0, 0.0, 1.0]}

    legs = []

    def run_leg_with_args(name, index, head, tail, hold_payload=None,
                          payload_mode=PAYLOAD_ATTACHED):
        label = label_list[index - 1]
        if overlay is not None:
            overlay.begin_leg(index, label,
                              color=scenario_leg_color(scenario, index - 1),
                              key=name)
        if recorder is not None:
            recorder.begin_leg(name, label, payload_mode, head, tail)
        metrics = run_leg(client, robot_id, joint_ids, tool_link, torque_limits,
                          obstacle_ids, joints[head], joints[tail], leg_duration,
                          payload_id, hold_payload, overlay, recorder)
        legs.append({"name": name, "metrics": metrics})
        if recorder is not None:
            recorder.end_leg(metrics)
        if overlay is not None and overlay.trace is not None:
            overlay.trace.flush()

    leg_specs = scenario_legs(scenario)
    run_leg_with_args(leg_specs[0][0], 1, leg_specs[0][1], leg_specs[0][2],
                      hold_at_pick, PAYLOAD_AT_PICK)

    grasp = p.createConstraint(robot_id, tool_link, payload_id, -1, p.JOINT_FIXED,
                               [0.0, 0.0, 0.0], [0.0, 0.0, GRIPPER_OFFSET_M],
                               [0.0, 0.0, 0.0])
    for index, (name, head, tail) in enumerate(leg_specs[1:-1], start=2):
        run_leg_with_args(name, index, head, tail)

    payload_position = np.asarray(p.getBasePositionAndOrientation(
        payload_id)[0], dtype=float)
    p.removeConstraint(grasp)
    place_error = float(np.linalg.norm(payload_position - place_center))

    last_name, last_head, last_tail = leg_specs[-1]
    run_leg_with_args(last_name, len(leg_specs), last_head, last_tail,
                      hold_at_place, PAYLOAD_AT_PLACE)

    return {"legs": legs, "place_error": place_error,
            "expected_legs": len(leg_specs),
            "arrival": max(leg["metrics"]["arrival"] for leg in legs),
            "deviation": max(leg["metrics"]["max_deviation"] for leg in legs),
            "collision": max(leg["metrics"]["collision"] for leg in legs),
            "state": max(leg["metrics"]["state"] for leg in legs),
            "error": max(leg["metrics"]["error"] for leg in legs),
            "cycle_time": sum(leg["metrics"]["duration"] for leg in legs)}


def print_summary(results, leg_duration, repeats, with_obstacles=True):
    print(f"搬运任务闭环结果：每段 {leg_duration:g} s、控制周期 10 ms、"
          f"每场景重复 {repeats} 次"
          + ("" if with_obstacles else "（本轮关闭障碍物，作为 APF 关闭的对照组）"),
          flush=True)
    print("  场景                                  循环  完成段数  到位误差(mm)  "
          "放置精度(mm)  避障偏移(mm)  碰撞  末态/错误码", flush=True)
    summary = {}
    for item in results:
        result = item["result"]
        expected = result.get("expected_legs", len(result["legs"]))
        passed = (len(result["legs"]) == expected and result["collision"] == 0
                  and result["state"] == 2 and result["error"] == 0
                  and result["arrival"] <= ARRIVAL_LIMIT_M
                  and result["place_error"] <= PLACE_LIMIT_M)
        entry = summary.setdefault(item["scenario"], {"runs": 0, "pass": 0,
                                                      "arrival": [], "place": [],
                                                      "deviation": []})
        entry["runs"] += 1
        entry["pass"] += int(passed)
        entry["arrival"].append(result["arrival"])
        entry["place"].append(result["place_error"])
        entry["deviation"].append(result["deviation"])
        print(f"  {item['scenario'][:34]:<34} #{item['repeat']:<4d} "
              f"{len(result['legs'])}/{expected:<7d} "
              f"{result['arrival'] * 1000.0:11.4f}  "
              f"{result['place_error'] * 1000.0:10.4f}  "
              f"{result['deviation'] * 1000.0:11.4f}  "
              f"{result['collision']:4d}  {result['state']}/{result['error']}",
              flush=True)
    for name, entry in summary.items():
        print(f"  [{name}] {entry['pass']}/{entry['runs']} 通过，"
              f"到位误差均值 {np.mean(entry['arrival']) * 1000.0:.4f} mm，"
              f"放置精度均值 {np.mean(entry['place']) * 1000.0:.4f} mm，"
              f"避障偏移均值 {np.mean(entry['deviation']) * 1000.0:.4f} mm",
              flush=True)
    print(f"  判定标准：作业段全部执行、全程 state=RUNNING、error=0、与障碍物无接触，"
          f"且到位误差 ≤ {ARRIVAL_LIMIT_M * 1000.0:.1f} mm、"
          f"抓取-放置精度 ≤ {PLACE_LIMIT_M * 1000.0:.1f} mm", flush=True)


def replay_recording(path, speed, headless, console_interval,
                     view=DEFAULT_VIEW):
    data = load_recording(path)
    leg_duration = float(data["leg_duration_s"])
    path_keys = list(data.get("path")
                     or [name for leg in data["legs"]
                         for name in (leg["head"], leg["tail"])])
    path_keys = list(dict.fromkeys(path_keys))
    scenario = {"name": data["scenario"], "obstacles": data["obstacles"],
                "path": path_keys,
                "waypoint_labels": data.get("waypoint_labels",
                                             DEFAULT_WAYPOINT_LABELS),
                **{key: data["waypoints"][key] for key in path_keys}}
    robot_id, joint_ids, _, _, payload_id = load_scene(data["obstacles"],
                                                       not headless)
    waypoints = {key: np.asarray(data["waypoints"][key], dtype=float)
                 for key in path_keys}
    matrices = {key: pose_of(value) for key, value in waypoints.items()}
    if not headless:
        setup_visualization(scenario, view)
    overlay = None if headless else DemoOverlay(
        data["scenario"], console_interval, mode="回放",
        leg_total=len(data["legs"]), trace=TrajectoryTrace())
    frame_s = DT / max(speed, 1e-6)

    total_steps = sum(len(leg["steps"]) for leg in data["legs"])
    print(f"回放 {path}：场景 {data['scenario']}，录制于 {data['recorded_at']}，"
          f"{len(data['legs'])} 段共 {total_steps} 个控制周期，"
          f"播放速度 {speed:g}x", flush=True)
    started = time.monotonic()
    for index, leg in enumerate(data["legs"], start=1):
        head, tail = leg["head"], leg["tail"]
        start_position = matrices[head][:3, 3]
        end_position = matrices[tail][:3, 3]
        if overlay is not None:
            overlay.begin_leg(index, leg["label"], leg.get("payload_mode"),
                              color=scenario_leg_color(data, index - 1),
                              key=leg["name"])
            overlay.state = leg["metrics"]["state"]
            overlay.error = leg["metrics"]["error"]
        for step, sample in enumerate(leg["steps"]):
            frame_started = time.monotonic()
            offset = RECORD_FIELD_OFFSETS
            position = sample[offset["q"]:offset["q"] + 6]
            velocity = sample[offset["v"]:offset["v"] + 6]
            tcp = np.asarray(sample[offset["tcp"]:offset["tcp"] + 3], dtype=float)
            payload = sample[offset["payload"]:offset["payload"] + 3]
            for joint, value in zip(joint_ids, position):
                p.resetJointState(robot_id, joint, float(value))
            p.resetBasePositionAndOrientation(payload_id, payload,
                                              [0.0, 0.0, 0.0, 1.0])
            p.stepSimulation()
            if overlay is not None:
                ratio = min((step + 1) * DT / leg_duration, 1.0)
                target = start_position + ratio * (end_position - start_position)
                overlay.update(step + 1, len(leg["steps"]),
                               sample[offset["seq"]], sample[offset["code"]],
                               position, velocity, tcp, target,
                               sample[offset["rtt_ms"]],
                               int(sample[offset["requests"]]),
                               int(sample[offset["retries"]]),
                               time.monotonic() - started)
            remaining = frame_s - (time.monotonic() - frame_started)
            if remaining > 0.0:
                time.sleep(remaining)
        metrics = leg["metrics"]
        if overlay is not None and overlay.trace is not None:
            overlay.trace.flush()
        print(f"  {index}/{len(data['legs'])} {leg['label']}: {metrics['steps']} 周期，"
              f"到位误差 {metrics['arrival'] * 1000.0:.4f} mm，"
              f"最大避障偏移 {metrics['max_deviation'] * 1000.0:.3f} mm，"
              f"碰撞 {metrics['collision']}，state={metrics['state']} "
              f"error={metrics['error']}", flush=True)
    result = data["result"]
    print(f"回放结束：末端到位误差 {result['arrival'] * 1000.0:.4f} mm，"
          f"抓取-放置精度 {result['place_error'] * 1000.0:.4f} mm，"
          f"最大避障偏移 {result['deviation'] * 1000.0:.4f} mm，"
          f"碰撞 {result['collision']}，state={result['state']} "
          f"error={result['error']}，用时 {time.monotonic() - started:.1f} s",
          flush=True)
    if headless:
        p.disconnect()
    return result


def main():
    parser = argparse.ArgumentParser(
        description="QEMU + PyBullet 工业搬运任务全流程闭环仿真。--gui 可调出\n"
                    "PyBullet 可视化界面")
    parser.add_argument("--leg-duration", type=float, default=2.0)
    parser.add_argument("--repeats", type=int, default=None,
                        help="每个场景重复次数；默认无头 3 次、展示 1 次")
    parser.add_argument("--scenarios", nargs="+",
                        help="要运行的场景名（S1...S4）",
                        default=[scenario["name"] for scenario in SCENARIOS])
    parser.add_argument("--byte-interval", type=float, default=0.001)
    parser.add_argument("--no-obstacles", action="store_true",
                        help="不下发障碍物，用作 APF 关闭的对照组")
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 界面")
    parser.add_argument("--record", action="store_true",
                        help="跑一遍闭环并写入录制文件")
    parser.add_argument("--replay", action="store_true",
                        help="回放录制文件，不连 QEMU")
    parser.add_argument("--headless", action="store_true",
                        help="不打开 PyBullet 界面")
    parser.add_argument("--record-file", default=DEFAULT_RECORD_FILE,
                        help=f"录制文件路径，默认 {DEFAULT_RECORD_FILE}；重新录制会覆盖，"
                             )
    parser.add_argument("--speed", type=float, default=1.0,
                        help="回放速度倍数，1 为实时、0.5 为半速慢放")
    parser.add_argument("--view", default=DEFAULT_VIEW, choices=list(CAMERA_PRESETS),
                        help="界面视角预设：iso 侧上方俯视、"
                             "top 近俯视、side 低角度侧视")
    parser.add_argument("--console-interval", type=float, default=None,
                        help="终端打印通信状态与位置反馈的间隔秒数；"
                             "默认展示模式 0.5 s、无头模式关闭")
    parser.add_argument("--log-file", default="/tmp/pick_place_task.log")
    parser.add_argument("--rows-cache", default="/tmp/pick_place_rows.json",
                        help="已完成循环的缓存，中断后重跑可跳过它们")
    parser.add_argument("--reset-cache", action="store_true")
    args = parser.parse_args()

    if args.repeats is None:
        args.repeats = 1 if (args.gui or args.record) else 3
    console_interval = args.console_interval
    if console_interval is None:
        console_interval = (0.5 if (args.gui or args.replay) and not args.headless
                            else 0.0)
    if args.replay:
        replay_recording(args.record_file, args.speed, args.headless,
                         console_interval, args.view)
        return
    use_cache = not (args.gui or args.record)

    logging.basicConfig(filename=args.log_file, filemode="w", level=logging.DEBUG,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    requested = [name.lower() for name in args.scenarios]
    selected = [scenario for scenario in SCENARIOS
                if scenario["name"].lower() in requested]
    if not selected:
        raise SystemExit("没有匹配的场景：" + ", ".join(args.scenarios)
                         + "；可选：" + ", ".join(s["name"] for s in SCENARIOS))
    if args.record and len(selected) > 1:
        print(f"提示：--record 只写一个录制文件（{args.record_file}），"
              f"本次选中的 {len(selected)} 个场景会互相覆盖，建议一次只录一个场景。",
              flush=True)
    if args.no_obstacles:
        selected = [dict(scenario, obstacles=[]) for scenario in selected]

    cache = pathlib.Path(args.rows_cache)
    results = []
    if use_cache and not args.reset_cache and cache.exists():
        try:
            results = json.loads(cache.read_text(encoding="utf-8"))
            print(f"复用缓存 {len(results)} 条结果", flush=True)
        except (ValueError, OSError):
            results = []
    done = {(item["scenario"], item["repeat"]) for item in results}

    client = None

    def connect():
        new_client = QemuRobotClient(byte_interval_s=args.byte_interval,
                                     logger=logging.getLogger("qemu.pick_place"),
                                     startup_delay_s=5.0)
        handshake = new_client.request(CMD_STATUS)
        if handshake["response_code"] != 0 or len(handshake["payload"]) != 50:
            new_client.close()
            raise RuntimeError("QEMU 启动握手失败")
        return new_client

    try:
        for scenario in selected:
            robot_id, joint_ids, tool_link, obstacle_ids, payload_id = load_scene(
                scenario["obstacles"], args.gui)
            torque_limits = joint_torque_limits(robot_id, joint_ids)
            if args.gui:
                setup_visualization(scenario, args.view)
            overlay = (DemoOverlay(scenario["name"], console_interval,
                                   leg_total=len(scenario_legs(scenario)),
                                   trace=TrajectoryTrace())
                       if args.gui else None)
            for repeat in range(1, args.repeats + 1):
                if (scenario["name"], repeat) in done:
                    continue
                recorder = (TrajectoryRecorder(scenario, args.leg_duration)
                            if args.record else None)
                for attempt in range(1, QEMU_RESTART_ATTEMPTS + 1):
                    try:
                        if client is None:
                            client = connect()
                        response = client.request(
                            CMD_SET_OBSTACLES,
                            obstacles_payload(scenario["obstacles"]))
                        if response["response_code"] != 0:
                            raise RuntimeError("障碍物下发被拒绝")
                        result = run_cycle(client, robot_id, joint_ids, tool_link,
                                           torque_limits, obstacle_ids, payload_id,
                                           scenario, args.leg_duration, overlay,
                                           recorder)
                        break
                    except RuntimeError as error:
                        print(f"{scenario['name']} 第 {repeat} 次第 {attempt} 次失败，"
                              f"重启 QEMU 后重试：{error}", flush=True)
                        if recorder is not None:
                            recorder = TrajectoryRecorder(scenario,
                                                          args.leg_duration)
                        if client is not None:
                            client.close()
                            client = None
                        if attempt == QEMU_RESTART_ATTEMPTS:
                            raise
                        # 重试
                        robot_id, joint_ids, tool_link, obstacle_ids, payload_id = (
                            reload_scene(scenario["obstacles"], args.gui))
                        torque_limits = joint_torque_limits(robot_id, joint_ids)
                        if args.gui:
                            setup_visualization(scenario, args.view)
                            overlay = DemoOverlay(
                                scenario["name"], console_interval,
                                leg_total=len(scenario_legs(scenario)),
                                trace=TrajectoryTrace())
                if args.record:
                    saved = save_recording(args.record_file, recorder, result,
                                           args.byte_interval)
                    print(f"已写入录制文件 {saved}（场景 {scenario['name']}，"
                          f"回放：--replay --record-file {saved}）", flush=True)
                results.append({"scenario": scenario["name"], "repeat": repeat,
                                "result": result})
                if use_cache:
                    cache.write_text(json.dumps(results, ensure_ascii=False),
                                     encoding="utf-8")
                print_summary(results, args.leg_duration, args.repeats,
                              not args.no_obstacles)
                for leg in result["legs"]:
                    metrics = leg["metrics"]
                    print(f"  [{scenario['name']} #{repeat}] {leg['name']:>12}: "
                          f"步数={metrics['steps']:>4} "
                          f"到位误差={metrics['arrival'] * 1000:.4f}mm "
                          f"最大避障偏移={metrics['max_deviation'] * 1000:.3f}mm "
                          f"碰撞={metrics['collision']} "
                          f"全物体接触={metrics['any_contact']} "
                          f"状态={metrics['state']}/{metrics['error']}", flush=True)
            if p.isConnected():
                p.disconnect()
    finally:
        if client is not None:
            client.close()
        if p.isConnected():
            p.disconnect()
    print_summary(results, args.leg_duration, args.repeats, not args.no_obstacles)


if __name__ == "__main__":
    main()
