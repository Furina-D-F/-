import pathlib
import struct
import sys
import time

import pybullet as p
import pybullet_data

from qemu_client import QemuRobotClient
from protocol import CMD_MOTION, CMD_STATUS


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]


def load_robot():
    if p.connect(p.DIRECT) < 0:
        raise RuntimeError("无法连接 PyBullet")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        str(PROJECT_ROOT / "simulation" / "models" / "ur5" / "ur5.urdf"),
        useFixedBase=True,
    )
    joint_ids = [
        index
        for index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, index)[2]
        in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    if len(joint_ids) != 6:
        raise RuntimeError(f"UR5 活动关节数量错误: {joint_ids}")
    return robot_id, joint_ids


def read_status(client):
    frame = client.request(CMD_STATUS)
    state, error, *values = struct.unpack("<BB6f6f", frame["payload"])
    return state, error, values[:6], values[6:]


def main():
    client = QemuRobotClient()
    robot_id, joint_ids = load_robot()
    try:
        target = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
        motion_payload = struct.pack(
            "<BB8f", 0, 1, *target, 1.0, 1.0
        )
        response = client.request(CMD_MOTION, motion_payload)
        if response["response_code"] != 0:
            raise RuntimeError("QEMU 拒绝 MOTION 指令")

        time.sleep(0.5)
        state, error, positions, velocities = read_status(client)
        if state != 2 or error != 0 or positions[0] <= 0.1:
            raise RuntimeError(
                f"控制状态异常: state={state}, error={error}, position={positions[0]}"
            )
        p.stepSimulation()
        for joint_id, position in zip(joint_ids, positions):
            p.resetJointState(robot_id, joint_id, position, 0.0)
        simulation_positions = [
            p.getJointState(robot_id, joint_id)[0] for joint_id in joint_ids
        ]
        position_error = max(abs(a - b) for a, b in zip(positions, simulation_positions))
        if position_error > 1e-5:
            raise RuntimeError(
                f"PyBullet 关节状态与 QEMU STATUS 不一致: "
                f"firmware={positions}, simulation={simulation_positions}, "
                f"error={position_error}"
            )
        if any(abs(value) > 1e-4 for value in positions[1:]):
            raise RuntimeError("非目标关节发生非预期运动")

        stop_payload = struct.pack(
            "<BB8f", 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        stop = client.request(CMD_MOTION, stop_payload)
        if stop["response_code"] != 0:
            raise RuntimeError("QEMU 拒绝 STOP 指令")
        state, error, _, velocities = read_status(client)
        if state != 3 or error != 0 or any(abs(value) > 1e-4 for value in velocities):
            raise RuntimeError("STOP 后状态异常")
        print(
            "qemu -> pybullet integration: PASS "
            f"joint0={positions[0]:.4f}, joints={len(joint_ids)}"
        )
    finally:
        client.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    main()
