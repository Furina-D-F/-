import argparse
import pathlib
import struct
import sys
import time

import pybullet as p
import pybullet_data

from protocol import CMD_CARTESIAN_LINE, CMD_MOTION, CMD_STATUS, cartesian_line_payload
from qemu_cartesian_link_test import fk, quaternion
from qemu_client import QemuRobotClient

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
OBSTACLE_MINIMUM = (-0.64, -0.46, 0.30)
OBSTACLE_MAXIMUM = (-0.58, -0.40, 0.60)
START_JOINT = (0.30, -1.0, 1.0, -1.0, 0.8, 0.2)
END_JOINT = (0.50, -1.0, 1.0, -1.0, 0.8, 0.2)


def load_scene(gui):
    if p.connect(p.GUI if gui else p.DIRECT) < 0:
        raise RuntimeError("无法连接 PyBullet")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        str(PROJECT_ROOT / "simulation" / "models" / "ur5" / "ur5.urdf"),
        useFixedBase=True,
    )
    joint_ids = [
        index for index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    center = tuple((left + right) * 0.5
                   for left, right in zip(OBSTACLE_MINIMUM, OBSTACLE_MAXIMUM))
    half_extents = tuple((right - left) * 0.5
                         for left, right in zip(OBSTACLE_MINIMUM, OBSTACLE_MAXIMUM))
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                 rgbaColor=(0.85, 0.25, 0.08, 0.85))
    obstacle_id = p.createMultiBody(0.0, collision, visual, center)
    start_marker = p.createVisualShape(p.GEOM_SPHERE, radius=0.025,
                                       rgbaColor=(0.1, 0.8, 0.2, 1.0))
    end_marker = p.createVisualShape(p.GEOM_SPHERE, radius=0.025,
                                     rgbaColor=(0.1, 0.3, 1.0, 1.0))
    start_pose = quaternion(fk(START_JOINT))
    end_pose = quaternion(fk(END_JOINT))
    p.createMultiBody(0.0, -1, start_marker, start_pose[:3])
    p.createMultiBody(0.0, -1, end_marker, end_pose[:3])
    if gui:
        p.resetDebugVisualizerCamera(1.7, -35.0, -22.0, [-0.45, -0.30, 0.45])
        p.addUserDebugLine(start_pose[:3], end_pose[:3], [0.7, 0.7, 0.7], 1.0)
    return robot_id, joint_ids, obstacle_id, start_pose, end_pose


def update_robot(robot_id, joint_ids, positions):
    for joint_id, position in zip(joint_ids, positions):
        p.resetJointState(robot_id, joint_id, position, 0.0)
    p.stepSimulation()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    client = QemuRobotClient()
    robot_id, joint_ids, obstacle_id, start_pose, end_pose = load_scene(not args.headless)
    try:
        seed_payload = struct.pack("<BB8f", 0, 0x3F, *START_JOINT, 1.0, 1.0)
        if client.request(CMD_MOTION, seed_payload)["response_code"] != 0:
            raise RuntimeError("起始关节姿态命令被拒绝")
        time.sleep(2.0)
        payload = cartesian_line_payload(start_pose, end_pose, args.duration, 0.01)
        response = client.request(CMD_CARTESIAN_LINE, payload)
        if response["response_code"] != 0:
            raise RuntimeError(f"笛卡尔命令被拒绝: {response['response_code']}")

        start_time = time.monotonic()
        maximum_collision = 0
        last_print = 0.0
        while time.monotonic() - start_time < args.duration + 1.0:
            status = client.request(CMD_STATUS)
            state, error, *values = struct.unpack("<BB6f6f", status["payload"])
            positions = values[:6]
            velocities = values[6:]
            update_robot(robot_id, joint_ids, positions)
            contacts = p.getClosestPoints(robot_id, obstacle_id, 0.0)
            maximum_collision = max(maximum_collision, len(contacts))
            now = time.monotonic()
            if now - last_print >= 0.2:
                print(f"state={state} error={error} collision={len(contacts)} "
                      f"position={[round(value, 3) for value in positions]} "
                      f"velocity={[round(value, 3) for value in velocities]}")
                last_print = now
            if not args.headless:
                time.sleep(0.05)
            else:
                time.sleep(0.02)
        if maximum_collision != 0:
            raise RuntimeError(f"检测到障碍物碰撞: contacts={maximum_collision}")
        print("PyBullet APF static-obstacle scene: PASS")
    finally:
        client.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    main()
