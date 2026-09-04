import argparse
import struct
import time

import pybullet as p
import pybullet_data

from link_test import start_firmware_bridge
from qemu_client import QemuRobotClient
from protocol import CMD_MOTION, CMD_STATUS, FRAME_COMMAND, FrameParser, encode


STATUS_NAMES = {
    0: "INIT",
    1: "IDLE",
    2: "RUNNING",
    3: "STOPPED",
    4: "ERROR",
}


class RobotClient:
    def __init__(self, use_qemu=False):
        if use_qemu:
            self.backend = QemuRobotClient()
            self.process = None
        else:
            self.backend = None
            self.process = start_firmware_bridge()
        self.parser = FrameParser()
        self.sequence = 1

    def close(self):
        if self.backend is not None:
            self.backend.close()
            return
        self.process.stdin.close()
        self.process.terminate()
        self.process.wait()

    def request(self, command, payload=b""):
        if self.backend is not None:
            return self.backend.request(command, payload)
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        self.process.stdin.write(encode(FRAME_COMMAND, sequence, command, payload=payload))
        self.process.stdin.flush()
        header = self.process.stdout.read(9)
        if len(header) != 9:
            raise RuntimeError("固件 bridge 在返回响应头前退出")
        payload_length = struct.unpack_from("<H", header, 4)[0]
        if payload_length > 128:
            raise RuntimeError(f"固件返回非法负载长度: {payload_length}")
        response = header + self.process.stdout.read(payload_length + 2)
        if len(response) != 11 + payload_length:
            raise RuntimeError("固件 bridge 返回了不完整响应")
        return self.parser.feed(response)[0]

    def motion(self, positions, velocity, acceleration):
        payload = struct.pack(
            "<BB8f", 0, 0x3F, *positions, velocity, acceleration
        )
        return self.request(CMD_MOTION, payload)

    def stop(self):
        payload = struct.pack(
            "<BB8f", 1, 0x3F, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        return self.request(CMD_MOTION, payload)

    def status(self):
        frame = self.request(CMD_STATUS)
        state, error, *values = struct.unpack("<BB6f6f", frame["payload"])
        return state, error, values[:6], values[6:]


class RobotVisualizer:
    def __init__(self):
        if p.connect(p.GUI) < 0:
            raise RuntimeError("无法连接 PyBullet GUI")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(
            "simulation/models/ur5/ur5.urdf",
            basePosition=[0.0, 0.0, 0.0],
            useFixedBase=True,
        )
        self.joint_ids = [
            index
            for index in range(p.getNumJoints(self.robot_id))
            if p.getJointInfo(self.robot_id, index)[2]
            in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
        ]
        p.resetDebugVisualizerCamera(1.8, -35.0, -20.0, [0.0, 0.0, 0.6])

    def update(self, positions):
        for joint_id, position in zip(self.joint_ids, positions):
            p.resetJointState(self.robot_id, joint_id, position, 0.0)
        p.stepSimulation()

    def close(self):
        if p.isConnected():
            p.disconnect()


def print_status(client, visualizer=None):
    state, error, positions, velocities = client.status()
    if visualizer is not None:
        visualizer.update(positions)
    print(
        f"state={STATUS_NAMES.get(state, state)} error={error} "
        f"position={[round(value, 4) for value in positions]} "
        f"velocity={[round(value, 4) for value in velocities]}"
    )


def interactive(client, visualizer=None):
    print("commands: status | move <6 rad> [velocity] [acceleration] | stop | watch <period> [count] | quit")
    while True:
        try:
            line = input("robot> ").strip()
        except EOFError:
            break
        if not line:
            continue
        fields = line.split()
        command = fields[0].lower()
        if command in ("quit", "exit"):
            break
        if command == "status":
            print_status(client, visualizer)
        elif command == "stop":
            print(f"STOP response={client.stop()['response_code']}")
        elif command == "move" and len(fields) in (7, 8, 9):
            positions = [float(value) for value in fields[1:7]]
            velocity = float(fields[7]) if len(fields) >= 8 else 1.0
            acceleration = float(fields[8]) if len(fields) == 9 else 1.0
            print(f"MOTION response={client.motion(positions, velocity, acceleration)['response_code']}")
        elif command == "watch" and len(fields) in (2, 3):
            period = float(fields[1])
            count = int(fields[2]) if len(fields) == 3 else 0
            index = 0
            while count == 0 or index < count:
                print_status(client, visualizer)
                if index + 1 < count or count == 0:
                    time.sleep(period)
                index += 1
        else:
            print("invalid command")


def main():
    parser = argparse.ArgumentParser(description="控制六关节主机仿真驱动")
    parser.add_argument("--target", nargs=6, type=float, metavar="RAD",
                        help="六个关节目标角度")
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--acceleration", type=float, default=1.0)
    parser.add_argument("--watch", type=float, default=0.1,
                        help="状态轮询周期，0 表示只查询一次")
    parser.add_argument("--count", type=int, default=1,
                        help="状态读取次数，0 表示持续读取")
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet GUI 并同步显示关节状态")
    parser.add_argument("--headless", action="store_true",
                        help="兼容无头自动化调用，不打开 PyBullet GUI")
    parser.add_argument("--qemu", action="store_true",
                        help="使用 QEMU 中运行的 ARM 固件作为控制后端")
    args = parser.parse_args()

    client = RobotClient(args.qemu)
    visualizer = RobotVisualizer() if args.gui and not args.headless else None
    try:
        if args.interactive:
            interactive(client, visualizer)
        elif args.stop:
            frame = client.stop()
            print(f"STOP response={frame['response_code']}")
        elif args.target is not None:
            frame = client.motion(args.target, args.velocity, args.acceleration)
            print(f"MOTION response={frame['response_code']}")

        count = 0
        while args.count == 0 or count < args.count:
            print_status(client, visualizer)
            count += 1
            if args.watch <= 0.0 or (args.count != 0 and count >= args.count):
                break
            time.sleep(args.watch)
    finally:
        client.close()
        if visualizer is not None:
            visualizer.close()


if __name__ == "__main__":
    main()
