import argparse
import time

import pybullet as p
import pybullet_data


def print_joint_states(robot_id):
    states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id)))
    for joint_index, state in enumerate(states):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        print(f"{joint_index}: {joint_name:16s} position={state[0]: .4f} velocity={state[1]: .4f}")


def set_camera(view):
    views = {
        "front": (1.8, -35.0, -20.0),
        "side": (1.8, 55.0, -20.0),
        "top": (2.2, 0.0, -75.0),
    }
    distance, yaw, pitch = views[view]
    p.resetDebugVisualizerCamera(distance, yaw, pitch, [0.0, 0.0, 0.6])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--state-every", type=float, default=1.0)
    args = parser.parse_args()

    connection_mode = p.DIRECT if args.headless else p.GUI
    if p.connect(connection_mode) < 0:
        raise RuntimeError("无法连接 PyBullet")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        "simulation/models/ur5/ur5.urdf",
        basePosition=[0.0, 0.0, 0.0],
        useFixedBase=True,
    )

    movable_joints = [
        index
        for index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]
    print(f"robot id: {robot_id}")
    print(f"joint count: {p.getNumJoints(robot_id)}")
    print(f"movable joints: {movable_joints}")

    if not args.headless:
        set_camera("front")
        print("视角: 1=front, 2=side, 3=top; q=退出")

    last_state_time = 0.0
    start_time = time.monotonic()
    while p.isConnected():
        p.stepSimulation()
        now = time.monotonic()
        if now - last_state_time >= args.state_every:
            print_joint_states(robot_id)
            last_state_time = now

        if not args.headless:
            keyboard = p.getKeyboardEvents()
            if ord("1") in keyboard:
                set_camera("front")
            elif ord("2") in keyboard:
                set_camera("side")
            elif ord("3") in keyboard:
                set_camera("top")
            elif ord("q") in keyboard:
                break
            time.sleep(1 / 240)

        if args.headless and now - start_time >= args.state_every:
            break

    p.disconnect()


if __name__ == "__main__":
    main()