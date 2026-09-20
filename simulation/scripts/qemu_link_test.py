import struct
import time

from protocol import CMD_MOTION, CMD_STATUS
from qemu_client import QemuRobotClient

def main():
    client = QemuRobotClient()
    try:
        motion_payload = struct.pack(
            "<BB8f", 0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        motion = client.request(CMD_MOTION, motion_payload)
        if motion["response_code"] != 0:
            raise RuntimeError("QEMU 拒绝 MOTION 指令")

        time.sleep(0.5)
        status = client.request(CMD_STATUS)
        state, error, *values = struct.unpack("<BB6f6f", status["payload"])
        if state != 2 or error != 0 or values[0] <= 0.0:
            raise RuntimeError(
                f"QEMU 状态异常: state={state}, error={error}, position={values[0]}"
            )
        print(f"qemu uart motion/status: PASS position_0={values[0]:.4f}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
