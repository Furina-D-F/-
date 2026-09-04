import select
import os
import struct
import subprocess
import time

from protocol import CMD_MOTION, CMD_STATUS, FRAME_COMMAND, FrameParser, encode

QEMU_RESPONSE_TIMEOUT = 10.0
QEMU_BYTE_INTERVAL = 0.02


QEMU_COMMAND = [
    "qemu-system-arm",
    "-M", "mps2-an386",
    "-kernel", "firmware/build/robot_firmware.elf",
    "-nographic",
    "-serial", "stdio",
    "-monitor", "none",
    "-no-reboot",
    "-no-shutdown",
]


def read_frame(process):
    data = bytearray()
    while len(data) < 11:
        ready, _, _ = select.select([process.stdout], [], [], QEMU_RESPONSE_TIMEOUT)
        if not ready:
            raise RuntimeError("QEMU 响应超时")
        chunk = os.read(process.stdout.fileno(), 11 - len(data))
        if not chunk:
            raise RuntimeError("QEMU 在返回响应头前退出")
        data.extend(chunk)

    payload_length = struct.unpack_from("<H", data, 4)[0]
    while len(data) < 11 + payload_length:
        ready, _, _ = select.select([process.stdout], [], [], QEMU_RESPONSE_TIMEOUT)
        if not ready:
            raise RuntimeError("QEMU 响应负载超时")
        chunk = os.read(
            process.stdout.fileno(), 11 + payload_length - len(data)
        )
        if not chunk:
            raise RuntimeError("QEMU 在返回完整响应前退出")
        data.extend(chunk)

    frames = FrameParser().feed(bytes(data))
    if len(frames) != 1:
        raise RuntimeError("QEMU 返回了非法协议帧")
    return frames[0]


def request(process, sequence, command, payload=b""):
    frame = encode(FRAME_COMMAND, sequence, command, payload=payload)
    for byte in frame:
        process.stdin.write(bytes((byte,)))
        process.stdin.flush()
        time.sleep(QEMU_BYTE_INTERVAL)
    return read_frame(process)


def main():
    process = subprocess.Popen(
        QEMU_COMMAND,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        time.sleep(1.0)
        motion_payload = struct.pack(
            "<BB8f", 0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        motion = request(process, 1, CMD_MOTION, motion_payload)
        if motion["response_code"] != 0:
            raise RuntimeError("QEMU 拒绝 MOTION 指令")

        time.sleep(0.5)
        status = request(process, 2, CMD_STATUS)
        state, error, *values = struct.unpack("<BB6f6f", status["payload"])
        if state != 2 or error != 0 or values[0] <= 0.0:
            raise RuntimeError(
                f"QEMU 状态异常: state={state}, error={error}, position={values[0]}"
            )
        print(f"qemu uart motion/status: PASS position_0={values[0]:.4f}")
    finally:
        process.terminate()
        process.wait(timeout=2)


if __name__ == "__main__":
    main()
