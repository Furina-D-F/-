import math
import os
import pathlib
import select
import struct
import subprocess
import time

from protocol import (
    CMD_CARTESIAN_ARC,
    CMD_CARTESIAN_LINE,
    CMD_MOTION,
    CMD_STATUS,
    FRAME_COMMAND,
    FrameParser,
    cartesian_arc_payload,
    cartesian_line_payload,
    encode,
)

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
RESPONSE_TIMEOUT_S = 30.0
BYTE_INTERVAL_S = 0.02
MAX_ATTEMPTS = 3

QEMU_COMMAND = [
    "qemu-system-arm", "-M", "mps2-an386",
    "-kernel", str(PROJECT_ROOT / "firmware/build/robot_firmware.elf"),
    "-nographic", "-serial", "stdio", "-monitor", "none",
    "-no-reboot", "-no-shutdown",
]


def multiply(left, right):
    return [[sum(left[row][index] * right[index][column] for index in range(4))
             for column in range(4)] for row in range(4)]


def fk(joints):
    parameters = [
        (0.0, 0.089159, math.pi / 2.0),
        (-0.425, 0.0, 0.0),
        (-0.39225, 0.0, 0.0),
        (0.0, 0.10915, math.pi / 2.0),
        (0.0, 0.09465, -math.pi / 2.0),
        (0.0, 0.0823, 0.0),
    ]
    result = [[1.0 if row == column else 0.0 for column in range(4)]
              for row in range(4)]
    for theta, (a, d, alpha) in zip(joints, parameters):
        cosine = math.cos(theta)
        sine = math.sin(theta)
        matrix = [
            [cosine, -sine * math.cos(alpha), sine * math.sin(alpha), a * cosine],
            [sine, cosine * math.cos(alpha), -cosine * math.sin(alpha), a * sine],
            [0.0, math.sin(alpha), math.cos(alpha), d],
            [0.0, 0.0, 0.0, 1.0],
        ]
        result = multiply(result, matrix)
    return result


def quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale
    return (matrix[0][3], matrix[1][3], matrix[2][3], qx, qy, qz, qw)


def read_frame(process):
    data = bytearray()
    deadline = time.monotonic() + RESPONSE_TIMEOUT_S
    while len(data) < 11:
        ready, _, _ = select.select([process.stdout], [], [], max(0.0, deadline - time.monotonic()))
        if not ready:
            raise RuntimeError("QEMU response timeout")
        data.extend(os.read(process.stdout.fileno(), 11 - len(data)))
    payload_length = struct.unpack_from("<H", data, 4)[0]
    while len(data) < 11 + payload_length:
        ready, _, _ = select.select([process.stdout], [], [], max(0.0, deadline - time.monotonic()))
        if not ready:
            raise RuntimeError("QEMU response payload timeout")
        data.extend(os.read(process.stdout.fileno(), 11 + payload_length - len(data)))
    frames = FrameParser().feed(bytes(data))
    if len(frames) != 1:
        raise RuntimeError("invalid QEMU response frame")
    return frames[0]


def request(process, sequence, command, payload=b""):
    frame = encode(FRAME_COMMAND, sequence, command, payload=payload)
    for byte in frame:
        process.stdin.write(bytes((byte,)))
        process.stdin.flush()
        time.sleep(BYTE_INTERVAL_S)
    response = read_frame(process)
    if response["sequence"] != sequence or response["command"] != command:
        raise RuntimeError("response does not match request")
    return response


def run_case(command, payload):
    process = subprocess.Popen(QEMU_COMMAND, cwd=PROJECT_ROOT, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        time.sleep(2.0)
        status = request(process, 1, CMD_STATUS)
        if status["response_code"] != 0:
            raise RuntimeError("initial STATUS failed")
        seed_motion = struct.pack(
            "<BB8f", 0, 0x3F, 0.3, -1.0, 1.0, -1.0, 0.8, 0.2, 1.0, 1.0
        )
        response = request(process, 2, CMD_MOTION, seed_motion)
        if response["response_code"] != 0:
            raise RuntimeError("seed MOTION rejected")
        time.sleep(2.0)
        response = request(process, 3, command, payload)
        if response["response_code"] != 0:
            raise RuntimeError(f"Cartesian command rejected: {response['response_code']}")
        time.sleep(0.05)
        status = request(process, 4, CMD_STATUS)
        state, error = struct.unpack_from("<BB", status["payload"])
        if state != 2 or error != 0:
            raise RuntimeError(f"Cartesian status invalid: state={state}, error={error}")
    finally:
        process.terminate()
        process.wait(timeout=2)


def main():
    start = quaternion(fk([0.3, -1.0, 1.0, -1.0, 0.8, 0.2]))
    end = quaternion(fk([0.35, -1.0, 1.0, -1.0, 0.8, 0.2]))
    line_payload = cartesian_line_payload(start, end, 0.20, 0.01)
    center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0,
              start[2], 0.0, 0.0, 0.0, 1.0)
    arc_payload = cartesian_arc_payload(start, end, center, 1, 0.20, 0.01)
    for command, payload in ((CMD_CARTESIAN_LINE, line_payload),
                             (CMD_CARTESIAN_ARC, arc_payload)):
        last_error = None
        for _ in range(MAX_ATTEMPTS):
            try:
                run_case(command, payload)
                break
            except RuntimeError as error:
                last_error = error
        else:
            raise last_error
    print("qemu Cartesian line/arc: PASS")


if __name__ == "__main__":
    main()
