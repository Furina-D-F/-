import os
import pathlib
import select
import struct
import subprocess
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from protocol import CMD_MOTION, CMD_STATUS, FRAME_COMMAND, FrameParser, encode

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
RESPONSE_TIMEOUT_S = 3.0
STARTUP_DELAY_S = 1.0
MAX_BOOT_ATTEMPTS = 3

QEMU_COMMAND = [
    "qemu-system-arm",
    "-M", "mps2-an386",
    "-kernel", str(PROJECT_ROOT / "firmware/build/robot_firmware.elf"),
    "-nographic",
    "-serial", "stdio",
    "-monitor", "none",
    "-no-reboot",
    "-no-shutdown",
]


def read_frame(process):
    data = bytearray()
    deadline = time.monotonic() + RESPONSE_TIMEOUT_S
    while len(data) < 11:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise RuntimeError("QEMU response header timeout")
        ready, _, _ = select.select([process.stdout], [], [], remaining)
        if not ready:
            continue
        chunk = os.read(process.stdout.fileno(), 11 - len(data))
        if not chunk:
            raise RuntimeError("QEMU exited before response header")
        data.extend(chunk)

    payload_length = struct.unpack_from("<H", data, 4)[0]
    while len(data) < 11 + payload_length:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise RuntimeError("QEMU response payload timeout")
        ready, _, _ = select.select([process.stdout], [], [], remaining)
        if not ready:
            continue
        chunk = os.read(process.stdout.fileno(), 11 + payload_length - len(data))
        if not chunk:
            raise RuntimeError("QEMU exited before complete response")
        data.extend(chunk)

    frames = FrameParser().feed(bytes(data))
    if len(frames) != 1:
        raise RuntimeError("QEMU returned an invalid protocol frame")
    return frames[0]


def request(process, sequence, command, payload=b"", byte_interval_s=0.0):
    frame = encode(FRAME_COMMAND, sequence, command, payload=payload)
    for byte in frame:
        process.stdin.write(bytes((byte,)))
        process.stdin.flush()
        if byte_interval_s > 0.0:
            time.sleep(byte_interval_s)
    response = read_frame(process)
    if response["sequence"] != sequence or response["command"] != command:
        raise RuntimeError("QEMU response does not match request")
    if response["response_code"] != 0:
        raise RuntimeError(f"QEMU rejected command {command}: {response['response_code']}")
    return response


def assert_status(process, sequence, byte_interval_s):
    status = request(process, sequence, CMD_STATUS, byte_interval_s=byte_interval_s)
    if len(status["payload"]) != 50:
        raise RuntimeError("QEMU returned an invalid STATUS payload")
    return status


def run_case_once(byte_interval_s):
    process = subprocess.Popen(
        QEMU_COMMAND,
        cwd=PROJECT_ROOT,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        time.sleep(STARTUP_DELAY_S)
        assert_status(process, 1, byte_interval_s)

        motion_payload = struct.pack(
            "<BB8f", 0, 0x03, 0.6, -0.4, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        request(process, 2, CMD_MOTION, motion_payload, byte_interval_s)
        time.sleep(0.2)
        status = assert_status(process, 3, byte_interval_s)
        _, error, *values = struct.unpack("<BB6f6f", status["payload"])
        if error != 0 or values[0] <= 0.0 or values[1] >= 0.0:
            raise RuntimeError("QEMU motion feedback did not advance as expected")

        stop_payload = struct.pack(
            "<BB8f", 1, 0x03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0
        )
        request(process, 4, CMD_MOTION, stop_payload, byte_interval_s)
        assert_status(process, 5, byte_interval_s)
    finally:
        process.terminate()
        process.wait(timeout=2)


def run_case(case_index, byte_interval_s):
    last_error = None
    for boot_attempt in range(1, MAX_BOOT_ATTEMPTS + 1):
        try:
            run_case_once(byte_interval_s)
            print(
                f"cold boot {case_index:02d}: PASS "
                f"attempt={boot_attempt} interval={byte_interval_s:.3f}s"
            )
            return boot_attempt
        except RuntimeError as error:
            last_error = error
    raise RuntimeError(
        f"cold boot {case_index:02d} failed after {MAX_BOOT_ATTEMPTS} attempts: "
        f"{last_error}"
    )


def main():
    total_boot_attempts = 0
    for case_index in range(1, 13):
        total_boot_attempts += run_case(case_index, 0.020)
    print(
        "qemu startup stability: 12/12 cold boots passed "
        f"using {total_boot_attempts} QEMU launches"
    )


if __name__ == "__main__":
    main()
