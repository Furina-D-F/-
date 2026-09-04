import pathlib
import os
import select
import struct
import subprocess
import time

from protocol import FRAME_COMMAND, FrameParser, encode

QEMU_RESPONSE_TIMEOUT = 10.0
QEMU_BYTE_INTERVAL = 0.02


class QemuRobotClient:
    def __init__(self):
        project_root = pathlib.Path(__file__).resolve().parents[2]
        command = [
            "qemu-system-arm",
            "-M", "mps2-an386",
            "-kernel", str(project_root / "firmware" / "build" / "robot_firmware.elf"),
            "-nographic",
            "-serial", "stdio",
            "-monitor", "none",
            "-no-reboot",
            "-no-shutdown",
        ]
        self.process = subprocess.Popen(
            command,
            cwd=project_root,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.parser = FrameParser()
        self.sequence = 1
        time.sleep(1.0)

    def close(self):
        if self.process.stdin is not None:
            self.process.stdin.close()
        self.process.terminate()
        self.process.wait(timeout=2)

    def request(self, command, payload=b""):
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = encode(FRAME_COMMAND, sequence, command, payload=payload)
        for byte in frame:
            self.process.stdin.write(bytes((byte,)))
            self.process.stdin.flush()
            time.sleep(QEMU_BYTE_INTERVAL)

        data = bytearray()
        while len(data) < 11:
            ready, _, _ = select.select([self.process.stdout], [], [], QEMU_RESPONSE_TIMEOUT)
            if not ready:
                raise RuntimeError("QEMU 响应超时")
            chunk = os.read(self.process.stdout.fileno(), 11 - len(data))
            if not chunk:
                raise RuntimeError("QEMU 在返回响应头前退出")
            data.extend(chunk)
        payload_length = struct.unpack_from("<H", data, 4)[0]
        if payload_length > 128:
            raise RuntimeError(f"QEMU 返回非法负载长度: {payload_length}")
        while len(data) < 11 + payload_length:
            ready, _, _ = select.select([self.process.stdout], [], [], QEMU_RESPONSE_TIMEOUT)
            if not ready:
                raise RuntimeError("QEMU 响应负载超时")
            chunk = os.read(
                self.process.stdout.fileno(), 11 + payload_length - len(data)
            )
            if not chunk:
                raise RuntimeError("QEMU 在返回完整响应前退出")
            data.extend(chunk)
        frames = self.parser.feed(bytes(data))
        if len(frames) != 1:
            raise RuntimeError("QEMU 返回了非法协议帧")
        return frames[0]
