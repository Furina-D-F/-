import pathlib
import logging
import os
import select
import struct
import subprocess
import time

from protocol import CMD_STATUS, FRAME_COMMAND, FrameParser, encode

QEMU_RESPONSE_TIMEOUT = 30.0
QEMU_RESPONSE_TIMEOUTS = (0.3, 0.6, 1.5, 30.0)

QEMU_BYTE_INTERVAL = 0.0002
QEMU_STARTUP_DELAY = 5.0
QEMU_STARTUP_ATTEMPTS = 5
QEMU_REQUEST_ATTEMPTS = 4
QEMU_BYTE_INTERVAL_BACKOFF = 2.0


class QemuRobotClient:
    def __init__(self, byte_interval_s=QEMU_BYTE_INTERVAL, logger=None,
                 gdb_port=None, startup_delay_s=QEMU_STARTUP_DELAY):
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
        if gdb_port is not None:
            command.extend(["-gdb", f"tcp::{gdb_port}"])
        self.byte_interval_s = byte_interval_s
        self.logger = logger or logging.getLogger(__name__)
        self.process = None
        self.request_count = 0
        self.retry_count = 0
        for attempt in range(1, QEMU_STARTUP_ATTEMPTS + 1):
            self.process = subprocess.Popen(
                command,
                cwd=project_root,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.parser = FrameParser()
            self.sequence = 1
            self.logger.info("QEMU start attempt=%d command=%s", attempt,
                             " ".join(command))
            time.sleep(startup_delay_s)
            if self.process.poll() is None:
                try:
                    response = self.request(CMD_STATUS)
                    if response["response_code"] == 0:
                        self.logger.info("QEMU ready: pid=%d byte_interval_s=%.6f "
                                         "startup_delay_s=%.3f", self.process.pid,
                                         self.byte_interval_s, startup_delay_s)
                        return
                except RuntimeError:
                    pass
            self.close()
        raise RuntimeError("QEMU 启动 STATUS 握手连续失败")

    def close(self):
        if self.process is None:
            return
        if self.process.poll() is not None:
            return
        if self.process.stdin is not None:
            self.process.stdin.close()
        self.process.terminate()
        self.process.wait(timeout=2)

    def _drain_stderr(self):
        if self.process.stderr is None:
            return b""
        output = bytearray()
        while True:
            ready, _, _ = select.select([self.process.stderr], [], [], 0.0)
            if not ready:
                break
            chunk = os.read(self.process.stderr.fileno(), 4096)
            if not chunk:
                break
            output.extend(chunk)
        return bytes(output)

    def _failure_message(self, reason):
        stderr = self._drain_stderr().decode("utf-8", "replace").strip()
        parser_buffer = bytes(self.parser.buffer).hex()
        return (f"{reason}; pid={self.process.pid}; returncode={self.process.poll()}; "
            f"stderr={stderr or '<empty>'}; parser_buffer={parser_buffer or '<empty>'}")

    def request(self, command, payload=b"", timeouts=None):
        ladder = QEMU_RESPONSE_TIMEOUTS if timeouts is None else tuple(timeouts)
        if not ladder:
            raise ValueError("超时梯子不能为空")
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        self.request_count += 1
        frame = encode(FRAME_COMMAND, sequence, command, payload=payload)
        for attempt in range(1, len(ladder) + 1):
            if attempt > 1:
                self.retry_count += 1
            self.logger.debug("UART TX attempt=%d seq=%d command=0x%02X payload=%d frame=%s",
                              attempt, sequence, command, len(payload), frame.hex())
            timeout = ladder[attempt - 1]
            if self.byte_interval_s > 0.0:
                interval = self.byte_interval_s * (
                    QEMU_BYTE_INTERVAL_BACKOFF ** (attempt - 1))
                for byte in frame:
                    self.process.stdin.write(bytes((byte,)))
                    self.process.stdin.flush()
                    time.sleep(interval)
            else:
                self.process.stdin.write(frame)
                self.process.stdin.flush()
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                ready, _, _ = select.select([self.process.stdout], [], [], remaining)
                if not ready:
                    break
                chunk = os.read(self.process.stdout.fileno(), 256)
                if not chunk:
                    raise RuntimeError(self._failure_message("QEMU 在返回响应前退出"))
                deadline = time.monotonic() + timeout
                self.logger.debug("UART RX raw=%s", chunk.hex())
                for response in self.parser.feed(chunk):
                    self.logger.debug(
                        "UART RX seq=%d command=0x%02X status=%d payload=%d",
                        response["sequence"], response["command"],
                        response["response_code"], len(response["payload"])
                    )
                    if (response["sequence"] == sequence
                            and response["command"] == command):
                        return response
        raise RuntimeError(self._failure_message(
            f"QEMU 响应超时 seq={sequence} command=0x{command:02X}"))
