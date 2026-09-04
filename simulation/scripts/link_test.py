import pathlib
import subprocess
import struct
import tempfile

from protocol import (
    CMD_STATUS,
    FRAME_COMMAND,
    FRAME_RESPONSE,
    FrameParser,
    STATUS_OK,
    encode,
)


def start_firmware_bridge():
    project_root = pathlib.Path(__file__).resolve().parents[2]
    source_files = [
        project_root / "firmware" / "tests" / "communication_link_host.c",
        project_root / "firmware" / "app" / "control.c",
        project_root / "firmware" / "drivers" / "communication.c",
        project_root / "firmware" / "drivers" / "joint_motor.c",
        project_root / "firmware" / "drivers" / "protocol.c",
        project_root / "firmware" / "drivers" / "uart.c",
    ]
    include_flags = [
        "-Ifirmware/config", "-Ifirmware/app", "-Ifirmware/drivers",
        "-Ifirmware/third_party/FreeRTOS-Kernel/include",
        "-Ifirmware/third_party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F",
    ]
    with tempfile.TemporaryDirectory() as directory:
        executable = pathlib.Path(directory) / "communication_link_host"
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
             *include_flags, *map(str, source_files), "-o", str(executable)],
            cwd=project_root,
            check=True,
        )
        return subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )


def main():
    firmware = start_firmware_bridge()
    try:
        request = encode(FRAME_COMMAND, 1, CMD_STATUS)
        firmware.stdin.write(request)
        firmware.stdin.flush()
        header = firmware.stdout.read(9)
        payload_length = struct.unpack_from("<H", header, 4)[0]
        response = header + firmware.stdout.read(payload_length + 2)
        response_frame = FrameParser().feed(response)[0]
        assert response_frame["type"] == FRAME_RESPONSE
        assert response_frame["sequence"] == 1
        assert response_frame["command"] == CMD_STATUS
        assert response_frame["response_code"] == STATUS_OK
        print("python -> embedded C -> python: PASS")
    finally:
        firmware.stdin.close()
        firmware.terminate()
        firmware.wait()


if __name__ == "__main__":
    main()