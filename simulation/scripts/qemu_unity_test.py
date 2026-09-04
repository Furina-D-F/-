import pathlib
import select
import subprocess
import time


def main():
    project_root = pathlib.Path(__file__).resolve().parents[2]
    command = [
        "qemu-system-arm",
        "-M", "mps2-an386",
        "-kernel", str(project_root / "firmware" / "build" / "robot_driver_unity_qemu.elf"),
        "-nographic",
        "-serial", "stdio",
        "-monitor", "none",
        "-no-reboot",
        "-no-shutdown",
    ]
    process = subprocess.Popen(
        command,
        cwd=project_root,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    output = bytearray()
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([process.stdout], [], [], remaining)
            if not ready:
                break
            chunk = process.stdout.read1(4096)
            if not chunk:
                break
            output.extend(chunk)
            if b"Tests" in output and (b"OK\r\n" in output or b"OK\n" in output):
                break
        text = output.decode("ascii", errors="replace")
        if b"16 Tests 0 Failures 0 Ignored" not in output:
            raise RuntimeError(f"QEMU Unity 测试失败或超时:\n{text}")
        print(text, end="")
    finally:
        process.terminate()
        process.wait(timeout=2)


if __name__ == "__main__":
    main()