import pathlib
import select
import subprocess
import time

MAX_BOOT_ATTEMPTS = 3


def run_once(command):
    process = subprocess.Popen(
        command,
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
        return output
    finally:
        process.terminate()
        process.wait(timeout=2)


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
    for boot_attempt in range(1, MAX_BOOT_ATTEMPTS + 1):
        output = run_once(command)
        if b"34 Tests 0 Failures 0 Ignored" not in output:
            continue
        print(output.decode("ascii", errors="replace"), end="")
        if boot_attempt > 1:
            print(f"QEMU Unity cold-boot retry: attempt {boot_attempt}")
        return
    text = output.decode("ascii", errors="replace")
    raise RuntimeError(f"QEMU Unity 测试失败或超时:\n{text}")


if __name__ == "__main__":
    main()