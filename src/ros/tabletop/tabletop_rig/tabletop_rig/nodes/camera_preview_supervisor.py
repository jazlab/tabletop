"""Run one isolated preview process for each FLIR camera."""

import signal
import subprocess
import sys
import time
from pathlib import Path

from camera_preview import CAMERA_NAMES


def main() -> int:
    preview_script = Path(__file__).with_name("camera_preview.py")
    processes = [
        subprocess.Popen(
            [
                sys.executable,
                str(preview_script),
                "--ros-args",
                "-r",
                f"__node:=preview_{camera_name}",
                "-p",
                f"camera_name:={camera_name}",
                "-p",
                "rate_hz:=10.0",
                "-p",
                "max_width:=640",
                "-p",
                f"publish_color:={'true' if camera_name == 'left_back_top_cam' else 'false'}",
            ]
        )
        for camera_name in CAMERA_NAMES
    ]

    stopping = False

    def stop_children(_signum=None, _frame=None) -> None:
        nonlocal stopping
        if stopping:
            return
        stopping = True
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop_children)
    signal.signal(signal.SIGTERM, stop_children)

    try:
        while not stopping:
            failed = next(
                (
                    process
                    for process in processes
                    if process.poll() is not None
                ),
                None,
            )
            if failed is not None:
                stop_children()
                break
            time.sleep(0.25)
    finally:
        stop_children()
        for process in processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()

    return max((process.returncode or 0) for process in processes)


if __name__ == "__main__":
    raise SystemExit(main())
