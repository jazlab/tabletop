"""Launch one RViz configuration on a deterministic noVNC workspace."""

from __future__ import annotations

import argparse
import signal
import subprocess
from pathlib import Path

from tabletop_rig.utils.novnc_workspace import (
    reserve_dwm_workspace,
    wait_for_window_title,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=int, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--log-level", default="WARN")
    parser.add_argument("--window-timeout", type=float, default=60.0)
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    command = [
        "ros2",
        "launch",
        "tabletop_rig",
        "rviz.launch.py",
        f"config_file:={args.config}",
        f"log_level:={args.log_level}",
    ]
    process: subprocess.Popen[str] | None = None

    def forward(signum: int, _frame) -> None:
        if process is not None and process.poll() is None:
            process.send_signal(signum)

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)

    with reserve_dwm_workspace(args.workspace) as switched:
        if not switched:
            print(
                f"WARNING: could not select noVNC workspace {args.workspace}; "
                "RViz will open on the current workspace",
                flush=True,
            )
        process = subprocess.Popen(command, text=True)
        if not wait_for_window_title(
            args.config.name, timeout=args.window_timeout
        ):
            print(
                f"WARNING: RViz window for {args.config.name} did not appear "
                f"within {args.window_timeout:.0f} seconds",
                flush=True,
            )

    return process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
