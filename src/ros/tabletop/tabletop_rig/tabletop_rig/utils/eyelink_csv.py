"""Derived EyeLink CSV jobs, separate from capture finalization and ROS.

The persistent EyeLink node owns one worker, not the short-lived task container.
Jobs use pinned EDF paths, wait for bag metadata (bag closure), and never modify
the capture-integrity report. An interrupted conversion can be retried via CLI.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

LOG = logging.getLogger(__name__)


def write_status(edf: Path, status: str, error: str | None = None) -> None:
    path = edf.parent.parent / "eyelink_conversion.json"
    temporary = path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(
            {
                "status": status,
                "edf_path": str(edf),
                "csv_path": str(edf.with_suffix(".csv")),
                "error": error,
                "updated_epoch": time.time(),
            },
            indent=2,
        )
        + "\n"
    )
    temporary.replace(path)


def convert(edf: Path, converter=None) -> Path:
    """Publish CSV atomically; a failure never changes EDF or a prior CSV."""
    edf = edf.resolve(strict=True)
    if edf.stat().st_size == 0:
        raise ValueError(f"EDF is empty: {edf}")
    write_status(edf, "running")
    try:
        if converter is None:
            from tabletop_py.gaze.edf import edf_to_csv

            converter = edf_to_csv
        with tempfile.TemporaryDirectory(
            prefix=".csv-", dir=edf.parent
        ) as scratch:
            partial = Path(scratch) / edf.with_suffix(".csv").name
            converter(str(edf), output_path=str(partial))
            if not partial.is_file() or partial.stat().st_size == 0:
                raise RuntimeError("Converter produced no CSV data")
            partial.replace(edf.with_suffix(".csv"))
        write_status(edf, "complete")
        return edf.with_suffix(".csv")
    except (Exception, KeyboardInterrupt) as exc:
        write_status(
            edf,
            "interrupted" if isinstance(exc, KeyboardInterrupt) else "error",
            f"{type(exc).__name__}: {exc}",
        )
        raise


class EyeLinkCSVWorker:
    """One low-priority subprocess at a time, outside the EyeLink executor/GIL."""

    def __init__(self) -> None:
        self._jobs: queue.Queue[tuple[Path, bool]] = queue.Queue(maxsize=8)
        self._stopped = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="eyelink-csv-jobs", daemon=True
        )
        self._thread.start()

    def submit(self, edf: str | Path) -> None:
        edf = Path(edf).resolve(strict=True)
        wait_for_bag = (edf.parent.parent / "bag").is_dir()
        write_status(edf, "waiting_bag_close" if wait_for_bag else "queued")
        if self._stopped.is_set():
            write_status(
                edf,
                "interrupted",
                "EyeLink service is shutting down; retry CSV offline",
            )
            return
        try:
            self._jobs.put_nowait((edf, wait_for_bag))
        except queue.Full:
            write_status(
                edf, "error", "CSV queue full; EDF safe, retry CSV offline"
            )

    def close(self) -> None:
        self._stopped.set()
        self._thread.join(timeout=3.0)
        while True:
            try:
                edf, _ = self._jobs.get_nowait()
            except queue.Empty:
                break
            try:
                write_status(
                    edf,
                    "interrupted",
                    "EyeLink service stopped before CSV conversion",
                )
            except OSError:
                LOG.exception("Could not save pending CSV job status")
            finally:
                self._jobs.task_done()

    def _run(self) -> None:
        while not self._stopped.is_set():
            try:
                edf, wait_for_bag = self._jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                metadata = edf.parent.parent / "bag" / "metadata.yaml"
                while wait_for_bag and not metadata.is_file():
                    if self._stopped.wait(0.1):
                        break
                if self._stopped.is_set():
                    write_status(
                        edf,
                        "interrupted",
                        "EyeLink service stopped before CSV conversion",
                    )
                    continue
                environment = os.environ.copy()
                for name in (
                    "OMP_NUM_THREADS",
                    "OPENBLAS_NUM_THREADS",
                    "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS",
                ):
                    environment[name] = "1"
                log_path = edf.parent.parent / "eyelink_conversion.log"
                with log_path.open("a") as log:
                    process = subprocess.Popen(
                        [sys.executable, "-m", __name__, str(edf)],
                        stdin=subprocess.DEVNULL,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        env=environment,
                        start_new_session=True,
                    )
                    while process.poll() is None:
                        if self._stopped.wait(0.1):
                            # The group belongs only to this CSV job and edf2asc.
                            try:
                                os.killpg(process.pid, signal.SIGTERM)
                            except ProcessLookupError:
                                pass
                            try:
                                process.wait(timeout=1.0)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=1.0)
                            write_status(
                                edf,
                                "interrupted",
                                "EyeLink service stopped; EDF safe, retry CSV offline",
                            )
                            break
                    if process.returncode and not self._stopped.is_set():
                        # The child normally writes a specific conversion error.
                        status_path = (
                            edf.parent.parent / "eyelink_conversion.json"
                        )
                        status = json.loads(status_path.read_text())
                        if status.get("status") not in (
                            "error",
                            "interrupted",
                        ):
                            write_status(
                                edf,
                                "error",
                                f"CSV worker exited with code {process.returncode}; see {log_path}",
                            )
            except Exception:
                LOG.exception("CSV job failed; EDF preserved: %s", edf)
                try:
                    write_status(
                        edf,
                        "error",
                        "CSV job failed; see EyeLink service log; EDF safe",
                    )
                except OSError:
                    LOG.exception("Could not save CSV job status")
            finally:
                self._jobs.task_done()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("edf", type=Path)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    # Only this separate process and its edf2asc child get lower priority.
    try:
        os.nice(19)
    except OSError:
        LOG.warning("Could not lower CSV worker CPU priority")
    if io_priority := shutil.which("ionice"):
        subprocess.run(
            [io_priority, "-c", "3", "-p", str(os.getpid())], check=False
        )

    def interrupt(*_args):
        raise KeyboardInterrupt("CSV worker was stopped")

    signal.signal(signal.SIGTERM, interrupt)
    try:
        print(f"CSV saved: {convert(args.edf)}", flush=True)
    except (Exception, KeyboardInterrupt):
        LOG.exception("CSV conversion failed; original EDF is preserved")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
