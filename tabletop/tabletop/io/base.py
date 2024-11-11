"""Base input/output module for the tabletop app."""

import abc
import csv
import threading
import time
from pathlib import Path

from tabletop import common
from tabletop.logger import logger


class BaseIO(abc.ABC):
    """Base input/output class for the tabletop app."""

    def __init__(self, name: str, run_loop_period_ms: float = 100):
        """Initialize the BaseIO class.

        Args:
            name: Name of the I/O module.
            run_loop_period_ms: Number of milliseconds to sleep in each
                iteration of the run loop.
        """
        self._name = name
        self._run_loop_period_seconds = run_loop_period_ms / 1000
        self._state = common.State.PAUSED
        self._log_path = None
        self._last_run_time = time.time()

        # Start thread
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    @abc.abstractmethod
    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def field_names(self) -> list:
        """Return the field names for the I/O module."""
        raise NotImplementedError

    def make_log_path(self, log_dir: Path):
        """Make a log path and initialize csv writer."""
        self._log_path = log_dir / f"{self._name}.csv"
        logger.info(f"\nCreated log path: {self._log_path.name}")
        logger.info(f"Field names: {self.field_names}")

        # Make csv writer
        self._writer_path = open(str(self._log_path), "w", newline="")
        self._writer = csv.DictWriter(
            self._writer_path, fieldnames=self.field_names
        )
        self._writer.writeheader()

    def start(self):
        """Start the I/O module."""
        if self._log_path is None:
            raise ValueError("Cannot start I/O module if log path is None.")
        self._state = common.State.RUNNING

    def pause(self):
        """Pause the I/O module, which will stop logging data."""
        self._state = common.State.PAUSED

    def stop(self):
        """Stop the I/O module and its thread."""
        self._state = common.State.STOPPED
        self._writer_path.close()
        logger.info(f"Closed log path: {self._log_path.name}")
        self._thread.join()

    def _run(self):
        """Run the I/O module."""
        while True:
            # Determine how long to sleep
            since_last_run = time.time() - self._last_run_time
            time.sleep(max(0, self._run_loop_period_seconds - since_last_run))
            self._last_run_time = time.time()

            # Run one iteration of the I/O module loop
            if self._state == common.State.RUNNING:
                data_rows = self._fetch_data()
                self._writer.writerows(data_rows)
            elif self._state == common.State.PAUSED:
                pass
            elif self._state == common.State.STOPPED:
                break
            else:
                raise ValueError(f"Invalid state: {self._state}")
