"""TableTop App."""

import csv
import shutil
import threading
import time
import tkinter
from pathlib import Path

from tabletop import common, influx, io
from tabletop import tasks as tasks_module


class TableTopApp:
    """TableTop App."""

    def __init__(
        self,
        task: tasks_module.BaseTask,
        io_modules: list[io.BaseIO],
        influx_client: influx.Influx,
        base_log_dir: str = "./logs",
        thread_loop_sleep_ms: float = 1000,
    ):
        """Constructor.

        Args:
            task: The task to run. See ./tasks/.
            io_modules: A list of I/O modules. Some of these may also be used by
                the task. See ./io_modules/.
            influx_client: InfluxDB client for writing trial data.
            base_log_dir: Base log directory.
            thread_loop_sleep_ms: Number of milliseconds to sleep in the main
                thread loop.
        """
        self._task = task
        self._io_modules = io_modules
        self._influx_client = influx_client
        self._thread_loop_sleep_seconds = thread_loop_sleep_ms / 1000

        # Initialize state
        self.state = common.State.PAUSED

        # Create logging directory
        self._log_dir = self._make_log_dir(base_log_dir)
        for io_module in self._io_modules:
            io_module.make_log_path(self._log_dir)

        # Create csv writer for task
        self._log_path_task = self._log_dir / "task.csv"
        self._writer_path = open(str(self._log_path_task), "w", newline="")
        self._csv_writer = csv.DictWriter(
            self._writer_path, fieldnames=self._task.field_names
        )
        self._csv_writer.writeheader()

        # Start task run loop in a thread
        self._task_thread = threading.Thread(target=self.run)
        self._task_thread.start()

        # Create and start GUI with three buttons: start, pause, stop
        self._gui = self._make_gui()

    def _make_gui(self) -> tkinter.Tk:
        """Make a tkinter GUI with three buttons: start, pause, stop."""

        # Make the window large and centered
        root = tkinter.Tk()
        root.geometry("900x200")

        # Make three buttons in a row with the following labels: start, pause, stop
        font = ("Helvetica", 30, "bold")
        start_button = tkinter.Button(
            root,
            text="Start",
            command=self._start,
            fg="green",
            font=font,
        )
        pause_button = tkinter.Button(
            root,
            text="Pause",
            command=self._pause,
            fg="orange",
            font=font,
        )
        stop_button = tkinter.Button(
            root,
            text="Stop",
            command=self._stop,
            fg="red",
            font=font,
        )

        # Pack the buttons
        for button in [start_button, pause_button, stop_button]:
            button.config(
                borderwidth=5,
                width=10,
                height=10,
                relief=tkinter.RAISED,
            )
            button.pack(side=tkinter.LEFT, padx=50, pady=10)

        # Start the GUI
        root.mainloop()

        return root

    def _make_log_dir(self, base_log_dir: str) -> Path:
        """Make a log directory.

        This creates a log directory self._base_log_dir/yyyy-mm-dd-hh-mm.

        Args:
            base_log_dir: The base log directory within which to create the log
                directory.

        Returns:
            log_dir: The log directory.
        """
        date = time.strftime("%Y-%m-%d")
        timestamp = time.strftime("%Y-%m-%d-%H-%M")
        log_dir = Path(base_log_dir) / date / timestamp
        logger.info(f"log directory: {log_dir}")

        # If log_dir exists, remove it
        if log_dir.exists():
            logger.info(f"Replacing existing log directory: {log_dir}")
            shutil.rmtree(log_dir)

        # Make log_dir
        log_dir.mkdir(parents=True, exist_ok=True)

        return log_dir

    def _start(self):
        """Start the application."""
        logger.info("\nSwitching to state RUNNING.")
        self.state = common.State.RUNNING
        for io_module in self._io_modules:
            io_module.start()

    def _pause(self):
        """Pause the application."""
        logger.info("\nSwitching to state PAUSED.")
        self.state = common.State.PAUSED
        self._task.finish_trial()
        for io_module in self._io_modules:
            io_module.pause()

    def _stop(self):
        """Stop the application and terminate the program."""
        logger.info("\nSwitching to state STOPPED.")
        self.state = common.State.STOPPED
        self._task.finish_trial()
        for io_module in self._io_modules:
            io_module.stop()
        self._writer_path.close()
        self._task_thread.join()
        exit()

    def run(self):
        """Run the application loop."""
        while True:
            if self.state == common.State.RUNNING:
                logger.info("\nRunning a trial")
                trial_data = self._task.run_trial()
                self._csv_writer.writerow(trial_data)
                self._influx_client.write(trial_data)
                time.sleep(self._thread_loop_sleep_seconds)
            elif self.state == common.State.PAUSED:
                time.sleep(self._thread_loop_sleep_seconds)
            elif self.state == common.State.STOPPED:
                break
            else:
                raise ValueError(f"Invalid state: {self.state}")
