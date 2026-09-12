"""Task runner for executing experimental tasks from YAML configuration.

This module provides the entry point for running behavioral experiment tasks.
Tasks are dynamically loaded from YAML configuration files that specify
the task class and its initialization parameters.

The run_tasks coroutine is designed to be called by the Commander node
after it has completed initialization. It reads the task configuration,
instantiates each task, and runs them in sequence.

Configuration file format:
    tasks:
      - class: ForagingTask
        kwargs:
          trial_generator:
            class: OrderedChoiceAlternating
            kwargs:
              poses: [...]
              num_trials: 100
          stimulus_duration: 1.0
          delay_duration: 2.0
          ...

Example:
    # Called by Commander coroutine mechanism
    await run_tasks(commander, "/path/to/config.yaml")
"""

import asyncio
import importlib
import os
import time
from copy import deepcopy
from pathlib import Path

import yaml
from tabletop_interfaces.msg import TaskEvent
from tabletop_rig.nodes import Commander

from tabletop_py.utils.common import yaml_dump_string
from tabletop_tasks.tasks.base import BaseTask


def _current_session_id() -> str:
    """Return the recorder session directory name when one is available."""
    bag_root = os.environ.get("ROS_BAG_DIR")
    if bag_root:
        latest = Path(bag_root) / "latest"
        try:
            if latest.exists():
                return latest.resolve().name
        except OSError:
            pass
    return time.strftime("task_%Y-%m-%d_%H-%M-%S")


async def _wait_for_flir_preflight(commander: Commander) -> None:
    """Wait until this launch session has passed camera recording checks."""
    bag_root = os.environ.get("ROS_BAG_DIR")
    if not bag_root:
        raise RuntimeError(
            "ROS_BAG_DIR is required when FLIR recording preflight is enabled"
        )
    report_path = Path(bag_root) / "latest" / "flir_preflight.yaml"
    launch_epoch = float(os.environ.get("TABLETOP_TASK_LAUNCH_EPOCH", "0"))
    timeout_sec = float(
        os.environ.get("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "90")
    )
    deadline = time.monotonic() + timeout_sec

    commander.log(f"Waiting for FLIR recording preflight: {report_path}")
    while time.monotonic() < deadline:
        try:
            is_current = (
                report_path.exists()
                and report_path.stat().st_mtime >= launch_epoch
            )
            if is_current:
                with report_path.open(encoding="utf-8") as file:
                    report = yaml.safe_load(file)
                if report.get("status") != "PASS":
                    failures = "; ".join(report.get("failures", []))
                    raise RuntimeError(f"FLIR preflight failed: {failures}")
                rates = report.get("metadata_rates_hz", {})
                commander.log(
                    "FLIR recording preflight passed: "
                    + ", ".join(
                        f"{camera}={rate:.1f} Hz"
                        for camera, rate in sorted(rates.items())
                    )
                )
                return
        except FileNotFoundError:
            pass
        await asyncio.sleep(0.1)

    raise RuntimeError(
        f"FLIR recording preflight did not pass within {timeout_sec:.1f}s"
    )


async def run_tasks(commander: Commander, config_file: str) -> None:
    """Run experimental tasks from a YAML configuration file.

    Loads task configurations from the specified YAML file and executes
    each task in sequence. Tasks are dynamically instantiated using
    importlib based on the class name specified in the configuration.

    Each task receives the commander instance and any additional keyword
    arguments specified in the configuration.

    Args:
        commander: The Commander node instance for robot and peripheral
            control. Passed to each task's constructor.
        config_file: Path to the YAML configuration file containing
            task definitions.

    Note:
        Tasks are run sequentially - each task's run() method must
        complete before the next task begins. The configuration file
        should contain a 'tasks' key with a list of task definitions,
        each having 'class' and 'kwargs' keys.
    """
    print("Running tasks")
    # A clean post-return pose need not be the named IDLE pose, but it must be
    # collision-free under the normal planning scene. Never attempt an
    # automatic retreat from an unknown mount/presentation state.
    commander.validate_task_start_state()
    if os.environ.get("TABLETOP_REQUIRE_FLIR_PREFLIGHT") == "true":
        await _wait_for_flir_preflight(commander)
    if os.environ.get("TABLETOP_REQUIRE_EYELINK_RECORDER") == "true":
        eyelink_timeout = float(
            os.environ.get("TABLETOP_EYELINK_RECORDER_TIMEOUT", "10")
        )
        commander.log(
            "Waiting for the rosbag recorder to subscribe to EyeLink batches"
        )
        await commander.wait_for_eyelink_recorder(timeout=eyelink_timeout)

    # Load task configuration from YAML file
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    preflight_config = config.pop("manipulation_preflight", None)
    if preflight_config is not None:
        report_name = preflight_config.get(
            "report_name", "manipulation_preflight.yaml"
        )
        preflight = commander.apply_manipulation_preflight(
            report_name=report_name,
            allow_real_use=preflight_config.get("allow_real_use", False),
        )
        if preflight_config.get("exclude_unvalidated", True):
            config = deepcopy(config)
            available = preflight["available_object_ids"]
            for task_config in config["tasks"]:
                generator_kwargs = (
                    task_config.get("kwargs", {})
                    .get("trial_generator", {})
                    .get("kwargs", {})
                )
                grouped_ids = generator_kwargs.get("grouped_object_ids")
                if grouped_ids is None:
                    continue
                for group_name, object_ids in grouped_ids.items():
                    excluded = [x for x in object_ids if x not in available]
                    grouped_ids[group_name] = [
                        x for x in object_ids if x in available
                    ]
                    if excluded:
                        commander.log(
                            f"Task preflight excluded {group_name} objects: "
                            f"{excluded}",
                            severity="WARN",
                        )
                    if not grouped_ids[group_name]:
                        raise RuntimeError(
                            f"Task preflight left no objects for {group_name}"
                        )

    print(f"Tasks config: {yaml_dump_string(config, width=80)}")

    # Enable the reference TTL only for actual task execution. Firmware drives
    # the output LOW immediately when disabled; finally also covers Ctrl-C and
    # task failures.
    await commander.set_sync_pulses(enable=True)
    commander.publish_task_event(
        TaskEvent.TASK_STARTED,
        session_id=_current_session_id(),
    )
    try:
        for task_config in config["tasks"]:
            task: BaseTask = getattr(
                importlib.import_module("tabletop_tasks.tasks"),
                task_config["class"],
            )(commander=commander, **task_config["kwargs"])

            print(f"Running task: {task_config['class']}")
            await task.run()
    finally:
        commander.publish_task_event(TaskEvent.TASK_ENDED)
        await commander.set_sync_pulses(enable=False)
