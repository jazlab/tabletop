"""Exercise the actual delayed-launch wiring with harmless Python processes.

No Commander, MoveIt, robot driver or ROS hardware node is started.
"""

import json
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)

LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / "launch/commander.launch.py"
)
GATE_KEYS = (
    "TABLETOP_REQUIRE_FLIR_PREFLIGHT",
    "TABLETOP_REQUIRE_EYELINK_RECORDER",
    "TABLETOP_TASK_LAUNCH_EPOCH",
    "OVERRIDE_LAUNCH_PROCESS_OUTPUT",
)


def test_snapshot_is_independent_of_later_scope_restoration():
    helper = runpy.run_path(str(LAUNCH_FILE))["commander_scoped_environment"]
    context = LaunchContext()
    context.environment.update(
        dict(zip(GATE_KEYS, ("true", "true", "123.5", "screen")))
    )
    captured = helper(context)
    for key in GATE_KEYS:
        context.environment.pop(key)
    assert captured == dict(
        zip(GATE_KEYS, ("true", "true", "123.5", "screen"))
    )
    assert helper(context) == {}  # Standalone Commander remains opt-in.


@pytest.mark.parametrize("recording", ["true", "false"])
def test_actual_commander_event_launch_keeps_task_flags(
    tmp_path, monkeypatch, recording
):
    namespace = runpy.run_path(str(LAUNCH_FILE))
    setup = namespace["launch_setup"]
    globals_ = setup.__globals__
    observed_path = tmp_path / "child-environment.json"
    captured_nodes = []

    class FakeBuilder:
        def __init__(self, **kwargs):
            pass

        def __getattr__(self, name):
            if name == "to_moveit_configs":
                return lambda: SimpleNamespace(to_dict=lambda: {})
            return lambda *args, **kwargs: self

    class HarmlessNode(ExecuteProcess):
        def __init__(self, **kwargs):
            captured_nodes.append(kwargs)
            if kwargs["executable"] == "commander":
                # These are the production Node's actual additional_env values.
                command = (
                    "import json,os; from pathlib import Path; "
                    f"keys={GATE_KEYS!r}; "
                    f"Path({str(observed_path)!r}).write_text(json.dumps("
                    "{k:os.environ.get(k) for k in keys}))"
                )
            else:
                assert kwargs["executable"] == "wait_for_robot_description"
                command = "pass"
            super().__init__(
                cmd=[sys.executable, "-c", command],
                additional_env=kwargs.get("additional_env"),
                output="screen",
            )

    monkeypatch.setitem(globals_, "Node", HarmlessNode)
    monkeypatch.setitem(globals_, "MoveItConfigsBuilder", FakeBuilder)
    monkeypatch.setitem(
        globals_, "save_commander_overrides", lambda _context, path: path
    )
    for key in GATE_KEYS:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "launch-logs"))
    expected = dict(zip(GATE_KEYS, (recording, recording, "123.5", "screen")))

    def scoped_setup(context):
        # Resolve the production defaults while still in the launch scope.
        for argument in namespace["declare_arguments"]():
            argument.execute(context)
        return setup(context)

    group = GroupAction(
        [
            *(
                SetEnvironmentVariable(name, value)
                for name, value in expected.items()
            ),
            OpaqueFunction(function=scoped_setup),
        ],
        scoped=True,
        forwarding=True,
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([group]))
    assert service.run() == 0
    assert observed_path.exists()
    assert json.loads(observed_path.read_text()) == expected
    commander = next(
        node for node in captured_nodes if node["executable"] == "commander"
    )
    assert commander["additional_env"] == expected
    # Confirm the flags really have gone from the restored scope. Without the
    # production fix the delayed child would observe missing values instead.
    assert all(key not in service.context.environment for key in GATE_KEYS)
