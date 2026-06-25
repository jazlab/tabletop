"""Unit tests for the URInterface stop-future lifecycle.

These tests cover the ``stop_program()`` / ``is_ready()`` interaction without a
live robot or dashboard. ``stop_program()`` is called from the Teensy safety
callback to halt the arm; once a stop has *succeeded* it suppresses further
stops until the stop future is cleared, so the dashboard is not flooded while
the operator recovers. The behaviour under test (added in this PR) is that
``is_ready()`` returning ``True`` -- i.e. the program is ``PLAYING`` again,
whether restarted by ``reset()`` or by the operator -- clears that future so the
next safety event can issue a fresh stop.

The interface is instantiated via ``object.__new__`` to bypass the ROS-heavy
``__init__`` (which builds a dozen dashboard service clients); only the handful
of attributes the two methods touch are injected as mocks.

Example:
    pytest tests/ur_stop_future_test.py -v
"""

import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

# Import the nodes package first: tabletop_rig's eager __init__ imports have a
# circular dependency that only resolves when the nodes package is imported
# before any interfaces submodule (mirrors how the runtime entry points load).
import tabletop_rig.nodes  # noqa: F401
from std_srvs.srv import Trigger
from tabletop_rig.interfaces.ur import URInterface
from ur_dashboard_msgs.msg import ProgramState, RobotMode, SafetyMode


def _make_ur() -> URInterface:
    """Build a URInterface with __init__ bypassed and the stop client mocked.

    The mocked ``_stop_client.call_async`` returns a fresh future each call, so
    tests can count how many stop requests were actually issued.
    """
    ur = object.__new__(URInterface)
    ur.log = MagicMock()
    ur._connected = True
    ur._stop_future = None
    ur._stop_client = MagicMock()
    ur._stop_client.call_async.side_effect = lambda *_a, **_k: MagicMock(
        name="stop_future"
    )
    return ur


def _completed_stop_future(success: bool) -> MagicMock:
    """A stop future that reports done with the given success result."""
    fut = MagicMock(name="completed_stop_future")
    fut.done.return_value = True
    fut.exception.return_value = None
    fut.result.return_value = Trigger.Response(success=success, message="")
    return fut


def _make_ready(ur: URInterface) -> None:
    """Configure the async dashboard getters so is_ready() returns True."""
    ur._is_in_remote_control = AsyncMock(return_value=True)
    ur._get_robot_mode = AsyncMock(
        return_value=SimpleNamespace(mode=RobotMode.RUNNING)
    )
    ur._get_program_state = AsyncMock(
        return_value=SimpleNamespace(state=ProgramState.PLAYING)
    )
    ur._get_safety_mode = AsyncMock(
        return_value=SimpleNamespace(mode=SafetyMode.NORMAL)
    )


class TestStopFutureLifecycle:
    """stop_program() suppression and is_ready()-driven re-arming."""

    def test_first_stop_issues_request(self):
        """With no prior future, stop_program() calls the dashboard once."""
        ur = _make_ur()
        ur.stop_program()
        assert ur._stop_client.call_async.call_count == 1
        assert ur._stop_future is not None

    def test_succeeded_stop_is_suppressed(self):
        """A completed, successful stop suppresses a second request."""
        ur = _make_ur()
        ur._stop_future = _completed_stop_future(success=True)
        ur.stop_program()
        ur._stop_client.call_async.assert_not_called()

    def test_pending_stop_is_suppressed(self):
        """An in-flight stop suppresses a duplicate request."""
        ur = _make_ur()
        pending = MagicMock(name="pending_stop_future")
        pending.done.return_value = False
        ur._stop_future = pending
        ur.stop_program()
        ur._stop_client.call_async.assert_not_called()

    def test_failed_stop_is_retried(self):
        """A completed stop that returned success=False is retried."""
        ur = _make_ur()
        ur._stop_future = _completed_stop_future(success=False)
        ur.stop_program()
        assert ur._stop_client.call_async.call_count == 1

    def test_is_ready_clears_stop_future(self):
        """is_ready() == True clears a previously-succeeded stop future."""
        ur = _make_ur()
        ur._stop_future = _completed_stop_future(success=True)
        _make_ready(ur)
        assert asyncio.run(ur.is_ready()) is True
        assert ur._stop_future is None

    def test_is_ready_false_keeps_stop_future(self):
        """is_ready() == False (program not PLAYING) must NOT clear it."""
        ur = _make_ur()
        succeeded = _completed_stop_future(success=True)
        ur._stop_future = succeeded
        _make_ready(ur)
        ur._get_program_state = AsyncMock(
            return_value=SimpleNamespace(state=ProgramState.STOPPED)
        )
        assert asyncio.run(ur.is_ready()) is False
        assert ur._stop_future is succeeded

    def test_stop_rearms_after_is_ready(self):
        """End-to-end: succeeded stop -> suppressed -> is_ready -> fresh stop."""
        ur = _make_ur()

        # First stop request goes out and "succeeds".
        ur.stop_program()
        assert ur._stop_client.call_async.call_count == 1
        ur._stop_future = _completed_stop_future(success=True)

        # While stopped, repeated safety events are suppressed.
        ur.stop_program()
        assert ur._stop_client.call_async.call_count == 1

        # Program is restarted -> is_ready() clears the future.
        _make_ready(ur)
        assert asyncio.run(ur.is_ready()) is True
        assert ur._stop_future is None

        # The next safety event issues a fresh stop.
        ur.stop_program()
        assert ur._stop_client.call_async.call_count == 2
