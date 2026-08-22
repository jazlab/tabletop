"""Regression tests for Flic advertisement reset and suppression."""

import asyncio
from types import SimpleNamespace

import pytest
from scapy.layers.bluetooth import (
    HCI_Cmd_Disconnect,
    HCI_Cmd_LE_Create_Connection,
    HCI_Cmd_LE_Set_Scan_Enable,
)

from tabletop_py.flic.scapy_client import FlicClient


class FakeTransport:
    """Minimal open transport used by the reset scheduler."""

    def is_closing(self) -> bool:
        return False


class RecordingFlicClient(FlicClient):
    """Flic client that simulates HCI completion events."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.events: list[str] = []

    async def send_command(self, cmd):
        if isinstance(cmd, HCI_Cmd_LE_Set_Scan_Enable):
            self.events.append("scan_on" if cmd.enable else "scan_off")
        else:
            self.events.append(type(cmd).__name__)
        return SimpleNamespace(status=0)

    def _send_command(self, cmd):
        assert self._loop is not None
        if isinstance(cmd, HCI_Cmd_LE_Create_Connection):
            self.events.append("connect")
            future = self._pending_connect_futures[str(cmd.paddr).lower()]
            self._loop.call_soon(future.set_result, 7)
        elif isinstance(cmd, HCI_Cmd_Disconnect):
            self.events.append("disconnect")
            future = self._pending_disconnect_futures[int(cmd.handle)]
            self._loop.call_soon(future.set_result, None)
        else:
            raise AssertionError(f"Unexpected command: {type(cmd).__name__}")


def test_reset_matches_validated_scan_connect_hold_disconnect_sequence():
    async def scenario():
        client = RecordingFlicClient(
            kill_delay=0.0,
            kill_hold=0.0,
            button_cooldown=0.05,
        )
        client._loop = asyncio.get_running_loop()
        client._transport = FakeTransport()  # type: ignore[assignment]

        await client._kill_advertising("90:88:a9:50:66:0d", 0)

        assert client.events == [
            "scan_off",
            "connect",
            "disconnect",
            "scan_on",
        ]
        assert client._button_is_suppressed("90:88:a9:50:66:0d")

    asyncio.run(scenario())


def test_first_packet_is_immediate_and_repeats_are_suppressed():
    async def scenario():
        addr = "90:88:a9:50:66:0d"
        report = SimpleNamespace(addr=addr, atype=0)
        client = RecordingFlicClient(
            kill_delay=0.01,
            kill_hold=0.01,
            button_cooldown=0.05,
        )
        client._loop = asyncio.get_running_loop()
        client._transport = FakeTransport()  # type: ignore[assignment]

        first_waiter = asyncio.create_task(client.wait_for_button(addr))
        await asyncio.sleep(0)
        client.on_advertising_report(report, event_time=123.0)

        first = await asyncio.wait_for(first_waiter, timeout=0.005)
        assert first.addr == addr
        assert first.time == 123.0

        reset_task = client._kill_tasks[addr]
        client.on_advertising_report(report, event_time=124.0)
        await reset_task
        await asyncio.sleep(0)

        cooldown_waiter = asyncio.create_task(client.wait_for_button(addr))
        await asyncio.sleep(0)
        client.on_advertising_report(report, event_time=125.0)
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(cooldown_waiter, timeout=0.01)

    asyncio.run(scenario())


def test_validated_reset_timing_defaults_are_preserved():
    client = FlicClient()

    assert client._kill_delay == 1.0
    assert client._kill_hold == 0.2
    assert client._button_cooldown == 0.5
