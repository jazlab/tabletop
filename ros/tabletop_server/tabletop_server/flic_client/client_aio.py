"""Flic client library for python

Requires python 3.3 or higher.

For detailed documentation, see the protocol documentation.

Notes on the data type used in this python implementation compared to the protocol documentation:
All kind of integers are represented as python integers.
Booleans use the Boolean type.
Enums use the defined python enums below.
Bd addr are represented as standard python strings, e.g. "aa:bb:cc:dd:ee:ff".
"""

import argparse
import asyncio
import itertools
import logging
import struct
import time
from collections import namedtuple
from dataclasses import dataclass
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class CreateConnectionChannelError(Enum):
    NoError = 0
    MaxPendingConnectionsReached = 1


class ConnectionStatus(Enum):
    Disconnected = 0
    Connected = 1
    Ready = 2


class DisconnectReason(Enum):
    Unspecified = 0
    ConnectionEstablishmentFailed = 1
    TimedOut = 2
    BondingKeysMismatch = 3


class RemovedReason(Enum):
    RemovedByThisClient = 0
    ForceDisconnectedByThisClient = 1
    ForceDisconnectedByOtherClient = 2

    ButtonIsPrivate = 3
    VerifyTimeout = 4
    InternetBackendError = 5
    InvalidData = 6

    CouldntLoadDevice = 7

    DeletedByThisClient = 8
    DeletedByOtherClient = 9
    ButtonBelongsToOtherPartner = 10
    DeletedFromButton = 11


class ClickType(Enum):
    ButtonDown = 0
    ButtonUp = 1
    ButtonClick = 2
    ButtonSingleClick = 3
    ButtonDoubleClick = 4
    ButtonHold = 5


class BdAddrType(Enum):
    PublicBdAddrType = 0
    RandomBdAddrType = 1


class LatencyMode(Enum):
    NormalLatency = 0
    LowLatency = 1
    HighLatency = 2


class BluetoothControllerState(Enum):
    Detached = 0
    Resetting = 1
    Attached = 2


class ScanWizardResult(Enum):
    WizardSuccess = 0
    WizardCancelledByUser = 1
    WizardFailedTimeout = 2
    WizardButtonIsPrivate = 3
    WizardBluetoothUnavailable = 4
    WizardInternetBackendError = 5
    WizardInvalidData = 6
    WizardButtonBelongsToOtherPartner = 7
    WizardButtonAlreadyConnectedToOtherDevice = 8


@dataclass(slots=True, kw_only=True, frozen=True)
class ButtonInfo:
    bd_addr: str
    uuid: str | None
    color: str | None
    serial_number: str | None
    flic_version: int
    firmware_version: int


@dataclass(slots=True, kw_only=True, frozen=True)
class Info:
    bluetooth_controller_state: BluetoothControllerState
    my_bd_addr: str
    my_bd_addr_type: BdAddrType
    max_pending_connections: int
    max_concurrently_connected_buttons: int
    current_pending_connections: int
    currently_no_space_for_new_connection: bool
    bd_addr_of_verified_buttons: tuple[str, ...]


class ButtonConnectionChannel:
    """ButtonConnectionChannel class.

    This class represents a connection channel to a Flic button.
    Add this button connection channel to a FlicClient by executing client.add_connection_channel(connection_channel).
    You may only have this connection channel added to one FlicClient at a time.

    Before you add the connection channel to the client, you should set up your callback functions by assigning
    the corresponding properties to this object with a function. Each callback function has a channel parameter as the first one,
    referencing this object.

    Available properties and the function parameters are:
    on_create_connection_channel_response: channel, error, connection_status
    on_removed: channel, removed_reason
    on_connection_status_changed: channel, connection_status, disconnect_reason
    on_button_up_or_down / on_button_click_or_hold / on_button_single_or_double_click / on_button_single_or_double_click_or_hold: channel, click_type, was_queued, time_diff
    """

    _cnt = itertools.count()

    def __init__(
        self,
        bd_addr: str,
        client: Any,
        latency_mode: LatencyMode = LatencyMode.LowLatency,
        auto_disconnect_time: int = 511,
    ):
        self._conn_id = next(ButtonConnectionChannel._cnt)
        self._bd_addr = bd_addr
        assert isinstance(client, FlicClient)
        self._client = client
        self._latency_mode = latency_mode
        self._auto_disconnect_time = auto_disconnect_time

        self.last_time_button_up_sec = -1
        self.last_time_button_down_sec = -1
        self.last_time_button_click_sec = -1
        self.last_time_button_single_click_sec = -1
        self.last_time_button_double_click_sec = -1
        self.last_time_button_hold_sec = -1

    @property
    def conn_id(self):
        return self._conn_id

    @property
    def bd_addr(self):
        return self._bd_addr

    @property
    def latency_mode(self):
        return self._latency_mode

    @property
    def auto_disconnect_time(self):
        return self._auto_disconnect_time

    @latency_mode.setter
    def latency_mode(self, latency_mode: LatencyMode):
        if self._client is None:
            self._latency_mode = latency_mode
            return

        self._latency_mode = latency_mode
        if not self._client._closed:
            self._client._send_command(
                "CmdChangeModeParameters",
                {
                    "conn_id": self._conn_id,
                    "latency_mode": self._latency_mode,
                    "auto_disconnect_time": self._auto_disconnect_time,
                },
            )

    @auto_disconnect_time.setter
    def auto_disconnect_time(self, auto_disconnect_time: int):
        if self._client is None:
            self._auto_disconnect_time = auto_disconnect_time
            return

        self._auto_disconnect_time = auto_disconnect_time
        if not self._client._closed:
            self._client._send_command(
                "CmdChangeModeParameters",
                {
                    "conn_id": self._conn_id,
                    "latency_mode": self._latency_mode,
                    "auto_disconnect_time": self._auto_disconnect_time,
                },
            )

    def on_create_connection_channel_response(self, error, connection_status):
        logger.info(
            f"Create connection channel response: {error} {connection_status}"
        )

    def on_removed(self, removed_reason):
        logger.info(f"Removed: {removed_reason}")

    def on_connection_status_changed(
        self, connection_status, disconnect_reason
    ):
        disconnect_reason_str = (
            f"disconnect_reason: {disconnect_reason}"
            if connection_status == ConnectionStatus.Disconnected
            else ""
        )

        logger.info(
            f"Connection status changed for {self._bd_addr} | "
            f"connection_status: {connection_status}, " + disconnect_reason_str
        )

    def _on_button(self, evt_type, click_type, was_queued, time_diff):
        logger.info(
            f"{evt_type} | "
            f"addr: {self._bd_addr}, "
            f"type: {click_type}, "
            f"was_queued: {was_queued}, "
            f"time_diff: {time_diff}"
            f"time: {time.time()}"
        )
        match click_type:
            case ClickType.ButtonUp:
                self.last_time_button_up_sec = time.time()
            case ClickType.ButtonDown:
                self.last_time_button_down_sec = time.time()
            case ClickType.ButtonClick:
                self.last_time_button_click_sec = time.time()
            case ClickType.ButtonSingleClick:
                self.last_time_button_single_click_sec = time.time()
            case ClickType.ButtonDoubleClick:
                self.last_time_button_double_click_sec = time.time()
            case ClickType.ButtonHold:
                self.last_time_button_hold_sec = time.time()

    def on_button_up_or_down(self, click_type, was_queued, time_diff):
        self._on_button("Button up or down", click_type, was_queued, time_diff)

    def on_button_click_or_hold(self, click_type, was_queued, time_diff):
        self._on_button(
            "Button click or hold", click_type, was_queued, time_diff
        )

    def on_button_single_or_double_click(
        self, click_type, was_queued, time_diff
    ):
        self._on_button(
            "Button single or double click",
            click_type,
            was_queued,
            time_diff,
        )

    def on_button_single_or_double_click_or_hold(
        self, click_type, was_queued, time_diff
    ):
        self._on_button(
            "Button single or double click or hold",
            click_type,
            was_queued,
            time_diff,
        )


class ButtonScanner:
    """ButtonScanner class.

    Usage:
    scanner = ButtonScanner()
    scanner.on_advertisement_packet = lambda scanner, bd_addr, name, rssi, is_private, already_verified, already_connected_to_this_device, already_connected_to_other_device: ...
    client.add_scanner(scanner)
    """

    _cnt = itertools.count()

    def __init__(self):
        self._scan_id = next(ButtonScanner._cnt)

    @property
    def scan_id(self):
        return self._scan_id

    def on_advertisement_packet(
        self,
        bd_addr,
        name,
        rssi,
        is_private,
        already_verified,
        already_connected_to_this_device,
        already_connected_to_other_device,
    ):
        logger.info(
            f"Received advertisement packet | "
            f"bd_addr: {bd_addr}, "
            f"name: {name}, "
            f"rssi: {rssi}, "
            f"is_private: {is_private}, "
            f"already_verified: {already_verified}, "
            f"already_connected_to_this_device: {already_connected_to_this_device}, "
            f"already_connected_to_other_device: {already_connected_to_other_device}"
        )


class ScanWizard:
    """ScanWizard class

    Usage:
    wizard = ScanWizard()
    wizard.on_found_private_button = lambda scan_wizard: ...
    wizard.on_found_public_button = lambda scan_wizard, bd_addr, name: ...
    wizard.on_button_connected = lambda scan_wizard, bd_addr, name: ...
    wizard.on_completed = lambda scan_wizard, result, bd_addr, name: ...
    client.add_scan_wizard(wizard)
    """

    _cnt = itertools.count()

    def __init__(self):
        self._scan_wizard_id = next(ScanWizard._cnt)
        self._bd_addr: str | None = None
        self._name: str | None = None
        self._cc: ButtonConnectionChannel | None = None
        self._result: ScanWizardResult | None = None
        self.completed = False
        self.completed_event = asyncio.Event()

    @property
    def scan_wizard_id(self):
        return self._scan_wizard_id

    @property
    def bd_addr(self) -> str | None:
        return self._bd_addr

    @property
    def name(self) -> str | None:
        return self._name

    @property
    def cc(self) -> ButtonConnectionChannel | None:
        if not self._completed:
            raise RuntimeError("ScanWizard not completed")
        return self._cc

    @property
    def result(self):
        if not self._completed:
            raise RuntimeError("ScanWizard not completed")
        assert self._result is not None
        return self._result

    def on_found_private_button(self):
        logger.info(
            "Found a private button. Please hold it down for 7 seconds to make it public."
        )

    def on_found_public_button(self, bd_addr: str, name: str):
        self._bd_addr = bd_addr
        self._name = name
        logger.info(
            f"Found public button {bd_addr} ({name}), now connecting..."
        )

    def on_button_connected(self):
        logger.info(
            f"Button {self._bd_addr} ({self._name}) was connected, now verifying..."
        )

    def on_completed(
        self,
        result: ScanWizardResult,
        client: Any,
        latency_mode: LatencyMode,
        auto_disconnect_time: int,
    ):
        logger.info(
            f"Scan wizard completed with result {result} for button {self._bd_addr} ({self._name})."
        )
        if result == ScanWizardResult.WizardSuccess:
            assert self._bd_addr is not None
            logger.info(
                f"Your button is now ready. The bd addr is {self._bd_addr}."
            )
            self._cc = ButtonConnectionChannel(
                self._bd_addr,
                client,
                latency_mode,
                auto_disconnect_time,
            )
        else:
            logger.error(
                f"Scan wizard failed with result {result} for button {self._bd_addr} ({self._name})."
            )
        self._result = result
        self._completed = True
        self.completed_event.set()


class BatteryStatusListener:
    """BatteryStatusListener class

    Usage:
    listener = BatteryStatusListener(bd_addr)
    listener.on_battery_status = lambda battery_status_listener, bd_addr, battery_percentage, timestamp: ...
    client.add_battery_status_listener(listener)
    """

    _cnt = itertools.count()

    def __init__(self, bd_addr: str):
        self._listener_id = next(BatteryStatusListener._cnt)
        self._bd_addr = bd_addr

    @property
    def listener_id(self):
        return self._listener_id

    @property
    def bd_addr(self):
        return self._bd_addr

    def on_battery_status(self, battery_percentage: int, timestamp: int):
        logger.info(f"Battery status: {battery_percentage} at {timestamp}")


class FlicClient(asyncio.Protocol):
    """FlicClient class.

    When this class is constructed, a socket connection is established.
    You may then send commands to the server and set timers.
    Once you are ready with the initialization you must call the handle_events() method which is a main loop that never exits, unless the socket is closed.
    For a more detailed description of all commands, events and enums, check the protocol specification.

    All commands are wrapped in more high level functions and events are reported using callback functions.

    All methods called on this class will take effect only if you eventually call the handle_events() method.

    The ButtonScanner is used to set up a handler for advertisement packets.
    The ButtonConnectionChannel is used to interact with connections to flic buttons and receive their events.

    Other events are handled by the following callback functions that can be assigned to this object (and a list of the callback function parameters):
    on_new_verified_button: bd_addr
    on_no_space_for_new_connection: max_concurrently_connected_buttons
    on_got_space_for_new_connection: max_concurrently_connected_buttons
    on_bluetooth_controller_state_change: state
    """

    _EVENTS = [
        (
            "EvtAdvertisementPacket",
            "<I6s17pb????",
            "scan_id bd_addr name rssi is_private already_verified already_connected_to_this_device already_connected_to_other_device",
        ),
        (
            "EvtCreateConnectionChannelResponse",
            "<IBB",
            "conn_id error connection_status",
        ),
        (
            "EvtConnectionStatusChanged",
            "<IBB",
            "conn_id connection_status disconnect_reason",
        ),
        ("EvtConnectionChannelRemoved", "<IB", "conn_id removed_reason"),
        (
            "EvtButtonUpOrDown",
            "<IBBI",
            "conn_id click_type was_queued time_diff",
        ),
        (
            "EvtButtonClickOrHold",
            "<IBBI",
            "conn_id click_type was_queued time_diff",
        ),
        (
            "EvtButtonSingleOrDoubleClick",
            "<IBBI",
            "conn_id click_type was_queued time_diff",
        ),
        (
            "EvtButtonSingleOrDoubleClickOrHold",
            "<IBBI",
            "conn_id click_type was_queued time_diff",
        ),
        ("EvtNewVerifiedButton", "<6s", "bd_addr"),
        (
            "EvtGetInfoResponse",
            "<B6sBBhBBH",
            "bluetooth_controller_state my_bd_addr my_bd_addr_type max_pending_connections max_concurrently_connected_buttons current_pending_connections currently_no_space_for_new_connection nb_verified_buttons",
        ),
        (
            "EvtNoSpaceForNewConnection",
            "<B",
            "max_concurrently_connected_buttons",
        ),
        (
            "EvtGotSpaceForNewConnection",
            "<B",
            "max_concurrently_connected_buttons",
        ),
        ("EvtBluetoothControllerStateChange", "<B", "state"),
        ("EvtPingResponse", "<I", "ping_id"),
        (
            "EvtGetButtonInfoResponse",
            "<6s16s17p17pBI",
            "bd_addr uuid color serial_number flic_version firmware_version",
        ),
        ("EvtScanWizardFoundPrivateButton", "<I", "scan_wizard_id"),
        (
            "EvtScanWizardFoundPublicButton",
            "<I6s17p",
            "scan_wizard_id bd_addr name",
        ),
        ("EvtScanWizardButtonConnected", "<I", "scan_wizard_id"),
        ("EvtScanWizardCompleted", "<IB", "scan_wizard_id result"),
        ("EvtButtonDeleted", "<6s?", "bd_addr deleted_by_this_client"),
        (
            "EvtBatteryStatus",
            "<Ibq",
            "listener_id battery_percentage timestamp",
        ),
    ]
    _EVENT_STRUCTS = list(map(lambda x: struct.Struct(x[1]), _EVENTS))
    _EVENT_NAMED_TUPLES = list(map(lambda x: namedtuple(x[0], x[2]), _EVENTS))

    _COMMANDS = [
        ("CmdGetInfo", "", ""),
        ("CmdCreateScanner", "<I", "scan_id"),
        ("CmdRemoveScanner", "<I", "scan_id"),
        (
            "CmdCreateConnectionChannel",
            "<I6sBh",
            "conn_id bd_addr latency_mode auto_disconnect_time",
        ),
        ("CmdRemoveConnectionChannel", "<I", "conn_id"),
        ("CmdForceDisconnect", "<6s", "bd_addr"),
        (
            "CmdChangeModeParameters",
            "<IBh",
            "conn_id latency_mode auto_disconnect_time",
        ),
        ("CmdPing", "<I", "ping_id"),
        ("CmdGetButtonInfo", "<6s", "bd_addr"),
        ("CmdCreateScanWizard", "<I", "scan_wizard_id"),
        ("CmdCancelScanWizard", "<I", "scan_wizard_id"),
        ("CmdDeleteButton", "<6s", "bd_addr"),
        ("CmdCreateBatteryStatusListener", "<I6s", "listener_id bd_addr"),
        ("CmdRemoveBatteryStatusListener", "<I", "listener_id"),
    ]

    _COMMAND_STRUCTS = list(map(lambda x: struct.Struct(x[1]), _COMMANDS))
    _COMMAND_NAMED_TUPLES = list(
        map(lambda x: namedtuple(x[0], x[2]), _COMMANDS)
    )
    _COMMAND_NAME_TO_OPCODE = dict((x[0], i) for i, x in enumerate(_COMMANDS))

    @staticmethod
    def _bdaddr_bytes_to_string(bdaddr_bytes):
        return ":".join(map(lambda x: "%02x" % x, reversed(bdaddr_bytes)))

    @staticmethod
    def _bdaddr_string_to_bytes(bdaddr_string):
        return bytearray.fromhex("".join(reversed(bdaddr_string.split(":"))))

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        default_latency_mode: LatencyMode = LatencyMode.LowLatency,
        default_auto_disconnect_time: int = 511,
    ):
        self._loop = loop
        self.default_latency_mode = default_latency_mode
        self.default_auto_disconnect_time = default_auto_disconnect_time
        self._buffer = b""
        self._transport = None

        self._scanners: dict[int, ButtonScanner] = {}
        self._scan_wizards: dict[int, ScanWizard] = {}
        self._connection_channels: dict[int, ButtonConnectionChannel] = {}
        self._bd_addrs: set[str] = set()
        self._battery_status_listeners: dict[int, BatteryStatusListener] = {}

        self._get_button_info_queue: asyncio.Queue[ButtonInfo] = (
            asyncio.Queue()
        )
        self._get_info_queue: asyncio.Queue[Info] = asyncio.Queue()

        self._closed_event = asyncio.Event()

        self.last_time_button_up_sec = -1
        self.last_time_button_down_sec = -1
        self.last_time_button_click_sec = -1
        self.last_time_button_single_click_sec = -1
        self.last_time_button_double_click_sec = -1
        self.last_time_button_hold_sec = -1

    @property
    def num_buttons(self) -> int:
        assert len(self._bd_addrs) == len(self._connection_channels)
        return len(self._connection_channels)

    @property
    def num_scanners(self) -> int:
        return len(self._scanners)

    @property
    def num_scan_wizards(self) -> int:
        return len(self._scan_wizards)

    @property
    def num_battery_status_listeners(self) -> int:
        return len(self._battery_status_listeners)

    ##############################################################
    # asyncio.Protocol methods
    ##############################################################

    def connection_made(self, transport: asyncio.Transport) -> None:
        self._transport = transport

    def data_received(self, data: bytes) -> None:
        cdata = self._buffer + data
        self._buffer = b""
        while len(cdata):
            packet_len = cdata[0] | (cdata[1] << 8)
            packet_len += 2
            if len(cdata) >= packet_len:
                self._dispatch_event(cdata[2:packet_len])
                cdata = cdata[packet_len:]
            else:
                if len(cdata):
                    self._buffer = cdata  # unlikely to happen but.....
                break

    def eof_received(self) -> None:
        assert self._transport is not None
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.close())

    ##############################################################
    # Command sender
    ##############################################################

    def _send_command(self, name, items):
        for key, value in items.items():
            if isinstance(value, Enum):
                items[key] = value.value

        if "bd_addr" in items:
            items["bd_addr"] = FlicClient._bdaddr_string_to_bytes(
                items["bd_addr"]
            )

        opcode = FlicClient._COMMAND_NAME_TO_OPCODE[name]
        data_bytes = FlicClient._COMMAND_STRUCTS[opcode].pack(
            *FlicClient._COMMAND_NAMED_TUPLES[opcode](**items)
        )
        bytes = bytearray(3)
        bytes[0] = (len(data_bytes) + 1) & 0xFF
        bytes[1] = (len(data_bytes) + 1) >> 8
        bytes[2] = opcode
        bytes += data_bytes
        if not self._transport:
            raise RuntimeError(
                "transport not yet connected, cannot send command"
            )
        self._transport.write(bytes)

    ##############################################################
    # Event dispatcher
    ##############################################################

    def _dispatch_event(self, data: bytes):
        if len(data) == 0:
            return
        opcode = data[0]

        if opcode >= len(FlicClient._EVENTS):
            raise ValueError(f"Unknown event opcode: {opcode}")

        event_name = FlicClient._EVENTS[opcode][0]
        data_tuple = FlicClient._EVENT_STRUCTS[opcode].unpack(
            data[1 : 1 + FlicClient._EVENT_STRUCTS[opcode].size]
        )
        items = (
            FlicClient._EVENT_NAMED_TUPLES[opcode]._make(data_tuple)._asdict()
        )

        # Process some kind of items whose data type is not supported by struct
        if "bd_addr" in items:
            items["bd_addr"] = FlicClient._bdaddr_bytes_to_string(
                items["bd_addr"]
            )
        if "name" in items:
            items["name"] = items["name"].decode("utf-8")

        match event_name:
            case "EvtCreateConnectionChannelResponse":
                items["error"] = CreateConnectionChannelError(items["error"])
                items["connection_status"] = ConnectionStatus(
                    items["connection_status"]
                )
            case "EvtConnectionStatusChanged":
                items["connection_status"] = ConnectionStatus(
                    items["connection_status"]
                )
                items["disconnect_reason"] = DisconnectReason(
                    items["disconnect_reason"]
                )
            case "EvtConnectionChannelRemoved":
                items["removed_reason"] = RemovedReason(
                    items["removed_reason"]
                )

            case "EvtGetInfoResponse":
                items["bluetooth_controller_state"] = BluetoothControllerState(
                    items["bluetooth_controller_state"]
                )
                items["my_bd_addr"] = FlicClient._bdaddr_bytes_to_string(
                    items["my_bd_addr"]
                )
                items["my_bd_addr_type"] = BdAddrType(items["my_bd_addr_type"])
                verified_buttons: list[str] = []
                pos = FlicClient._EVENT_STRUCTS[opcode].size
                for _ in range(items["nb_verified_buttons"]):
                    verified_buttons.append(
                        FlicClient._bdaddr_bytes_to_string(
                            data[1 + pos : 1 + pos + 6]
                        )
                    )
                    pos += 6
                items["bd_addr_of_verified_buttons"] = tuple(verified_buttons)

            case "EvtBluetoothControllerStateChange":
                items["state"] = BluetoothControllerState(items["state"])

            case "EvtGetButtonInfoResponse":
                items["uuid"] = "".join(
                    map(lambda x: "%02x" % x, items["uuid"])
                )
                if items["uuid"] == "00000000000000000000000000000000":
                    items["uuid"] = None
                items["color"] = items["color"].decode("utf-8")
                if items["color"] == "":
                    items["color"] = None
                items["serial_number"] = items["serial_number"].decode("utf-8")
                if items["serial_number"] == "":
                    items["serial_number"] = None

            case "EvtScanWizardCompleted":
                items["result"] = ScanWizardResult(items["result"])

            case _ if event_name.startswith("EvtButton"):
                items["click_type"] = ClickType(items["click_type"])

        # Process event
        match event_name:
            case "EvtAdvertisementPacket":
                self.on_advertisement_packet(
                    scan_id=items["scan_id"],
                    bd_addr=items["bd_addr"],
                    name=items["name"],
                    rssi=items["rssi"],
                    is_private=items["is_private"],
                    already_verified=items["already_verified"],
                    already_connected_to_this_device=items[
                        "already_connected_to_this_device"
                    ],
                    already_connected_to_other_device=items[
                        "already_connected_to_other_device"
                    ],
                )

            case "EvtCreateConnectionChannelResponse":
                self.on_create_connection_channel_response(
                    conn_id=items["conn_id"],
                    error=items["error"],
                    connection_status=items["connection_status"],
                )

            case "EvtConnectionStatusChanged":
                self.on_connection_status_changed(
                    conn_id=items["conn_id"],
                    connection_status=items["connection_status"],
                    disconnect_reason=items["disconnect_reason"],
                )

            case "EvtConnectionChannelRemoved":
                self.on_connection_channel_removed(
                    conn_id=items["conn_id"],
                    removed_reason=items["removed_reason"],
                )

            case "EvtButtonUpOrDown":
                self.on_button_up_or_down(
                    conn_id=items["conn_id"],
                    click_type=items["click_type"],
                    was_queued=items["was_queued"],
                    time_diff=items["time_diff"],
                )
            case "EvtButtonClickOrHold":
                self.on_button_click_or_hold(
                    conn_id=items["conn_id"],
                    click_type=items["click_type"],
                    was_queued=items["was_queued"],
                    time_diff=items["time_diff"],
                )
            case "EvtButtonSingleOrDoubleClick":
                self.on_button_single_or_double_click(
                    conn_id=items["conn_id"],
                    click_type=items["click_type"],
                    was_queued=items["was_queued"],
                    time_diff=items["time_diff"],
                )
            case "EvtButtonSingleOrDoubleClickOrHold":
                self.on_button_single_or_double_click_or_hold(
                    conn_id=items["conn_id"],
                    click_type=items["click_type"],
                    was_queued=items["was_queued"],
                    time_diff=items["time_diff"],
                )

            case "EvtNewVerifiedButton":
                self.on_new_verified_button(items["bd_addr"])

            case "EvtButtonDeleted":
                self.on_button_deleted(
                    items["bd_addr"], items["deleted_by_this_client"]
                )

            case "EvtGetInfoResponse":
                self.on_got_info(
                    bluetooth_controller_state=items[
                        "bluetooth_controller_state"
                    ],
                    my_bd_addr=items["my_bd_addr"],
                    my_bd_addr_type=items["my_bd_addr_type"],
                    max_pending_connections=items["max_pending_connections"],
                    max_concurrently_connected_buttons=items[
                        "max_concurrently_connected_buttons"
                    ],
                    current_pending_connections=items[
                        "current_pending_connections"
                    ],
                    currently_no_space_for_new_connection=items[
                        "currently_no_space_for_new_connection"
                    ],
                    bd_addr_of_verified_buttons=items[
                        "bd_addr_of_verified_buttons"
                    ],
                )

            case "EvtGetButtonInfoResponse":
                self.on_got_button_info(
                    bd_addr=items["bd_addr"],
                    uuid=items["uuid"],
                    color=items["color"],
                    serial_number=items["serial_number"],
                    flic_version=items["flic_version"],
                    firmware_version=items["firmware_version"],
                )
            case "EvtNoSpaceForNewConnection":
                self.on_no_space_for_new_connection(
                    max_concurrently_connected_buttons=items[
                        "max_concurrently_connected_buttons"
                    ]
                )

            case "EvtGotSpaceForNewConnection":
                self.on_got_space_for_new_connection(
                    max_concurrently_connected_buttons=items[
                        "max_concurrently_connected_buttons"
                    ]
                )

            case "EvtBluetoothControllerStateChange":
                self.on_bluetooth_controller_state_change(state=items["state"])

            case "EvtScanWizardFoundPrivateButton":
                self.on_scan_wizard_found_private_button(
                    scan_wizard_id=items["scan_wizard_id"]
                )

            case "EvtScanWizardFoundPublicButton":
                self.on_scan_wizard_found_public_button(
                    scan_wizard_id=items["scan_wizard_id"],
                    bd_addr=items["bd_addr"],
                    name=items["name"],
                )

            case "EvtScanWizardButtonConnected":
                self.on_scan_wizard_button_connected(
                    scan_wizard_id=items["scan_wizard_id"]
                )

            case "EvtScanWizardCompleted":
                self.on_scan_wizard_completed(
                    scan_wizard_id=items["scan_wizard_id"],
                    result=items["result"],
                )
            case "EvtBatteryStatus":
                self.on_battery_status(
                    listener_id=items["listener_id"],
                    battery_percentage=items["battery_percentage"],
                    timestamp=items["timestamp"],
                )
            case "EvtPingResponse":
                self.on_ping_response(
                    ping_id=items["ping_id"],
                )
            case _:
                raise ValueError(f"Unknown event: {event_name}")

    ##############################################################
    # Event handlers
    ##############################################################

    def on_no_space_for_new_connection(
        self, max_concurrently_connected_buttons: int
    ):
        logger.warning(
            f"No space for new connection. Max: {max_concurrently_connected_buttons}"
        )

    def on_got_space_for_new_connection(
        self, max_concurrently_connected_buttons: int
    ):
        logger.info(
            f"Got space for new connection. Max: {max_concurrently_connected_buttons}"
        )

    def on_bluetooth_controller_state_change(
        self, state: BluetoothControllerState
    ):
        logger.info(f"Bluetooth controller state changed: {state}")

    def on_new_verified_button(self, bd_addr: str):
        logger.info(f"New verified button {bd_addr}")

    def on_button_deleted(self, bd_addr: str, deleted_by_this_client: bool):
        logger.info(f"Button deleted: {bd_addr} {deleted_by_this_client}")

    def on_scan_wizard_found_private_button(self, scan_wizard_id: int):
        scan_wizard = self._scan_wizards[scan_wizard_id]
        scan_wizard.on_found_private_button()

    def on_scan_wizard_found_public_button(
        self, scan_wizard_id: int, bd_addr: str, name: str
    ):
        scan_wizard = self._scan_wizards[scan_wizard_id]
        scan_wizard.on_found_public_button(bd_addr, name)

    def on_scan_wizard_button_connected(self, scan_wizard_id: int):
        scan_wizard = self._scan_wizards[scan_wizard_id]
        scan_wizard.on_button_connected()

    def on_scan_wizard_completed(
        self,
        scan_wizard_id: int,
        result: ScanWizardResult,
    ):
        scan_wizard = self._scan_wizards[scan_wizard_id]
        scan_wizard.on_completed(
            result=result,
            client=self,
            latency_mode=self.default_latency_mode,
            auto_disconnect_time=self.default_auto_disconnect_time,
        )

    def on_got_info(
        self,
        bluetooth_controller_state: BluetoothControllerState,
        my_bd_addr: str,
        my_bd_addr_type: BdAddrType,
        max_pending_connections: int,
        max_concurrently_connected_buttons: int,
        current_pending_connections: int,
        currently_no_space_for_new_connection: bool,
        bd_addr_of_verified_buttons: tuple[str, ...],
    ):
        logger.info(
            f"Got info: {bluetooth_controller_state}, {my_bd_addr}, {my_bd_addr_type}, {max_pending_connections}, {max_concurrently_connected_buttons}, {current_pending_connections}, {currently_no_space_for_new_connection}, {bd_addr_of_verified_buttons}"
        )

        info = Info(
            bluetooth_controller_state=bluetooth_controller_state,
            my_bd_addr=my_bd_addr,
            my_bd_addr_type=my_bd_addr_type,
            max_pending_connections=max_pending_connections,
            max_concurrently_connected_buttons=max_concurrently_connected_buttons,
            current_pending_connections=current_pending_connections,
            currently_no_space_for_new_connection=currently_no_space_for_new_connection,
            bd_addr_of_verified_buttons=bd_addr_of_verified_buttons,
        )
        self._get_info_queue.put_nowait(info)

    def on_got_button_info(
        self,
        bd_addr: str,
        uuid: str | None,
        color: str | None,
        serial_number: str | None,
        flic_version: int,
        firmware_version: int,
    ):
        logger.info(
            f"Got button info: {bd_addr}, {uuid}, {color}, {serial_number}, {flic_version}, {firmware_version}"
        )

        button_info = ButtonInfo(
            bd_addr=bd_addr,
            uuid=uuid,
            color=color,
            serial_number=serial_number,
            flic_version=flic_version,
            firmware_version=firmware_version,
        )
        self._get_button_info_queue.put_nowait(button_info)

    def on_advertisement_packet(
        self,
        scan_id: int,
        bd_addr: str,
        name: str,
        rssi: int,
        is_private: bool,
        already_verified: bool,
        already_connected_to_this_device: bool,
        already_connected_to_other_device: bool,
    ):
        scanner = self._scanners[scan_id]
        if scanner is not None:
            scanner.on_advertisement_packet(
                bd_addr=bd_addr,
                name=name,
                rssi=rssi,
                is_private=is_private,
                already_verified=already_verified,
                already_connected_to_this_device=already_connected_to_this_device,
                already_connected_to_other_device=already_connected_to_other_device,
            )

    def on_create_connection_channel_response(
        self,
        conn_id: int,
        error: CreateConnectionChannelError,
        connection_status: ConnectionStatus,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_create_connection_channel_response(
            error=error,
            connection_status=connection_status,
        )
        if error != CreateConnectionChannelError.NoError:
            del self._connection_channels[conn_id]
            self._bd_addrs.remove(channel.bd_addr)

    def on_connection_status_changed(
        self,
        conn_id: int,
        connection_status: ConnectionStatus,
        disconnect_reason: DisconnectReason,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_connection_status_changed(
            connection_status=connection_status,
            disconnect_reason=disconnect_reason,
        )

    def _update_button_times(self, click_type, channel):
        match click_type:
            case ClickType.ButtonUp:
                self.last_time_button_up_sec = channel.last_time_button_up_sec
            case ClickType.ButtonDown:
                self.last_time_button_down_sec = (
                    channel.last_time_button_down_sec
                )
            case ClickType.ButtonClick:
                self.last_time_button_click_sec = (
                    channel.last_time_button_click_sec
                )
            case ClickType.ButtonSingleClick:
                self.last_time_button_single_click_sec = (
                    channel.last_time_button_single_click_sec
                )
            case ClickType.ButtonDoubleClick:
                self.last_time_button_double_click_sec = (
                    channel.last_time_button_double_click_sec
                )
            case ClickType.ButtonHold:
                self.last_time_button_hold_sec = (
                    channel.last_time_button_hold_sec
                )

    def on_button_up_or_down(
        self,
        conn_id: int,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_button_up_or_down(
            click_type=click_type,
            was_queued=was_queued,
            time_diff=time_diff,
        )
        self._update_button_times(click_type, channel)

    def on_button_click_or_hold(
        self,
        conn_id: int,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_button_click_or_hold(
            click_type=click_type,
            was_queued=was_queued,
            time_diff=time_diff,
        )
        self._update_button_times(click_type, channel)

    def on_button_single_or_double_click(
        self,
        conn_id: int,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_button_single_or_double_click(
            click_type=click_type,
            was_queued=was_queued,
            time_diff=time_diff,
        )
        self._update_button_times(click_type, channel)

    def on_button_single_or_double_click_or_hold(
        self,
        conn_id: int,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_button_single_or_double_click_or_hold(
            click_type=click_type,
            was_queued=was_queued,
            time_diff=time_diff,
        )
        self._update_button_times(click_type, channel)

    def on_connection_channel_removed(
        self,
        conn_id: int,
        removed_reason: RemovedReason,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_removed(removed_reason=removed_reason)
        del self._connection_channels[conn_id]
        self._bd_addrs.remove(channel.bd_addr)

    def on_battery_status(
        self, listener_id: int, battery_percentage: int, timestamp: int
    ):
        listener = self._battery_status_listeners.get(listener_id)
        if listener is not None:
            listener.on_battery_status(battery_percentage, timestamp)

    def on_ping_response(self, ping_id: int):
        logger.info(f"Ping response received with id: {ping_id}")

    ##############################################################
    # Command sender wrappers
    ##############################################################

    def add_scanner(self, scanner: ButtonScanner) -> bool:
        """Add a ButtonScanner object.

        The scan will start directly once the scanner is added.
        """
        logger.info("Adding scanner")
        if scanner._scan_id in self._scanners:
            return False

        self._scanners[scanner._scan_id] = scanner
        self._send_command("CmdCreateScanner", {"scan_id": scanner._scan_id})
        return True

    def remove_scanner(self, scanner: ButtonScanner) -> bool:
        """Remove a ButtonScanner object.

        You will no longer receive advertisement packets.
        """
        logger.info("Removing scanner")
        if scanner._scan_id not in self._scanners:
            return False

        del self._scanners[scanner._scan_id]
        self._send_command("CmdRemoveScanner", {"scan_id": scanner._scan_id})
        return True

    def add_scan_wizard(self, scan_wizard: ScanWizard) -> bool:
        """Add a ScanWizard object.

        The scan wizard will start directly once the scan wizard is added.
        """
        logger.info("Adding scan wizard")
        if scan_wizard.scan_wizard_id in self._scan_wizards:
            logger.info("Scan wizard already exists")
            return False

        self._scan_wizards[scan_wizard._scan_wizard_id] = scan_wizard
        self._send_command(
            "CmdCreateScanWizard",
            {"scan_wizard_id": scan_wizard._scan_wizard_id},
        )
        logger.info(
            "Scan wizard added. Hold down the button for 7 seconds to make it public."
        )
        return True

    def cancel_scan_wizard(self, scan_wizard: ScanWizard):
        """Cancel a ScanWizard.

        Note: The effect of this command will take place at the time the on_completed event arrives on the scan wizard object.
        If cancelled due to this command, "result" in the on_completed event will be "WizardCancelledByUser".
        """
        logger.info("Cancelling scan wizard")
        if scan_wizard.scan_wizard_id not in self._scan_wizards:
            return

        self._send_command(
            "CmdCancelScanWizard",
            {"scan_wizard_id": scan_wizard._scan_wizard_id},
        )

    def add_connection_channel(self, channel: ButtonConnectionChannel):
        """Adds a connection channel to a specific Flic button.

        This will start listening for a specific Flic button's connection and button events.
        Make sure the Flic is either in public mode (by holding it down for 7 seconds) or already verified before calling this method.

        The on_create_connection_channel_response callback property will be called on the
        connection channel after this command has been received by the server.

        You may have as many connection channels as you wish for a specific Flic Button.
        """
        logger.info("Adding connection channel")

        assert len(self._bd_addrs) == len(self._connection_channels)
        if channel.conn_id in self._connection_channels:
            logger.warning("Connection channel already exists")
            assert channel.bd_addr in self._bd_addrs
            return

        self._connection_channels[channel.conn_id] = channel
        self._bd_addrs.add(channel.bd_addr)
        self._send_command(
            "CmdCreateConnectionChannel",
            {
                "conn_id": channel.conn_id,
                "bd_addr": channel.bd_addr,
                "latency_mode": channel.latency_mode,
                "auto_disconnect_time": channel.auto_disconnect_time,
            },
        )

    def remove_connection_channel(self, channel: ButtonConnectionChannel):
        """Remove a connection channel.

        This will stop listening for new events for a specific connection channel that has previously been added.
        Note: The effect of this command will take place at the time the on_removed event arrives on the connection channel object.
        """
        logger.info("Removing connection channel")

        assert len(self._bd_addrs) == len(self._connection_channels)
        if channel.conn_id not in self._connection_channels:
            logger.warning("Connection channel not found")
            assert channel.bd_addr not in self._bd_addrs
            return

        self._send_command(
            "CmdRemoveConnectionChannel", {"conn_id": channel.conn_id}
        )

    def add_battery_status_listener(self, listener: BatteryStatusListener):
        """Adds a battery status listener for a specific Flic button."""
        logger.info("Adding battery status listener")
        if listener._listener_id in self._battery_status_listeners:
            return

        self._battery_status_listeners[listener._listener_id] = listener
        self._send_command(
            "CmdCreateBatteryStatusListener",
            {
                "listener_id": listener._listener_id,
                "bd_addr": listener._bd_addr,
            },
        )

    def remove_battery_status_listener(self, listener: BatteryStatusListener):
        """Remove a battery status listener."""
        logger.info("Removing battery status listener")
        if listener._listener_id not in self._battery_status_listeners:
            return

        del self._battery_status_listeners[listener._listener_id]
        self._send_command(
            "CmdRemoveBatteryStatusListener",
            {"listener_id": listener._listener_id},
        )

    def force_disconnect(self, bd_addr: str):
        """Force disconnection or cancel pending connection of a specific Flic button.

        This removes all connection channels for all clients connected to the server for this specific Flic button.
        """
        logger.info("Force disconnecting")
        self._send_command("CmdForceDisconnect", {"bd_addr": bd_addr})

    async def get_button_info(self, bd_addr: str) -> ButtonInfo:
        """Get button info for a verified button."""
        logger.info("Getting button info")
        self._send_command("CmdGetButtonInfo", {"bd_addr": bd_addr})
        return await self._get_button_info_queue.get()

    async def get_info(self) -> Info:
        """Get info about the current state of the server."""
        logger.info("Getting info")
        self._send_command("CmdGetInfo", {})
        return await self._get_info_queue.get()

    ##############################################################
    # Connection methods
    ##############################################################

    async def connect_existing_buttons(self) -> None:
        info = await self.get_info()
        for bd_addr in info.bd_addr_of_verified_buttons:
            cc = ButtonConnectionChannel(bd_addr, client=self)
            self.add_connection_channel(cc)

    async def scan_and_connect(self) -> bool:
        logger.info("Starting the scan")
        scan_wizard = ScanWizard()
        self.add_scan_wizard(scan_wizard)
        await scan_wizard.completed_event.wait()
        logger.info("Scan completed")
        if scan_wizard.result == ScanWizardResult.WizardSuccess:
            assert scan_wizard.cc is not None
            self.add_connection_channel(scan_wizard.cc)
            return True
        else:
            return False

    ##############################################################
    # Close methods
    ##############################################################

    async def close(self):
        """Closes the transport and the client."""
        logger.info("Closing client")
        if self._transport is not None:
            self._transport.close()
        self._closed = True
        self._closed_event.set()

    @property
    def closed(self):
        return self._closed

    async def wait_for_closed(self):
        await self._closed_event.wait()


async def run(host: str, port: int, num_buttons: int):
    loop = asyncio.get_event_loop()
    _, client = await loop.create_connection(
        lambda: FlicClient(loop=loop), host, port
    )

    try:
        await client.connect_existing_buttons()
        while client.num_buttons < num_buttons:
            if await client.scan_and_connect():
                break
            else:
                logger.warning("Scan failed, sleeping for 3 seconds")
                await asyncio.sleep(3)
        logger.info(f"Connected to {client.num_buttons} buttons")
        logger.info("Spinning")
        await client.wait_for_closed()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt")
    finally:
        await client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="172.17.0.1")
    parser.add_argument("--port", type=int, default=5551)
    parser.add_argument("--num-buttons", type=int, default=2)
    parser.add_argument("--log-level", type=str, default="INFO")
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level)
    asyncio.run(run(args.host, args.port, args.num_buttons))


if __name__ == "__main__":
    main()
