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
import struct
import time
from collections import namedtuple
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import rclpy
import rclpy.logging
import rclpy.utilities
from rclpy.impl.logging_severity import LoggingSeverity

logger = rclpy.logging.get_logger("flic_client")


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


class ScanWizardError(Exception):
    """Scan wizard error"""

    def __init__(self, result: ScanWizardResult):
        self.result = result
        super().__init__(f"Scan wizard failed: {result}")


class ConnectionChannelError(Exception):
    """Connection channel error"""

    def __init__(self, error: CreateConnectionChannelError):
        self.error = error
        super().__init__(f"Connection channel not created: {error}")


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

    def __init__(
        self,
        bd_addr: str,
        latency_mode: LatencyMode = LatencyMode.LowLatency,
        auto_disconnect_time: int = 511,
        ignore_queued: bool = False,
        log_click_types: Iterable[ClickType] = (ClickType.ButtonDown,),
    ):
        """Initialize a ButtonConnectionChannel.

        Args:
            bd_addr: The Bluetooth address of the button.
            latency_mode: The latency mode to use for the connection.
            auto_disconnect_time: The auto-disconnect time in seconds.
            ignore_queued: Whether to ignore queued button events.
            log_click_types: The click types to log.
        """
        self._conn_id = self.bd_addr_to_conn_id(bd_addr)
        self._bd_addr = bd_addr
        self._latency_mode = latency_mode
        self._auto_disconnect_time = auto_disconnect_time
        self._ignore_queued = ignore_queued
        self._log_click_types = set(log_click_types)

        self._created_event = asyncio.Event()
        self._removed_event = asyncio.Event()

        self._last_time_button_event: dict[ClickType, Any] = {}

        self._button_events = {
            click_type: asyncio.Event() for click_type in ClickType
        }

    @staticmethod
    def bd_addr_to_conn_id(bd_addr: str) -> int:
        return hash(bd_addr) % 2**32

    @property
    def conn_id(self) -> int:
        return self._conn_id

    @property
    def bd_addr(self) -> str:
        return self._bd_addr

    @property
    def latency_mode(self) -> LatencyMode:
        return self._latency_mode

    @latency_mode.setter
    def latency_mode(self, latency_mode: LatencyMode):
        self._latency_mode = latency_mode

    @property
    def auto_disconnect_time(self) -> int:
        return self._auto_disconnect_time

    @auto_disconnect_time.setter
    def auto_disconnect_time(self, auto_disconnect_time: int):
        self._auto_disconnect_time = auto_disconnect_time

    @property
    def created(self) -> bool:
        return self._created_event.is_set()

    @property
    def removed(self) -> bool:
        return self._removed_event.is_set()

    @property
    def create_connection_channel_error(self) -> CreateConnectionChannelError:
        if not self.created:
            raise asyncio.InvalidStateError(
                "Create connection channel response not received"
            )
        return self._create_connection_channel_error

    @property
    def connection_status(self) -> ConnectionStatus:
        if not self.created:
            raise asyncio.InvalidStateError("Connection status not received")
        return self._connection_status

    @property
    def removed_reason(self) -> RemovedReason:
        if not self.removed:
            raise asyncio.InvalidStateError("Removed reason not received")
        return self._removed_reason

    async def wait_for_creation(self):
        await self._created_event.wait()

    async def wait_for_removal(self):
        await self._removed_event.wait()

    def on_create_connection_channel_response(
        self,
        error: CreateConnectionChannelError,
        connection_status: ConnectionStatus,
    ):
        assert (
            not self._created_event.is_set()
        ), "Create connection channel response already received"
        logger.debug(
            f"Create connection channel response: {error} {connection_status}"
        )
        self._create_connection_channel_error = error
        self._connection_status = connection_status
        self._created_event.set()

    def on_removed(self, removed_reason: RemovedReason):
        logger.debug(f"Removed: {removed_reason}")
        self._removed_reason = removed_reason
        self._removed_event.set()

    def on_connection_status_changed(
        self,
        connection_status: ConnectionStatus,
        disconnect_reason: DisconnectReason,
    ):
        disconnect_reason_str = (
            f"disconnect_reason: {disconnect_reason}"
            if connection_status == ConnectionStatus.Disconnected
            else ""
        )

        logger.debug(
            f"Connection status changed for {self._bd_addr} | "
            f"connection_status: {connection_status}, " + disconnect_reason_str
        )

    def on_button_event(
        self,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
        event_time: Any,
    ):
        msg = (
            f"{click_type.name} | "
            f"addr: {self._bd_addr}, "
            f"was_queued: {was_queued}, "
            f"time_diff: {time_diff}, "
            f"time: {event_time}"
        )

        if self._ignore_queued and was_queued:
            logger.debug(f"Ignoring queued button event: {msg}")
            return

        if click_type in self._log_click_types:
            logger.info(msg)
        else:
            logger.debug(msg)

        self._last_time_button_event[click_type] = event_time
        self._button_events[click_type].set()

    async def wait_for_button_event(self, click_type: ClickType) -> Any:
        """Wait for a button to be pressed."""
        if not self.created:
            raise asyncio.InvalidStateError("Connection channel not created")
        if self.removed:
            raise asyncio.InvalidStateError("Connection channel removed")

        self._button_events[click_type].clear()
        button_task = asyncio.create_task(
            self._button_events[click_type].wait()
        )
        removed_task = asyncio.create_task(self._removed_event.wait())

        await asyncio.wait(
            [button_task, removed_task],
            return_when=asyncio.FIRST_COMPLETED,
        )

        if button_task.done():
            return self._last_time_button_event[click_type]
        else:
            assert removed_task.done()
            raise RuntimeError(
                "Connection channel removed while waiting for button event"
            )


class ButtonScanner:
    """ButtonScanner class.

    Usage:
    scanner = ButtonScanner()
    scanner.on_advertisement_packet = lambda scanner, bd_addr, name, rssi, is_private, already_verified, already_connected_to_this_device, already_connected_to_other_device: ...
    client.add_scanner(scanner)
    """

    _cnt = itertools.count()

    def __init__(self, log_interval: Optional[float] = None):
        self._scan_id = next(ButtonScanner._cnt)
        self._last_log_time = {}
        self._log_interval = log_interval

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
        now = time.time()
        if self._log_interval is None or (
            bd_addr not in self._last_log_time
            or now - self._last_log_time[bd_addr] >= self._log_interval
        ):
            logger.debug(
                f"Received advertisement packet | "
                f"bd_addr: {bd_addr}, "
                f"name: {name}, "
                f"rssi: {rssi}, "
                f"is_private: {is_private}, "
                f"already_verified: {already_verified}, "
                f"already_connected_to_this_device: {already_connected_to_this_device}, "
                f"already_connected_to_other_device: {already_connected_to_other_device}"
            )
            self._last_log_time[bd_addr] = now


class ScanWizard:
    """ScanWizard class"""

    _cnt = itertools.count()

    def __init__(self):
        self._scan_wizard_id = next(ScanWizard._cnt)
        self._bd_addr: str | None = None
        self._name: str | None = None
        self._completed_event = asyncio.Event()

    @property
    def scan_wizard_id(self) -> int:
        return self._scan_wizard_id

    @property
    def bd_addr(self) -> str | None:
        if not self._completed_event.is_set():
            raise asyncio.InvalidStateError("ScanWizard not completed")
        return self._bd_addr

    @property
    def name(self) -> str | None:
        if not self._completed_event.is_set():
            raise asyncio.InvalidStateError("ScanWizard not completed")
        return self._name

    @property
    def result(self) -> ScanWizardResult:
        if not self._completed_event.is_set():
            raise asyncio.InvalidStateError("ScanWizard not completed")
        return self._result

    async def wait(self) -> ScanWizardResult:
        await self._completed_event.wait()
        return self._result

    def on_found_private_button(self):
        logger.debug(
            "Found a private button. Please hold it down for 7 seconds to make it public."
        )

    def on_found_public_button(self, bd_addr: str, name: str):
        self._bd_addr = bd_addr
        self._name = name
        logger.debug(
            f"Found public button {bd_addr} ({name}), now connecting..."
        )

    def on_button_connected(self):
        logger.debug(
            f"Button {self._bd_addr} ({self._name}) was connected, now verifying..."
        )

    def on_completed(self, result: ScanWizardResult):
        logger.debug(
            f"Scan wizard completed with result {result} for button {self._bd_addr} ({self._name})."
        )
        if result == ScanWizardResult.WizardSuccess:
            assert self._bd_addr is not None
            logger.debug(
                f"Your button is now ready. The bd addr is {self._bd_addr}."
            )
        elif self._bd_addr is not None and self._name is not None:
            logger.warning(
                f"Scan wizard failed with result {result} for button {self._bd_addr} ({self._name})."
            )
        else:
            logger.warning(f"Scan wizard failed with result {result}")
        self._result = result
        self._completed_event.set()


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
        logger.debug(f"Battery status: {battery_percentage} at {timestamp}")


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

    MAX_CONCURRENT_CONNECTIONS = 10

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
        default_log_click_types: Iterable[ClickType] = (ClickType.ButtonDown,),
        default_ignore_queued: bool = False,
        max_connection_channels: int = MAX_CONCURRENT_CONNECTIONS,
        time_fn: Callable[[], Any] = time.time,
    ):
        self._loop = loop
        self.default_latency_mode = default_latency_mode
        self.default_auto_disconnect_time = default_auto_disconnect_time
        self.default_log_click_types = default_log_click_types
        self.default_ignore_queued = default_ignore_queued
        if max_connection_channels > self.MAX_CONCURRENT_CONNECTIONS:
            raise ValueError(
                f"max_connection_channels must be less than {self.MAX_CONCURRENT_CONNECTIONS}"
            )
        self.max_connection_channels = max_connection_channels
        self.time = time_fn
        self._buffer = b""
        self._transport = None

        self._scanners: dict[int, ButtonScanner] = {}
        self._scan_wizards: dict[int, ScanWizard] = {}
        self._pending_connection_channels: dict[
            int, ButtonConnectionChannel
        ] = {}
        self._connection_channels: dict[int, ButtonConnectionChannel] = {}
        self._battery_status_listeners: dict[int, BatteryStatusListener] = {}
        self._buttons_ordered: dict[int, ButtonConnectionChannel] = {}

        self._get_button_info_queue: asyncio.Queue[ButtonInfo] = (
            asyncio.Queue()
        )
        self._get_info_queue: asyncio.Queue[Info] = asyncio.Queue()

        self._closed_event = asyncio.Event()

        self._last_time_button_event: dict[ClickType, Any] = {}
        self._button_events = {
            click_type: asyncio.Event() for click_type in ClickType
        }

        logger.debug(
            f"FlicClient initialized with "
            f"default_latency_mode: {default_latency_mode}, "
            f"default_auto_disconnect_time: {default_auto_disconnect_time}, "
            f"default_log_click_types: {default_log_click_types}, "
            f"max_connection_channels: {max_connection_channels}"
        )

    @property
    def num_connection_channels(self) -> int:
        return len(self._connection_channels) + len(
            self._pending_connection_channels
        )

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

    def connection_made(self, transport: asyncio.Transport):
        logger.debug(f"Connection made to {type(transport)}: {transport}")
        self._transport = transport

    def data_received(self, data: bytes):
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

    def eof_received(self):
        self.close()

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
            case "EvtButtonDeleted":
                pass  # EvtButtonDeleted starts with EvtButton but has no click_type
            case _ if event_name.startswith("EvtButton"):
                items["click_type"] = ClickType(items["click_type"])
                items["event_time"] = self.time()

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
            case "EvtNewVerifiedButton":
                self.on_new_verified_button(items["bd_addr"])
            case "EvtButtonDeleted":
                self.on_button_deleted(
                    items["bd_addr"], items["deleted_by_this_client"]
                )
            case _ if event_name.startswith("EvtButton"):
                self.on_button_event(
                    conn_id=items["conn_id"],
                    click_type=items["click_type"],
                    was_queued=items["was_queued"],
                    time_diff=items["time_diff"],
                    event_time=items["event_time"],
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
        logger.debug(
            f"Got space for new connection. Max: {max_concurrently_connected_buttons}"
        )

    def on_bluetooth_controller_state_change(
        self, state: BluetoothControllerState
    ):
        logger.debug(f"Bluetooth controller state changed: {state}")

    def on_new_verified_button(self, bd_addr: str):
        logger.debug(f"New verified button {bd_addr}")

    def on_button_deleted(self, bd_addr: str, deleted_by_this_client: bool):
        logger.debug(f"Button deleted: {bd_addr} {deleted_by_this_client}")

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
        scan_wizard.on_completed(result=result)
        del self._scan_wizards[scan_wizard_id]

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
        logger.debug(f"Got info: {info}")
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
        logger.debug(
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
        channel = self._pending_connection_channels[conn_id]
        channel.on_create_connection_channel_response(
            error=error,
            connection_status=connection_status,
        )
        del self._pending_connection_channels[conn_id]

        if error == CreateConnectionChannelError.NoError:
            self._connection_channels[conn_id] = channel
        else:
            logger.warning(f"Failed to create connection channel: {error}")

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

    def on_button_event(
        self,
        conn_id: int,
        click_type: ClickType,
        was_queued: bool,
        time_diff: int,
        event_time: Any,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_button_event(
            click_type=click_type,
            was_queued=was_queued,
            time_diff=time_diff,
            event_time=event_time,
        )

        if (
            click_type == ClickType.ButtonDown
            and channel.conn_id not in self._buttons_ordered
        ):
            self._buttons_ordered[channel.conn_id] = channel

        if self.default_ignore_queued and was_queued:
            return

        self._last_time_button_event[click_type] = event_time
        self._button_events[click_type].set()

    def on_connection_channel_removed(
        self,
        conn_id: int,
        removed_reason: RemovedReason,
    ):
        channel = self._connection_channels[conn_id]
        channel.on_removed(removed_reason=removed_reason)
        del self._connection_channels[conn_id]

    def on_battery_status(
        self, listener_id: int, battery_percentage: int, timestamp: int
    ):
        listener = self._battery_status_listeners.get(listener_id)
        if listener is not None:
            listener.on_battery_status(battery_percentage, timestamp)

    def on_ping_response(self, ping_id: int):
        logger.debug(f"Ping response received with id: {ping_id}")

    ##############################################################
    # Command sender wrappers
    ##############################################################

    def add_scanner(self, scanner: ButtonScanner):
        """Add a ButtonScanner object.

        The scan will start directly once the scanner is added.
        """
        logger.debug("Adding scanner")
        if scanner._scan_id in self._scanners:
            raise ValueError("Scanner already exists")

        self._scanners[scanner._scan_id] = scanner
        self._send_command("CmdCreateScanner", {"scan_id": scanner._scan_id})

    def remove_scanner(self, scanner: ButtonScanner):
        """Remove a ButtonScanner object.

        You will no longer receive advertisement packets.
        """
        logger.debug("Removing scanner")
        if scanner._scan_id not in self._scanners:
            raise ValueError("Scanner not found")

        del self._scanners[scanner._scan_id]
        self._send_command("CmdRemoveScanner", {"scan_id": scanner._scan_id})

    def add_scan_wizard(self, scan_wizard: ScanWizard):
        """Add a ScanWizard object.

        The scan wizard will start directly once the scan wizard is added.
        """
        logger.debug("Adding scan wizard")
        if scan_wizard.scan_wizard_id in self._scan_wizards:
            raise ValueError(
                f"Scan wizard with id {scan_wizard.scan_wizard_id} already exists"
            )

        self._scan_wizards[scan_wizard._scan_wizard_id] = scan_wizard
        self._send_command(
            "CmdCreateScanWizard",
            {"scan_wizard_id": scan_wizard._scan_wizard_id},
        )

    def cancel_scan_wizard(self, scan_wizard: ScanWizard):
        """Cancel a ScanWizard.

        Note: The effect of this command will take place at the time the on_completed event arrives on the scan wizard object.
        If cancelled due to this command, "result" in the on_completed event will be "WizardCancelledByUser".
        """
        logger.debug("Cancelling scan wizard")
        if scan_wizard.scan_wizard_id not in self._scan_wizards:
            raise ValueError(
                f"Scan wizard with id {scan_wizard.scan_wizard_id} not found"
            )

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
        logger.debug("Adding connection channel")

        if channel.created:
            raise ValueError("Connection channel already created")

        if channel.conn_id in self._connection_channels:
            raise ValueError("Connection channel already exists")
        elif channel.conn_id in self._pending_connection_channels:
            raise ValueError("Connection channel is pending")

        self._pending_connection_channels[channel.conn_id] = channel
        self._send_command(
            "CmdCreateConnectionChannel",
            {
                "conn_id": channel.conn_id,
                "bd_addr": channel.bd_addr,
                "latency_mode": channel.latency_mode,
                "auto_disconnect_time": channel.auto_disconnect_time,
            },
        )

    def remove_connection_channel(self, conn_id: int):
        """Remove a connection channel.

        This will stop listening for new events for a specific connection channel that has previously been added.
        Note: The effect of this command will take place at the time the on_connection_channel_removed event arrives on the connection channel object.
        """
        logger.debug("Removing connection channel")

        if conn_id not in self._connection_channels:
            raise ValueError("Connection channel not found")

        self._send_command("CmdRemoveConnectionChannel", {"conn_id": conn_id})

    def update_connection_channel(
        self,
        conn_id: int,
        latency_mode: Optional[LatencyMode] = None,
        auto_disconnect_time: Optional[int] = None,
    ):
        """Update the connection channel parameters."""
        logger.debug("Updating connection channel parameters")

        channel = self._connection_channels[conn_id]

        if latency_mode is None and auto_disconnect_time is None:
            raise ValueError("No parameters to update")

        if latency_mode is not None:
            channel.latency_mode = latency_mode

        if auto_disconnect_time is not None:
            channel.auto_disconnect_time = auto_disconnect_time

        self._send_command(
            "CmdChangeModeParameters",
            {
                "conn_id": channel.conn_id,
                "latency_mode": channel.latency_mode,
                "auto_disconnect_time": channel.auto_disconnect_time,
            },
        )

    def add_battery_status_listener(self, listener: BatteryStatusListener):
        """Adds a battery status listener for a specific Flic button."""
        logger.debug("Adding battery status listener")
        if listener._listener_id in self._battery_status_listeners:
            raise ValueError("Battery status listener already exists")

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
        logger.debug("Removing battery status listener")
        if listener._listener_id not in self._battery_status_listeners:
            raise ValueError("Battery status listener not found")

        del self._battery_status_listeners[listener._listener_id]
        self._send_command(
            "CmdRemoveBatteryStatusListener",
            {"listener_id": listener._listener_id},
        )

    def force_disconnect(self, bd_addr: str):
        """Force disconnection or cancel pending connection of a specific Flic button.

        This removes all connection channels for all clients connected to the server for this specific Flic button.
        """
        logger.debug("Force disconnecting")
        self._send_command("CmdForceDisconnect", {"bd_addr": bd_addr})

    def delete_button(self, bd_addr: str):
        """Delete a button."""
        logger.debug("Deleting button")
        self._send_command("CmdDeleteButton", {"bd_addr": bd_addr})

    async def get_button_info(self, bd_addr: str) -> ButtonInfo:
        """Get button info for a verified button."""
        logger.debug("Getting button info")
        self._send_command("CmdGetButtonInfo", {"bd_addr": bd_addr})
        return await self._get_button_info_queue.get()

    async def get_info(self) -> Info:
        """Get info about the current state of the server."""
        logger.debug("Getting info")
        self._send_command("CmdGetInfo", {})
        return await self._get_info_queue.get()

    ##############################################################
    # Connection methods
    ##############################################################

    async def disconnect_all(self):
        """Disconnect all buttons."""
        info = await self.get_info()
        logger.info(
            f"Disconnecting any of {len(info.bd_addr_of_verified_buttons)} buttons"
        )
        for bd_addr in info.bd_addr_of_verified_buttons:
            self.force_disconnect(bd_addr)

    async def disconnect_first(self):
        """Disconnect the first button."""
        if len(self._connection_channels) == 0:
            raise ValueError("No connection channels to disconnect")

        cc = next(iter(self._connection_channels.values()))
        old_num_connection_channels = self.num_connection_channels
        self.force_disconnect(cc.bd_addr)
        await cc.wait_for_removal()
        assert self.num_connection_channels == old_num_connection_channels - 1

    async def scan(self) -> str:
        scan_wizard = ScanWizard()
        self.add_scan_wizard(scan_wizard)

        logger.info(
            "Scan wizard added. Hold down the button for 7 seconds to make it public."
        )

        await scan_wizard.wait()

        if scan_wizard.result != ScanWizardResult.WizardSuccess:
            raise ScanWizardError(scan_wizard.result)

        assert scan_wizard.bd_addr is not None

        return scan_wizard.bd_addr

    async def connect(
        self,
        bd_addr: str,
        latency_mode: Optional[LatencyMode] = None,
        auto_disconnect_time: Optional[int] = None,
        ignore_queued: Optional[bool] = None,
        log_click_types: Optional[Iterable[ClickType]] = None,
    ) -> ButtonConnectionChannel:
        """Connect to a button."""
        if latency_mode is None:
            latency_mode = self.default_latency_mode
        if auto_disconnect_time is None:
            auto_disconnect_time = self.default_auto_disconnect_time
        if ignore_queued is None:
            ignore_queued = self.default_ignore_queued
        if log_click_types is None:
            log_click_types = self.default_log_click_types

        cc = ButtonConnectionChannel(
            bd_addr,
            latency_mode,
            auto_disconnect_time,
            ignore_queued,
            log_click_types,
        )

        if cc.conn_id in self._connection_channels:
            logger.debug(f"Connection channel {cc.conn_id} already exists")
            cc = self._connection_channels[cc.conn_id]
        elif cc.conn_id in self._pending_connection_channels:
            logger.debug(f"Connection channel {cc.conn_id} is pending")
            cc = self._pending_connection_channels[cc.conn_id]
        else:
            logger.debug(f"Creating connection channel {cc.conn_id}")
            info = await self.get_info()
            if bd_addr not in info.bd_addr_of_verified_buttons:
                raise ValueError(f"Button {bd_addr} is not verified")

            if self.num_connection_channels >= self.max_connection_channels:
                await self.disconnect_first()

            self.add_connection_channel(cc)

        await cc.wait_for_creation()

        assert cc.created and not cc.removed

        if (
            cc.create_connection_channel_error
            != CreateConnectionChannelError.NoError
        ):
            raise ConnectionChannelError(cc.create_connection_channel_error)

        return cc

    ##############################################################
    # Event methods
    ##############################################################

    async def wait_for_button_event(self, click_type: ClickType):
        """Wait for a button to be pressed."""
        self._button_events[click_type].clear()
        await self._button_events[click_type].wait()

    ##############################################################
    # Close methods
    ##############################################################

    def close(self):
        """Closes the transport and the client."""
        if self._closed_event.is_set():
            return

        logger.info("Closing client")
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        self._closed_event.set()

    @property
    def closed(self):
        return self._closed_event.is_set()

    async def wait_for_closed(self):
        await self._closed_event.wait()


async def scan(client: FlicClient):
    """Scan for buttons until cancelled."""

    logger.info("Starting scan loop")
    while True:
        await client.get_info()

        try:
            await client.scan()
        except ScanWizardError as e:
            logger.warning(f"Scan wizard failed: {e}")
            continue
        logger.info("Scan wizard succeeded")


async def spin(client: FlicClient, period: float):
    """Repeatedly connect to verified buttons to get all button events."""

    info = await client.get_info()
    logger.info(
        f"Spinning {len(info.bd_addr_of_verified_buttons)} buttons every {period}s"
    )
    time_per_button = period / len(info.bd_addr_of_verified_buttons)
    try:
        while True:
            for bd_addr in info.bd_addr_of_verified_buttons:
                start = time.time()
                await client.connect(bd_addr)
                remaining = time_per_button - (time.time() - start)
                if remaining > 0:
                    await asyncio.sleep(remaining)
    finally:
        print("Buttons pressed in order:")
        for channel in client._buttons_ordered.values():
            print(channel.bd_addr)


async def delete(client: FlicClient, addresses: str | list[str], all: bool):
    """Delete buttons."""

    if all and len(addresses) > 0:
        raise ValueError("Cannot use all and provide addresses")

    info = await client.get_info()

    if all:
        addresses = list(info.bd_addr_of_verified_buttons)
    elif isinstance(addresses, str):
        addresses = [addresses]
    elif len(addresses) == 0:
        raise ValueError("No addresses provided")

    elif any(
        bd_addr not in info.bd_addr_of_verified_buttons
        for bd_addr in addresses
    ):
        raise ValueError("Invalid address provided")

    for bd_addr in addresses:
        client.delete_button(bd_addr)

    await asyncio.sleep(1.0)


async def info(client: FlicClient):
    """Get info."""
    info = await client.get_info()
    logger.info(f"Info: {info}")


async def main_async(
    command: str,
    host: str,
    port: int,
    max_connections: int,
    ignore_queued: bool,
    **kwargs,
):
    match command:
        case "scan":
            coro_fn = scan
        case "delete":
            coro_fn = delete
        case "spin":
            coro_fn = spin
        case "info":
            coro_fn = info
        case _:
            raise ValueError(f"Invalid command: {command}")

    loop = asyncio.get_event_loop()
    _, client = await loop.create_connection(
        lambda: FlicClient(
            loop=loop,
            max_connection_channels=max_connections,
            default_ignore_queued=ignore_queued,
        ),
        host,
        port,
    )
    try:
        coro_task = asyncio.create_task(coro_fn(client, **kwargs))
        closed_task = asyncio.create_task(client.wait_for_closed())

        done, _ = await asyncio.wait(
            [coro_task, closed_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in done:
            task.result()
    finally:
        client.close()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="172.17.0.1")
    parser.add_argument("--port", type=int, default=5551)
    parser.add_argument(
        "--max-connections",
        type=int,
        default=FlicClient.MAX_CONCURRENT_CONNECTIONS,
    )
    parser.add_argument(
        "--ignore-queued",
        action="store_true",
        default=False,
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("scan")
    subparsers.add_parser("info")

    parser_spin = subparsers.add_parser("spin")
    parser_spin.add_argument("--period", type=float, default=0.5)

    parser_delete = subparsers.add_parser("delete")
    parser_delete.add_argument("addresses", nargs="*")
    parser_delete.add_argument("--all", action="store_true")

    non_ros_args = rclpy.utilities.remove_ros_args(args)
    args, _ = parser.parse_known_args(non_ros_args[1:])

    if args.verbose:
        logger.set_level(LoggingSeverity.DEBUG)
    else:
        logger.set_level(LoggingSeverity.INFO)
    delattr(args, "verbose")

    try:
        asyncio.run(main_async(**vars(args)))
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt")
    finally:
        rclpy.try_shutdown()  # type: ignore


if __name__ == "__main__":
    main()
