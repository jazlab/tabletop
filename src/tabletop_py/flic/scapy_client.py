"""Drop-in Flic client that sniffs BLE advertisements directly via scapy.

The default :mod:`tabletop_py.flic.client` talks to ``flicd``, which itself
connects to each Flic button over GATT to read the click-type
characteristic. That GATT round-trip adds tens of ms of latency between
the user pressing the button and the daemon emitting an event.

This module bypasses ``flicd`` entirely. It opens a raw HCI socket via
scapy, enables LE scanning with duplicate filtering disabled, and reports
the first advertisement received from each verified Flic button as a
synthetic ``ButtonDown``. A Flic broadcasts a connectable advertisement
as soon as the contact closes, so we catch the press at the wake-up
edge instead of after a connection has been established.

The client follows the same :class:`asyncio.Protocol` pattern as
:mod:`tabletop_py.flic.client`: a Transport drives reads from the HCI
socket and ``FlicClient`` (the Protocol) gets ``connection_made`` /
``data_received`` / ``connection_lost`` callbacks. We can't use
``loop.create_connection`` here because it requires ``SOCK_STREAM`` and
the HCI socket is ``SOCK_RAW`` — instead :meth:`FlicClient.create` wires
up a small custom :class:`_HCISocketTransport`. Construction looks like:

    client = await FlicClient.create(
        loop=loop,
    )

Trade-offs vs. the daemon-based client:
    - No connection channels: we never connect to the button, so click
      type (single vs. double vs. hold), battery status, ScanWizard
      pairing, and ``EvtButtonUp`` events are not available. Only the
      advertisement-packet path is functional.
    - Verified buttons must be supplied at construction time. There is
      no daemon database to query.
    - The process needs ``CAP_NET_ADMIN`` + ``CAP_NET_RAW`` to open the
      HCI socket. Either run as root or grant capabilities with::

          sudo setcap 'cap_net_raw,cap_net_admin+eip' $(readlink -f $(which python3))

    - ``flicd``/``bluetoothd`` must not be holding the controller in a
      conflicting scan state. Stop ``flicd`` before running this client.
"""

import asyncio
import logging
import socket
import time
from collections.abc import Callable
from fcntl import ioctl
from typing import Any, Literal, NamedTuple

from scapy.data import MTU
from scapy.layers.bluetooth import (
    BluetoothCommandError,
    BluetoothHCISocket,
    BluetoothMonitorSocket,
    BluetoothUserSocket,
    HCI_Cmd_Disconnect,
    HCI_Cmd_LE_Create_Connection,
    HCI_Cmd_LE_Create_Connection_Cancel,
    HCI_Cmd_LE_Read_Buffer_Size_V1,
    HCI_Cmd_LE_Read_Buffer_Size_V2,
    HCI_Cmd_LE_Set_Scan_Enable,
    HCI_Cmd_LE_Set_Scan_Parameters,
    HCI_Cmd_Read_LE_Host_Support,
    HCI_Cmd_Reset,
    HCI_Cmd_Set_Event_Filter,
    HCI_Cmd_Set_Event_Mask,
    HCI_Command_Hdr,
    HCI_Event_Command_Complete,
    HCI_Event_Command_Status,
    HCI_Event_Disconnection_Complete,
    HCI_Hdr,
    HCI_LE_Meta_Advertising_Report,
    HCI_LE_Meta_Advertising_Reports,
    HCI_LE_Meta_Connection_Complete,
    HCI_Mon_Hdr,
)
from scapy.packet import Packet

logger = logging.getLogger(__name__)

# From hci.h
HCIDEVUP = 0x400448C9  # 201
HCIDEVDOWN = 0x400448CA  # 202
HCIGETDEVINFO = 0x7FFBB72D  # _IOR(ord('H'), 211, 4)
HCISETSCAN = 0x400448DD  # 221
HCI_CHANNEL_RAW = 0
HCI_CHANNEL_USER = 1
HCI_CHANNEL_MONITOR = 2
HCI_CHANNEL_CONTROL = 3
HCI_CHANNEL_LOGGING = 4

# Opcodes we need to recognize on Command Status (those commands don't
# emit Command Complete). Computed once from scapy classes at import.
_OPCODE_LE_CREATE_CONNECTION = 0x200D
_OPCODE_LE_CREATE_CONNECTION_CANCEL = 0x200E
_OPCODE_DISCONNECT = 0x0406

# "Remote User Terminated Connection" — the recommended host-initiated
# disconnect reason per the Bluetooth Core spec.
_DISCONNECT_REASON_REMOTE_USER = 0x13


def _hci_dev_down(adapter_idx: int = 0):
    sock = socket.socket(
        socket.AF_BLUETOOTH,  # type: ignore
        socket.SOCK_RAW,
        socket.BTPROTO_HCI,  # type: ignore
    )
    try:
        ioctl(sock.fileno(), HCIDEVDOWN, adapter_idx)
    finally:
        sock.close()


def _hci_dev_up(adapter_idx: int = 0):
    sock = socket.socket(
        socket.AF_BLUETOOTH,  # type: ignore
        socket.SOCK_RAW,
        socket.BTPROTO_HCI,  # type: ignore
    )
    try:
        ioctl(sock.fileno(), HCIDEVUP, adapter_idx)
    finally:
        sock.close()


class ButtonPressInfo(NamedTuple):
    """Information about a button press event.

    Attributes:
        addr: Bluetooth address of the button (e.g., "aa:bb:cc:dd:ee:ff").
        time: Timestamp of the button press event.
    """

    addr: str
    time: Any


def default_advertisement_event_filter(
    report: HCI_LE_Meta_Advertising_Report, event_time: Any
) -> bool:
    """Filter BLE advertisements to only Flic button MAC address prefixes.

    Accepts advertisement reports from Flic buttons (identifiable by their
    manufacturer-specific MAC address prefixes).

    Args:
        report: HCI LE advertising report from the BLE controller.
        event_time: Timestamp of the advertisement event.

    Returns:
        True if the report is from a known Flic button, False otherwise.
    """
    addr = str(report.addr).lower()
    prefix = addr[:8]
    return prefix in ("80:e4:da", "90:88:a9")


type SocketTypeT = (
    BluetoothHCISocket | BluetoothUserSocket | BluetoothMonitorSocket
)


class _HCISocketTransport(asyncio.BaseTransport):
    """Minimal Transport wrapping a scapy HCI socket.

    ``loop.create_connection`` rejects this socket (it requires
    ``SOCK_STREAM``; HCI is ``SOCK_RAW``), so we hand-roll the bits of
    the transport contract we actually need: register an ``add_reader``
    on the fd, pass each frame to ``protocol.data_received``, and expose
    a synchronous ``write`` for the protocol's command sender.
    """

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        sock: SocketTypeT,
        protocol: "FlicClient",
    ):
        self._loop = loop
        self._sock = sock
        self._protocol = protocol
        self._closing = False

        loop.add_reader(sock.fileno(), self._on_readable)
        # asyncio's stock transports schedule this via call_soon; here
        # we call it inline so FlicClient.create() can issue commands
        # immediately after constructing the transport without first
        # yielding to the loop.
        protocol.connection_made(self)

    def _on_readable(self):
        try:
            data = self._sock.ins.recv(MTU)
        except (BlockingIOError, OSError) as e:
            logger.warning(f"HCI recv failed: {e}")
            return
        if not data:
            # SOCK_RAW shouldn't normally return zero bytes, but if it
            # does treat it as EOF on the underlying fd.
            self._protocol.eof_received()
            return
        self._protocol.data_received(data)

    def write(self, data: bytes) -> int:
        """Write data to the HCI socket.

        Args:
            data: Bytes to write to the socket.

        Returns:
            Number of bytes written.

        Raises:
            RuntimeError: If transport is closing or send fails.
        """
        if self._closing:
            raise RuntimeError("transport is closing, cannot write")
        n = self._sock.ins.send(data)
        if n <= 0:
            raise RuntimeError(f"HCI send returned {n}")
        return n

    def close(self):
        """Close the transport and underlying socket.

        Safely closes the HCI socket and schedules the protocol's
        connection_lost callback. Safe to call multiple times.
        """
        if self._closing:
            return
        self._closing = True
        try:
            self._loop.remove_reader(self._sock.fileno())
        except Exception as e:
            logger.debug(f"remove_reader failed: {e}")
        try:
            self._sock.close()
        except Exception as e:
            logger.debug(f"sock.close failed: {e}")
        self._loop.call_soon(self._protocol.connection_lost, None)

    def is_closing(self) -> bool:
        """Check if the transport is closing.

        Returns:
            True if close() has been called, False otherwise.
        """
        return self._closing


class FlicClient(asyncio.Protocol):
    """Async Flic client backed by direct BLE advertisement sniffing.

    Subclasses :class:`asyncio.Protocol`. Construction goes through
    :meth:`create` (not ``loop.create_connection``) because the HCI
    socket is ``SOCK_RAW`` and asyncio's TCP setup rejects non-stream
    sockets.

    Implements the subset of the flicd client API that
    ``tabletop_rig.nodes.flic`` actually exercises: scanner-style
    advertisement waiting, info lookup, and lifecycle (close /
    wait_for_closed).
    """

    def __init__(
        self,
        socket_type: Literal["hci", "user", "monitor"] = "user",
        advertising_event_filter: Callable[
            [HCI_LE_Meta_Advertising_Report, Any], bool
        ] = default_advertisement_event_filter,
        event_time_fn: Callable[[], Any] = time.time,
        kill_on_press: bool = True,
        kill_timeout: float = 2.0,
    ):
        self._socket_type = socket_type
        self._advertising_event_filter = advertising_event_filter
        self._event_time_fn = event_time_fn
        self._kill_on_press = kill_on_press
        self._kill_timeout = kill_timeout

        self._loop: asyncio.AbstractEventLoop | None = None
        self._transport: _HCISocketTransport | None = None
        self._closed_event = asyncio.Event()

        self._pending_command_futures: dict[int, asyncio.Future[Packet]] = {}

        # Kill-on-press bookkeeping. The controller can only initiate
        # one LE connection at a time, so we serialize via _killing and
        # drop overlapping triggers.
        self._kill_lock = asyncio.Lock()
        self._kill_tasks: dict[str, asyncio.Task] = {}
        # handle from LE_Meta_Connection_Complete. At most one entry.
        self._pending_connect_futures: dict[str, asyncio.Future[int]] = {}
        # connection_handle -> future resolving when Disconnection
        # Complete arrives.
        self._pending_disconnect_futures: dict[int, asyncio.Future[None]] = {}

        self._button_press_futures: dict[
            str, asyncio.Future[ButtonPressInfo]
        ] = {}

        self._any_button_press_future: (
            asyncio.Future[ButtonPressInfo] | None
        ) = None

    @staticmethod
    def _create_socket(
        device_id: int,
        socket_type: Literal["hci", "user", "monitor"],
    ) -> SocketTypeT:
        match socket_type:
            case "hci":
                sock = BluetoothHCISocket(device_id)
            case "user":
                _hci_dev_down(device_id)
                sock = BluetoothUserSocket(device_id)
            case "monitor":
                sock = BluetoothMonitorSocket()
            case _:
                raise ValueError(
                    f"Unknown socket_type: '{socket_type}'. Supported: ['hci', 'user', 'monitor']"
                )

        sock.ins.setblocking(False)
        assert not sock.ins.getblocking()
        sock.nonblocking_socket = True

        return sock

    @classmethod
    async def create(
        cls,
        *,
        device_id: int = 0,
        socket_type: Literal["hci", "user", "monitor"] = "user",
        advertising_event_filter: Callable[
            [HCI_LE_Meta_Advertising_Report, Any], bool
        ] = default_advertisement_event_filter,
        event_time_fn: Callable[[], Any] = time.time,
        kill_on_press: bool = True,
        kill_timeout: float = 2.0,
        loop: asyncio.AbstractEventLoop,
        active_scan: bool = False,
        scan_interval: int = 16,
        scan_window: int = 16,
    ) -> "FlicClient":
        """Open the HCI socket, install the Protocol, and start LE scanning.

        The shape mirrors ``loop.create_connection(lambda: FlicClient(...))``:
        we build the protocol, attach a transport, and (only on ``user``
        sockets) run the reset + LE-scan-enable sequence over that
        transport.

        Args:
            loop: Event loop to register the socket reader on.
            device_id: HCI controller index (``hciN``). Defaults to 0.
            active_scan: If ``True``, the controller sends SCAN_REQ to
                advertisers. Passive scanning is sufficient for Flic
                press detection.
            scan_interval, scan_window: 0.625 ms units. Equal values
                mean continuous scanning.
            kill_on_press: If ``True``, every advertisement from a
                verified button triggers a brief LE connect + immediate
                disconnect, which forces the Flic to stop advertising
                its press burst. See :meth:`_kill_advertising`.
            kill_timeout: Per-step timeout (connect, then disconnect)
                for the kill flow, in seconds.
        """
        sock = cls._create_socket(device_id, socket_type)
        client = cls(
            socket_type=socket_type,
            advertising_event_filter=advertising_event_filter,
            event_time_fn=event_time_fn,
            kill_on_press=kill_on_press,
            kill_timeout=kill_timeout,
        )
        client._loop = loop
        # Side effect: synchronously invokes client.connection_made(),
        # which sets client._transport so _send_command works below.
        _HCISocketTransport(loop, sock, client)

        try:
            await client._configure_scan(
                active_scan=active_scan,
                scan_interval=scan_interval,
                scan_window=scan_window,
            )
        except BaseException:
            client.close()
            raise

        return client

    ##############################################################
    # asyncio.Protocol methods
    ##############################################################

    def connection_made(self, transport: asyncio.BaseTransport):
        """Store the transport reference once the HCI socket is wired up."""
        logger.debug(f"Connection made to {type(transport)}: {transport}")
        assert isinstance(transport, _HCISocketTransport)
        self._transport = transport

    def data_received(self, data: bytes):
        """Parse a raw HCI frame and dispatch it to the appropriate handler."""
        event_time = self._event_time_fn()
        try:
            if self._socket_type == "monitor":
                pkt = HCI_Mon_Hdr(data)
            else:
                pkt = HCI_Hdr(data)
        except Exception as e:
            logger.warning(f"Failed to parse HCI packet: {e}")
            return
        self._dispatch_event(pkt, event_time)

    def eof_received(self):
        """Close the client when the underlying HCI socket signals EOF."""
        logger.info("EOF received on HCI socket")
        self.close()

    def connection_lost(self, exc: Exception | None):
        """Close the client when the transport reports the connection is gone."""
        if exc is not None:
            logger.warning(f"Connection lost: {exc}")
        else:
            logger.debug("Connection lost")
        self.close()

    ##############################################################
    # Event dispatcher
    ##############################################################

    def _dispatch_event(self, pkt: Packet, event_time: Any):
        """Route an incoming HCI packet to the relevant event handler(s).

        A single HCI packet can carry multiple LE meta reports, so we
        dispatch per layer rather than via a single match.
        """
        if HCI_Event_Command_Complete in pkt:
            self.on_command_complete(pkt[HCI_Event_Command_Complete])

        if HCI_Event_Command_Status in pkt:
            self.on_command_status(pkt[HCI_Event_Command_Status])

        if HCI_LE_Meta_Advertising_Reports in pkt:
            for report in pkt[HCI_LE_Meta_Advertising_Reports].reports:
                self.on_advertising_report(report, event_time=event_time)

        # if HCI_LE_Meta_Advertising_Report in pkt:
        #     self.on_advertising_report(
        #         pkt[HCI_LE_Meta_Advertising_Report], event_time=event_time
        #     )

        if HCI_LE_Meta_Connection_Complete in pkt:
            self.on_connection_complete(pkt[HCI_LE_Meta_Connection_Complete])

        if HCI_Event_Disconnection_Complete in pkt:
            self.on_disconnection_complete(
                pkt[HCI_Event_Disconnection_Complete]
            )

    ##############################################################
    # Event handlers
    ##############################################################

    def on_command_complete(self, pkt: HCI_Event_Command_Complete):
        """Resolve the pending future for this opcode, if any."""
        opcode = pkt.opcode
        future = self._pending_command_futures.get(opcode)
        if future is None:
            logger.debug(
                f"Unsolicited Command Complete (opcode=0x{opcode:04x})"
            )
            return
        if future.done():
            return
        future.set_result(pkt)

    def on_command_status(self, pkt: HCI_Event_Command_Status):
        """Surface failures for commands that emit Command Status only.

        LE_Create_Connection and Disconnect don't generate Command
        Complete — they return Command Status, then a follow-up event
        (Connection Complete / Disconnection Complete). A non-zero
        status here means the controller refused the command outright;
        we fail the matching pending future so the kill task aborts
        instead of waiting for an event that will never come.
        """
        if pkt.status == 0:
            return  # success — wait for the follow-up event
        logger.warning(
            f"Command Status: opcode=0x{pkt.opcode:04x} "
            f"status=0x{pkt.status:02x}"
        )
        if pkt.opcode == _OPCODE_LE_CREATE_CONNECTION:
            for addr, future in list(self._pending_connect_futures.items()):
                if not future.done():
                    future.set_exception(
                        BluetoothCommandError(
                            f"LE_Create_Connection rejected: "
                            f"status=0x{pkt.status:02x}"
                        )
                    )
                self._pending_connect_futures.pop(addr, None)
        elif pkt.opcode == _OPCODE_DISCONNECT:
            for handle, future in list(
                self._pending_disconnect_futures.items()
            ):
                if not future.done():
                    future.set_exception(
                        BluetoothCommandError(
                            f"Disconnect rejected: status=0x{pkt.status:02x}"
                        )
                    )
                self._pending_disconnect_futures.pop(handle, None)

    def on_connection_complete(self, pkt: HCI_LE_Meta_Connection_Complete):
        """Resolve the pending connect future for the matching peer."""
        addr = str(pkt.paddr).lower()
        future = self._pending_connect_futures.pop(addr, None)
        if future is None:
            logger.debug(
                f"Unsolicited Connection Complete for {addr} "
                f"handle={pkt.handle}"
            )
            return
        if future.done():
            return
        if pkt.status != 0:
            future.set_exception(
                BluetoothCommandError(
                    f"Connection to {addr} failed: status=0x{pkt.status:02x}"
                )
            )
            return
        future.set_result(int(pkt.handle))

    def on_disconnection_complete(self, pkt: HCI_Event_Disconnection_Complete):
        """Resolve the pending disconnect future for the matching handle."""
        future = self._pending_disconnect_futures.pop(int(pkt.handle), None)
        if future is None:
            logger.debug(
                f"Unsolicited Disconnection Complete handle={pkt.handle}"
            )
            return
        if not future.done():
            future.set_result(None)

    def on_advertising_report(
        self, pkt: HCI_LE_Meta_Advertising_Report, *, event_time: Any
    ):
        """Forward an LE advertising report to the button waiters."""
        if not self._advertising_event_filter(pkt, event_time):
            return
        addr = str(pkt.addr).lower()

        if addr not in self._kill_tasks:
            self._dispatch_button_press(addr, event_time=event_time)

        if self._kill_on_press and addr not in self._kill_tasks:
            self._schedule_kill_advertising(
                addr, int(getattr(pkt, "atype", 0))
            )

    def _dispatch_button_press(self, addr: str, *, event_time: Any):
        if (
            addr in self._button_press_futures
            and not self._button_press_futures[addr].done()
        ):
            self._button_press_futures[addr].set_result(
                ButtonPressInfo(addr, event_time)
            )

        if (
            self._any_button_press_future is not None
            and not self._any_button_press_future.done()
        ):
            self._any_button_press_future.set_result(
                ButtonPressInfo(addr, event_time)
            )

    ##############################################################
    # Kill-on-press flow
    ##############################################################

    def _schedule_kill_advertising(self, addr: str, patype: int):
        """Fire a one-shot connect+disconnect to silence the Flic.

        Called synchronously from the reader callback so we can issue
        LE_Create_Connection on the very next event-loop tick. The
        controller only supports one initiating attempt at a time, so
        overlapping adverts (from a second button, or a repeat from the
        same one) are dropped until the in-flight kill finishes.
        """
        if self._loop is None or self._transport is None:
            return
        if self._transport.is_closing() or self._closed_event.is_set():
            return

        task = self._loop.create_task(self._kill_advertising(addr, patype))
        self._kill_tasks[addr] = task
        task.add_done_callback(lambda x: self._on_kill_done(x, addr=addr))

    def _on_kill_done(self, task: asyncio.Task, *, addr: str):
        self._kill_tasks.pop(addr, None)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            logger.warning(f"Kill task raised: {exc!r}")

    async def _kill_advertising(self, addr: str, patype: int):
        """Connect to ``addr`` and immediately disconnect.

        The CONNECT_IND sent by our controller during initiation is what
        actually stops the Flic from continuing its advertising burst.
        The follow-up Disconnect is just bookkeeping so the controller
        doesn't keep the (useless) link alive.

        ``patype`` is the peer address type taken from the advertising
        report (0=public, 1=random).
        """
        assert self._loop is not None

        async with self._kill_lock:
            logger.debug(f"Killing advertising for {addr}")

            connect_future: asyncio.Future[int] = self._loop.create_future()
            self._pending_connect_futures[addr] = connect_future
            try:
                # Parameters lifted from the BlueZ defaults; the actual
                # connection latency / timeout don't matter much because
                # we tear the link down on the first event.
                self._send_command(
                    HCI_Cmd_LE_Create_Connection(
                        interval=0x60,
                        window=0x60,
                        filter=0,  # use the peer address fields below
                        patype=patype,
                        paddr=addr,
                        atype=0,  # our address type — public
                        min_interval=0x18,
                        max_interval=0x28,
                        latency=0,
                        timeout=0x2A,
                        min_ce=0,
                        max_ce=0,
                    )
                )
                handle = await asyncio.wait_for(
                    connect_future, timeout=self._kill_timeout
                )
            except asyncio.TimeoutError:
                logger.warning(f"Kill connect to {addr} timed out, cancelling")
                self._pending_connect_futures.pop(addr, None)
                try:
                    self._send_command(HCI_Cmd_LE_Create_Connection_Cancel())
                except Exception as ce:
                    logger.debug(f"Create_Connection_Cancel failed: {ce}")
                return
            except BluetoothCommandError as e:
                logger.warning(f"Kill connect to {addr} failed: {e}")
                self._pending_connect_futures.pop(addr, None)
                return
            finally:
                # In every other exit path the handler has already popped;
                # this guards against leakage on cancellation.
                self._pending_connect_futures.pop(addr, None)

            disconnect_future: asyncio.Future[None] = (
                self._loop.create_future()
            )
            self._pending_disconnect_futures[handle] = disconnect_future
            try:
                self._send_command(
                    HCI_Cmd_Disconnect(
                        handle=handle,
                        reason=_DISCONNECT_REASON_REMOTE_USER,
                    )
                )
                await asyncio.wait_for(
                    disconnect_future, timeout=self._kill_timeout
                )
            except asyncio.TimeoutError:
                logger.warning(
                    f"Disconnect for handle={handle} ({addr}) timed out"
                )
            except BluetoothCommandError as e:
                logger.warning(f"Disconnect for {addr} failed: {e}")
            finally:
                self._pending_disconnect_futures.pop(handle, None)

            logger.debug(f"Done killing advertising for {addr}")

    ##############################################################
    # Command sender
    ##############################################################

    def _send_command(self, cmd: Packet):
        """Write a command to the transport without awaiting a response.

        Mirrors :meth:`FlicClient._send_command` in
        :mod:`tabletop_py.flic.client`: pure write, no future bookkeeping.
        """
        if self._transport is None:
            raise RuntimeError(
                "transport not yet connected, cannot send command"
            )
        pkt = HCI_Hdr() / HCI_Command_Hdr() / cmd
        logger.debug(f"Sending command: {pkt}")
        self._transport.write(bytes(pkt))

    async def send_command(self, cmd: Packet) -> Packet:
        """Send a command and await the matching Command Complete event.

        The future is registered *before* the write so a Command
        Complete arriving on the very next reader callback can never
        slip past us. Raises :class:`BluetoothCommandError` on
        non-zero status.
        """
        if self._loop is None:
            raise RuntimeError("Client not opened, cannot send command")

        pkt = HCI_Hdr() / HCI_Command_Hdr() / cmd
        opcode = pkt[HCI_Command_Hdr].opcode

        prev = self._pending_command_futures.get(opcode)
        if prev is not None and not prev.done():
            logger.warning(
                f"Another command with opcode 0x{opcode:04x} is still "
                "pending, cancelling and trying again"
            )
            prev.cancel()

        future: asyncio.Future[Packet] = self._loop.create_future()
        self._pending_command_futures[opcode] = future

        try:
            self._send_command(cmd)
            r = await future
        finally:
            if self._pending_command_futures.get(opcode) is future:
                del self._pending_command_futures[opcode]

        assert r.opcode == opcode
        if r.status != 0:
            raise BluetoothCommandError(
                "Command %x failed with %x" % (opcode, r.status)
            )
        return r

    ##############################################################
    # Scan configuration
    ##############################################################

    async def _configure_scan(
        self, *, active_scan: bool, scan_interval: int, scan_window: int
    ):
        await self.send_command(HCI_Cmd_Reset())
        await self.send_command(HCI_Cmd_LE_Set_Scan_Enable(enable=0))
        await self.send_command(HCI_Cmd_Set_Event_Filter())
        await self.send_command(HCI_Cmd_Set_Event_Mask())
        await self.send_command(HCI_Cmd_Read_LE_Host_Support())
        await self.send_command(HCI_Cmd_LE_Read_Buffer_Size_V1())
        await self.send_command(HCI_Cmd_LE_Read_Buffer_Size_V2())
        await self.send_command(
            HCI_Cmd_LE_Set_Scan_Parameters(
                type=1 if active_scan else 0,
                interval=scan_interval,
                window=scan_window,
            )
        )
        await self.send_command(
            HCI_Cmd_LE_Set_Scan_Enable(enable=1, filter_dups=0)
        )

    ##############################################################
    # Event waiters (mirror flicd client API)
    ##############################################################

    async def wait_for_any_button(self) -> ButtonPressInfo:
        """Await the next advertisement from any verified Flic button.

        Reuses an existing pending future if one is already registered,
        so multiple callers all wake up on the same press event.

        Returns:
            ButtonPressInfo for the first button press detected.
        """
        if (
            self._any_button_press_future is not None
            and not self._any_button_press_future.done()
        ):
            future = self._any_button_press_future
        else:
            future = asyncio.get_running_loop().create_future()
            self._any_button_press_future = future

        return await future

    async def wait_for_button(self, addr: str) -> ButtonPressInfo:
        """Await the next advertisement from a specific Flic button.

        Reuses an existing pending future for the address if one is
        already registered.

        Args:
            addr: Bluetooth address of the button to wait for
                (e.g., ``"80:e4:da:xx:xx:xx"``).

        Returns:
            ButtonPressInfo for the press from the specified button.
        """
        if (
            addr in self._button_press_futures
            and not self._button_press_futures[addr].done()
        ):
            future = self._button_press_futures[addr]
        else:
            future = asyncio.get_running_loop().create_future()
            self._button_press_futures[addr] = future

        info = await future
        assert info.addr == addr
        return info

    ##############################################################
    # Lifecycle
    ##############################################################

    def close(self):
        """Close the Flic client and underlying HCI socket.

        Cancels all pending kill tasks, disables LE scanning (if using
        user socket), and closes the transport. Safe to call multiple times.
        """
        # Set the closed flag *first* so the connection_lost callback
        # scheduled by transport.close() short-circuits cleanly.
        if self._closed_event.is_set():
            return
        self._closed_event.set()

        logger.info("Closing scapy flic client")

        for task in self._kill_tasks.values():
            task.cancel()

        if (
            self._transport is not None
            and not self._transport.is_closing()
            and self._socket_type == "user"
        ):
            try:
                self._send_command(HCI_Cmd_LE_Set_Scan_Enable(enable=0))
            except Exception as e:
                logger.warning(f"Failed to disable LE scan on close: {e}")

        if self._transport is not None:
            try:
                self._transport.close()
            except Exception as e:
                logger.debug(f"transport.close failed: {e}")

    @property
    def closed(self) -> bool:
        """Return True if the client has been closed."""
        return self._closed_event.is_set()

    async def wait_for_closed(self):
        """Await until the client is closed (i.e., ``close()`` has been called)."""
        await self._closed_event.wait()


async def main_async(
    *,
    device_id: int,
    active_scan: bool,
    kill_on_press: bool,
):
    """Run the Flic sniffer loop: create client, print each button press.

    Opens a ``FlicClient`` on the specified HCI adapter and loops
    indefinitely, logging each ``ButtonPressInfo`` received from any
    verified Flic button. Closes the client on exit (e.g., KeyboardInterrupt).

    Args:
        device_id: HCI controller index (``hciN``).
        active_scan: If True, send SCAN_REQ packets to advertisers.
        kill_on_press: If True, silence the button after each press via a
            brief connect+disconnect.
    """
    loop = asyncio.get_running_loop()
    client = await FlicClient.create(
        loop=loop,
        device_id=device_id,
        active_scan=active_scan,
        kill_on_press=kill_on_press,
    )
    logger.info(
        "Sniffing BLE advertisements for Flic button presses; "
        "press Ctrl-C to stop."
    )
    try:
        while True:
            packet_info = await client.wait_for_any_button()
            logger.info(f"Button pressed: {packet_info}")
    finally:
        client.close()


def main(args=None):
    """Command-line entry point for raw BLE Flic button sniffer.

    Parses command-line arguments and launches the async client to detect
    Flic button presses via direct BLE advertisement sniffing (bypassing
    the flicd daemon).

    Args:
        args: Optional list of command-line arguments. If None, uses sys.argv.
    """
    import argparse

    parser = argparse.ArgumentParser(
        description="Sniff Flic button presses via raw BLE advertisements."
    )
    parser.add_argument("--device-id", type=int, default=0)
    parser.add_argument("--active-scan", action="store_true")
    parser.add_argument(
        "--no-kill-on-press",
        action="store_true",
        help=(
            "After detecting an advertisement, the default behavior "
            "is to briefly connect to the button to force it to stop "
            "its press advertising burst. Use this flag to turn off "
            "this behavior."
        ),
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    ns = parser.parse_args(args)

    logging.basicConfig(
        level=logging.DEBUG if ns.verbose else logging.INFO,
        format="%(levelname)s - %(message)s",
    )

    try:
        asyncio.run(
            main_async(
                device_id=ns.device_id,
                active_scan=ns.active_scan,
                kill_on_press=not ns.no_kill_on_press,
            )
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
