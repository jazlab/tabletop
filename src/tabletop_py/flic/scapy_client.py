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
        bd_addrs=("aa:bb:cc:dd:ee:ff",),
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
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from fcntl import ioctl
from typing import Any, Literal, NamedTuple, Optional

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


@dataclass(slots=True)
class _ScanConfig:
    active_scan: bool = False
    # Interval/window are in 0.625 ms units (BT spec). 16 = 10 ms — a
    # tight, continuous scan suitable for response-time experiments.
    scan_interval: int = 16
    scan_window: int = 16


class PacketInfo(NamedTuple):
    bd_addr: str
    rssi: int | None
    time: Any


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
        if self._closing:
            raise RuntimeError("transport is closing, cannot write")
        n = self._sock.ins.send(data)
        if n <= 0:
            raise RuntimeError(f"HCI send returned {n}")
        return n

    def close(self):
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
        bd_addrs: Optional[Iterable[str]] = None,
        socket_type: Literal["hci", "user", "monitor"] = "user",
        time_fn: Callable[[], Any] = time.time,
        kill_on_press: bool = False,
        kill_timeout: float = 2.0,
    ):
        self.time = time_fn
        self._socket_type = socket_type
        self._kill_on_press = kill_on_press
        self._kill_timeout = kill_timeout

        # Normalize to lowercase so we can compare against scapy's
        # MAC formatting without surprises.
        self._bd_addrs: set[str] | None
        if not bd_addrs:
            logger.warning(
                "'bd_addrs' not provided or empty, listening for all advertisements"
            )
            self._bd_addrs = None
        else:
            self._bd_addrs = set(a.lower() for a in bd_addrs)

        self._loop: asyncio.AbstractEventLoop | None = None
        self._transport: _HCISocketTransport | None = None
        self._closed_event = asyncio.Event()

        self._pending_command_futures: dict[int, asyncio.Future[Packet]] = {}

        # Kill-on-press bookkeeping. The controller can only initiate
        # one LE connection at a time, so we serialize via _killing and
        # drop overlapping triggers.
        self._killing: bool = False
        self._kill_tasks: set[asyncio.Task] = set()
        # bd_addr (lowercased) -> future resolving to the connection
        # handle from LE_Meta_Connection_Complete. At most one entry.
        self._pending_connect_futures: dict[str, asyncio.Future[int]] = {}
        # connection_handle -> future resolving when Disconnection
        # Complete arrives.
        self._pending_disconnect_futures: dict[int, asyncio.Future[None]] = {}

        self._button_packet_info: dict[str, PacketInfo] = {}
        self._button_packet_events: dict[str, asyncio.Event] = {}

        self._any_button_packet_info: PacketInfo | None = None
        self._any_button_packet_event: asyncio.Event = asyncio.Event()
        self._any_button_packet_event.set()

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
        loop: asyncio.AbstractEventLoop,
        device_id: int = 0,
        bd_addrs: Optional[Iterable[str]] = None,
        time_fn: Callable[[], Any] = time.time,
        socket_type: Literal["hci", "user", "monitor"] = "user",
        active_scan: bool = False,
        scan_interval: int = 16,
        scan_window: int = 16,
        kill_on_press: bool = False,
        kill_timeout: float = 2.0,
    ) -> "FlicClient":
        """Open the HCI socket, install the Protocol, and start LE scanning.

        The shape mirrors ``loop.create_connection(lambda: FlicClient(...))``:
        we build the protocol, attach a transport, and (only on ``user``
        sockets) run the reset + LE-scan-enable sequence over that
        transport.

        Args:
            loop: Event loop to register the socket reader on.
            bd_addrs: Bluetooth addresses of buttons to listen for.
                Adverts from unlisted devices are silently dropped.
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
            bd_addrs=bd_addrs,
            socket_type=socket_type,
            time_fn=time_fn,
            kill_on_press=kill_on_press,
            kill_timeout=kill_timeout,
        )
        client._loop = loop
        # Side effect: synchronously invokes client.connection_made(),
        # which sets client._transport so _send_command works below.
        _HCISocketTransport(loop, sock, client)

        try:
            if isinstance(sock, BluetoothUserSocket):
                await client._configure_scan(
                    _ScanConfig(
                        active_scan=active_scan,
                        scan_interval=scan_interval,
                        scan_window=scan_window,
                    )
                )
        except BaseException:
            client.close()
            raise

        return client

    ##############################################################
    # asyncio.Protocol methods
    ##############################################################

    def connection_made(self, transport: asyncio.BaseTransport):
        logger.debug(f"Connection made to {type(transport)}: {transport}")
        assert isinstance(transport, _HCISocketTransport)
        self._transport = transport

    def data_received(self, data: bytes):
        event_time = self.time()
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
        logger.info("EOF received on HCI socket")
        self.close()

    def connection_lost(self, exc: Exception | None):
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

        if HCI_LE_Meta_Connection_Complete in pkt:
            self.on_connection_complete(pkt[HCI_LE_Meta_Connection_Complete])

        if HCI_Event_Disconnection_Complete in pkt:
            self.on_disconnection_complete(
                pkt[HCI_Event_Disconnection_Complete]
            )

    ##############################################################
    # Event handlers
    ##############################################################

    def on_command_complete(self, res: Packet):
        """Resolve the pending future for this opcode, if any."""
        opcode = res.opcode
        future = self._pending_command_futures.get(opcode)
        if future is None:
            logger.debug(
                f"Unsolicited Command Complete (opcode=0x{opcode:04x})"
            )
            return
        if future.done():
            return
        future.set_result(res)

    def on_command_status(self, res: Packet):
        """Surface failures for commands that emit Command Status only.

        LE_Create_Connection and Disconnect don't generate Command
        Complete — they return Command Status, then a follow-up event
        (Connection Complete / Disconnection Complete). A non-zero
        status here means the controller refused the command outright;
        we fail the matching pending future so the kill task aborts
        instead of waiting for an event that will never come.
        """
        if res.status == 0:
            return  # success — wait for the follow-up event
        logger.warning(
            f"Command Status: opcode=0x{res.opcode:04x} "
            f"status=0x{res.status:02x}"
        )
        if res.opcode == _OPCODE_LE_CREATE_CONNECTION:
            for bd_addr, future in list(self._pending_connect_futures.items()):
                if not future.done():
                    future.set_exception(
                        BluetoothCommandError(
                            f"LE_Create_Connection rejected: "
                            f"status=0x{res.status:02x}"
                        )
                    )
                self._pending_connect_futures.pop(bd_addr, None)
        elif res.opcode == _OPCODE_DISCONNECT:
            for handle, future in list(
                self._pending_disconnect_futures.items()
            ):
                if not future.done():
                    future.set_exception(
                        BluetoothCommandError(
                            f"Disconnect rejected: status=0x{res.status:02x}"
                        )
                    )
                self._pending_disconnect_futures.pop(handle, None)

    def on_connection_complete(self, info: Packet):
        """Resolve the pending connect future for the matching peer."""
        bd_addr = str(info.paddr).lower()
        future = self._pending_connect_futures.pop(bd_addr, None)
        if future is None:
            logger.debug(
                f"Unsolicited Connection Complete for {bd_addr} "
                f"handle={info.handle}"
            )
            return
        if future.done():
            return
        if info.status != 0:
            future.set_exception(
                BluetoothCommandError(
                    f"Connection to {bd_addr} failed: "
                    f"status=0x{info.status:02x}"
                )
            )
            return
        future.set_result(int(info.handle))

    def on_disconnection_complete(self, info: Packet):
        """Resolve the pending disconnect future for the matching handle."""
        future = self._pending_disconnect_futures.pop(int(info.handle), None)
        if future is None:
            logger.debug(
                f"Unsolicited Disconnection Complete handle={info.handle}"
            )
            return
        if not future.done():
            future.set_result(None)

    def filter_report(self, report: Packet) -> bool:
        bd_addr = str(report.addr).lower()
        if "80:e4:da" not in bd_addr and "90:88:a9" not in bd_addr:
            return False
        if self._bd_addrs is not None and bd_addr not in self._bd_addrs:
            return False
        return True

    def on_advertising_report(self, report: Packet, *, event_time: Any):
        """Forward an LE advertising report to the button waiters."""
        if not self.filter_report(report):
            return
        bd_addr = str(report.addr).lower()
        rssi = getattr(report, "rssi", None)
        packet_info = PacketInfo(bd_addr, rssi, event_time)
        self._dispatch_advertisement(packet_info)

        if self._kill_on_press:
            self._schedule_kill_advertising(
                bd_addr, int(getattr(report, "atype", 0))
            )

    def _dispatch_advertisement(self, packet_info: PacketInfo):
        bd_addr = packet_info.bd_addr

        if (
            bd_addr in self._button_packet_events
            and not self._button_packet_events[bd_addr].is_set()
        ):
            assert bd_addr not in self._button_packet_info
            self._button_packet_info[bd_addr] = packet_info
            self._button_packet_events[bd_addr].set()

        if not self._any_button_packet_event.is_set():
            assert self._any_button_packet_info is None
            self._any_button_packet_info = packet_info
            self._any_button_packet_event.set()

    ##############################################################
    # Kill-on-press flow
    ##############################################################

    def _schedule_kill_advertising(self, bd_addr: str, patype: int):
        """Fire a one-shot connect+disconnect to silence the Flic.

        Called synchronously from the reader callback so we can issue
        LE_Create_Connection on the very next event-loop tick. The
        controller only supports one initiating attempt at a time, so
        overlapping adverts (from a second button, or a repeat from the
        same one) are dropped until the in-flight kill finishes.
        """
        if self._killing:
            logger.debug(f"Kill already in flight, skipping {bd_addr}")
            return
        if self._loop is None or self._transport is None:
            return
        if self._transport.is_closing() or self._closed_event.is_set():
            return

        self._killing = True
        task = self._loop.create_task(self._kill_advertising(bd_addr, patype))
        self._kill_tasks.add(task)
        task.add_done_callback(self._on_kill_done)

    def _on_kill_done(self, task: asyncio.Task):
        self._kill_tasks.discard(task)
        self._killing = False
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            logger.warning(f"Kill task raised: {exc!r}")

    async def _kill_advertising(self, bd_addr: str, patype: int):
        """Connect to ``bd_addr`` and immediately disconnect.

        The CONNECT_IND sent by our controller during initiation is what
        actually stops the Flic from continuing its advertising burst.
        The follow-up Disconnect is just bookkeeping so the controller
        doesn't keep the (useless) link alive.

        ``patype`` is the peer address type taken from the advertising
        report (0=public, 1=random).
        """
        assert self._loop is not None
        logger.debug(f"Killing advertising for {bd_addr}")

        connect_future: asyncio.Future[int] = self._loop.create_future()
        self._pending_connect_futures[bd_addr] = connect_future
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
                    paddr=bd_addr,
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
            logger.warning(f"Kill connect to {bd_addr} timed out, cancelling")
            self._pending_connect_futures.pop(bd_addr, None)
            try:
                self._send_command(HCI_Cmd_LE_Create_Connection_Cancel())
            except Exception as ce:
                logger.debug(f"Create_Connection_Cancel failed: {ce}")
            return
        except BluetoothCommandError as e:
            logger.warning(f"Kill connect to {bd_addr} failed: {e}")
            self._pending_connect_futures.pop(bd_addr, None)
            return
        finally:
            # In every other exit path the handler has already popped;
            # this guards against leakage on cancellation.
            self._pending_connect_futures.pop(bd_addr, None)

        disconnect_future: asyncio.Future[None] = self._loop.create_future()
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
                f"Disconnect for handle={handle} ({bd_addr}) timed out"
            )
        except BluetoothCommandError as e:
            logger.warning(f"Disconnect for {bd_addr} failed: {e}")
        finally:
            self._pending_disconnect_futures.pop(handle, None)

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
        logger.debug(repr(r))
        return r

    ##############################################################
    # Scan configuration
    ##############################################################

    async def _configure_scan(self, cfg: _ScanConfig):
        await self.send_command(HCI_Cmd_Reset())
        await self.send_command(HCI_Cmd_LE_Set_Scan_Enable(enable=0))
        await self.send_command(HCI_Cmd_Set_Event_Filter())
        await self.send_command(HCI_Cmd_Set_Event_Mask())
        await self.send_command(HCI_Cmd_Read_LE_Host_Support())
        await self.send_command(HCI_Cmd_LE_Read_Buffer_Size_V1())
        await self.send_command(HCI_Cmd_LE_Read_Buffer_Size_V2())
        await self.send_command(
            HCI_Cmd_LE_Set_Scan_Parameters(
                type=1 if cfg.active_scan else 0,
                interval=cfg.scan_interval,
                window=cfg.scan_window,
            )
        )
        await self.send_command(
            HCI_Cmd_LE_Set_Scan_Enable(enable=1, filter_dups=0)
        )

    ##############################################################
    # Event waiters (mirror flicd client API)
    ##############################################################

    async def wait_for_any_button(self) -> PacketInfo:
        assert self._any_button_packet_info is None
        self._any_button_packet_event.clear()
        try:
            await self._any_button_packet_event.wait()
            packet_info = self._any_button_packet_info
            assert packet_info is not None
            return packet_info
        finally:
            self._any_button_packet_info = None

    async def wait_for_button(self, bd_addr: str) -> PacketInfo:
        assert bd_addr not in self._button_packet_events
        assert bd_addr not in self._button_packet_info
        self._button_packet_events[bd_addr] = asyncio.Event()
        try:
            await self._button_packet_events[bd_addr].wait()
            return self._button_packet_info[bd_addr]
        finally:
            del self._button_packet_info[bd_addr]

    ##############################################################
    # Lifecycle
    ##############################################################

    def close(self):
        # Set the closed flag *first* so the connection_lost callback
        # scheduled by transport.close() short-circuits cleanly.
        if self._closed_event.is_set():
            return
        self._closed_event.set()

        logger.info("Closing scapy flic client")

        # Cancel any in-flight kill tasks so they don't try to write to
        # a torn-down transport. The done_callback clears _killing.
        for task in list(self._kill_tasks):
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
        return self._closed_event.is_set()

    async def wait_for_closed(self):
        await self._closed_event.wait()


async def main_async(
    *,
    bd_addrs: Optional[list[str]],
    device_id: int,
    active_scan: bool,
    kill_on_press: bool,
):
    loop = asyncio.get_running_loop()
    client = await FlicClient.create(
        loop=loop,
        bd_addrs=bd_addrs,
        device_id=device_id,
        active_scan=active_scan,
        kill_on_press=kill_on_press,
    )
    logger.info(
        f"Sniffing advertisements from {len(bd_addrs) if bd_addrs else 'all'} "
        f"button(s); press Ctrl-C to stop."
    )
    try:
        while True:
            packet_info = await client.wait_for_any_button()
            logger.info(f"Button pressed: {packet_info}")
    finally:
        client.close()


DEFAULT_BD_ADDRS = [
    "90:88:a9:50:5f:b6",
    "90:88:a9:50:5f:db",
    "90:88:a9:50:5d:f7",
    "90:88:a9:50:5f:92",
    "90:88:a9:50:7b:a3",
    "90:88:a9:50:65:da",
    "90:88:a9:50:63:08",
    "90:88:a9:50:62:eb",
    "90:88:a9:50:66:0f",
    "90:88:a9:50:61:4e",
    "90:88:a9:50:5f:ac",
    "90:88:a9:50:60:11",
    "90:88:a9:50:60:9f",
    "90:88:a9:50:66:10",
    "90:88:a9:50:7c:cc",
    "90:88:a9:50:65:ff",
    "90:88:a9:50:7d:8f",
    "90:88:a9:50:5e:c2",
    "90:88:a9:50:60:09",
    "90:88:a9:50:61:a5",
    "90:88:a9:50:5f:70",
    "90:88:a9:50:7c:7a",
    "90:88:a9:50:5f:87",
    "90:88:a9:50:7c:9d",
    "90:88:a9:50:7c:b1",
    "90:88:a9:50:7c:ba",
    "90:88:a9:50:75:07",
    "90:88:a9:50:65:fb",
    "90:88:a9:50:66:09",
    "90:88:a9:50:5f:68",
]


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Sniff Flic button presses via raw BLE advertisements."
    )
    parser.add_argument(
        "--bd-addr",
        action="append",
        dest="bd_addrs",
        default=None,
        help="Verified Flic bd_addr to listen for (repeatable).",
    )
    parser.add_argument("--device-id", type=int, default=0)
    parser.add_argument("--active-scan", action="store_true")
    parser.add_argument(
        "--kill-on-press",
        action="store_true",
        help=(
            "After detecting an advertisement, briefly connect to the "
            "button to force it to stop its press advertising burst."
        ),
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    ns = parser.parse_args(args)

    # if ns.bd_addrs is None:
    #     print("yayyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
    #     ns.bd_addrs = DEFAULT_BD_ADDRS

    logging.basicConfig(
        level=logging.DEBUG if ns.verbose else logging.INFO,
        format="%(levelname)s - %(message)s",
    )

    try:
        asyncio.run(
            main_async(
                bd_addrs=ns.bd_addrs,
                device_id=ns.device_id,
                active_scan=ns.active_scan,
                kill_on_press=ns.kill_on_press,
            )
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
