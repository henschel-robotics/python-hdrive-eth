"""
HDrive motor — main interface.

This is the primary class users interact with. It manages the TCP
connection for commands and the UDP telemetry receiver.

Example::

    from hdrive_eth import HDriveETH

    with HDriveETH("192.168.1.102") as motor:
        motor.move_to(90)
        print(motor.telemetry)
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

from typing import Callable, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)

from .exceptions import (
    CommandError,
    ConnectionError,
    FirmwareVersionError,
    NotConnectedError
)
from .protocol import (
    Mode,
    TXTicket,
    build_can_c2_command,
    build_can_conf_command,
    build_can_conf_reset_command,
    build_can_pos_command,
    build_control_command,
)
from .telemetry import TelemetryPayload, TelemetryReceiver


# Default network ports
_TCP_COMMAND_PORT = 1000
_UDP_TELEMETRY_PORT = 1001


class HDriveETH:
    """Interface to an HDrive17-ETH servo drive.

    Args:
        ip: IP address of the HDrive (e.g. ``"192.168.1.102"``).
        tcp_port: TCP port for commands (default 1000).
        udp_port: UDP port for telemetry (default 1001).
        connect: If ``True`` (default), connect immediately on creation.
        telemetry_ticket: ``m4s22`` / ``TicketManager::TX_Ticket`` value for streamed telemetry.
            Default ``None`` selects ``TXTicket.BINARY`` (33×int32).
            Use ``TXTicket.BINARY_CAN_FULL`` for the 49×int32 CAN-full layout.
        telemetry_format: Passed to ``TelemetryReceiver`` — ``\"auto\"`` parses by UDP length,
            or ``\"binary_can_full\"`` / ``\"binary_can\"`` / ``\"debug\"`` / ``\"binary\"`` to enforce size.

    Example:

        # Context manager (recommended)
        with HDriveETH("192.168.1.102") as motor:
            motor.move_to(90)
            time.sleep(2)
            print(motor.telemetry)
    """

    def __init__(
        self,
        ip: str,
        tcp_port: Optional[int] = None,
        udp_port: Optional[int] = None,
        connect: bool = True,
        telemetry_ticket: Optional[int] = None,
        telemetry_format: str = "auto",
    ):
        self.ip = ip
        self.tcp_port = tcp_port or _TCP_COMMAND_PORT
        self.udp_port = udp_port or _UDP_TELEMETRY_PORT
        self._ports_from_user = (tcp_port is not None, udp_port is not None)
        self.telemetry_ticket = (
            TXTicket.BINARY if telemetry_ticket is None else telemetry_ticket
        )
        self.telemetry_format = telemetry_format

        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._telemetry: Optional[TelemetryReceiver] = None
        self._user_telemetry_callback: Optional[Callable] = None
        self._connected = False

        # Cached ``<canC2/>`` profile (RXConfigTicketCANAdvanced). 
        self._can_c2_master: Tuple[int, int, int] = (0, 0, 0)
        self._can_c2_slaves: List[Tuple[int, int, int]] = [(0, 0, 0) for _ in range(8)]

        if connect:
            self.connect()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the HDrive over TCP and start telemetry.

        During connection the driver will:
        1. Open a TCP socket for motion commands (port from ``tcp_port`` or default 1000).
        2. Read firmware version (m3s0); refuse if below the minimum supported version.
        3. Read UDP telemetry port from m4s17 unless ``udp_port`` was set explicitly.
        4. Enable UDP (m4s19), autosend (m4s34), and select TX telemetry ticket (``m4s22``, default ``TXTicket.BINARY``).
        5. Start the UDP telemetry receiver on the chosen UDP port.
        """
        if self._connected:
            return

        logger.info("Connecting to HDrive at %s (TCP %d, UDP %d) ...",
                     self.ip, self.tcp_port, self.udp_port)

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._socket.settimeout(5.0)
            self._socket.connect((self.ip, self.tcp_port))
            # Blocking sends: a global socket timeout also limits sendall(); fast objWrite
            # loops can fill the TCP window and spuriously hit that limit on long runs.
            self._socket.settimeout(None)
        except OSError as exc:
            self._socket = None
            raise ConnectionError(
                f"Could not connect to HDrive at {self.ip}:{self.tcp_port} — {exc}"
            ) from exc

        self._connected = True

        # Check firmware version (m3s0) — must be >= 266
        self._check_firmware_version()

        # Read UDP port from drive if user didn't set it
        self._read_udp_port()

        # Ensure UDP is enabled, autosend is on, and binary ticket is selected
        self._configure_telemetry()
        self._telemetry = TelemetryReceiver(
            port=self.udp_port,
            callback=self._user_telemetry_callback,
            telemetry_format=self.telemetry_format,
        )
        self._telemetry.start()

    _MIN_FIRMWARE_VERSION = 266

    def _check_firmware_version(self) -> None:
        """Read firmware version (m3s0) and abort if too old."""
        try:
            version = self.read_object(index=3, subindex=0)
            logger.info("Firmware version (m3s0): %d", version)
        except CommandError as exc:
            self.close()
            raise FirmwareVersionError(
                f"Could not read firmware version (m3s0): {exc}"
            ) from exc

        if version < self._MIN_FIRMWARE_VERSION:
            self.close()
            raise FirmwareVersionError(
                f"Firmware version {version} is too old. "
                f"Minimum required: {self._MIN_FIRMWARE_VERSION}. "
                f"Please update the HDrive17-ETH firmware."
            )

    def _read_udp_port(self) -> None:
        """Read UDP port from the drive (m4s17) via TCP."""
        if self._ports_from_user[1]:
            return
        try:
            port = self.read_object(index=4, subindex=17)
            if port > 0:
                self.udp_port = port
                logger.info("UDP port read from drive (m4s17): %d", port)
        except Exception as exc:
            logger.debug("Could not read m4s17 (UDP port), using default %d: %s",
                         self.udp_port, exc)

    def _configure_telemetry(self) -> None:
        """Check UDP comm + autosend are enabled and select binary-ticket."""
        # Check m4s19 — UDP communication enabled
        logger.debug("Writing m4s19 = 1 (UDP communication enabled) ...")
        try:
            self.write_object(index=4, subindex=19, value=1)
        except CommandError as exc:
            logger.warning("Could not write m4s19 (UDP communication flag): %s", exc)

        time.sleep(0.1)

        # Check m4s34 — autosend enabled
        logger.debug("Writing m4s34 = 1 (autosend enabled) ...")
        try:
            self.write_object(index=4, subindex=34, value=1)
        except CommandError as exc:
            logger.warning("Could not write m4s34 (autosend flag): %s", exc)

        time.sleep(0.1)

        # m4s22 = TicketManager::TX_Ticket (see ``hdrive_eth.protocol.TXTicket``).
        logger.debug("Writing m4s22 = %d ...", self.telemetry_ticket)
        try:
            self.write_object(index=4, subindex=22, value=self.telemetry_ticket)
            logger.info("m4s22 set to %d (TX telemetry ticket)", self.telemetry_ticket)
        except CommandError as exc:
            logger.warning(
                "Could not write m4s22 = %d: %s. "
                "Telemetry parsing may fail if the ticket format doesn't match.",
                self.telemetry_ticket,
                exc,
            )

    def close(self) -> None:
        """Stop the motor, close the connection, and stop telemetry.

        Automatically called when leaving a ``with`` block or when the
        object is garbage-collected.
        """
        if self._connected:
            try:
                cmd = build_control_command(
                    position=0,
                    speed=0,
                    torque=0,
                    mode=0,
                    acc=0,
                    decc=0,
                )
                self._send(cmd)
            except Exception:
                pass

        if self._telemetry is not None:
            self._telemetry.stop()

        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def __enter__(self) -> "HDriveETH":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @property
    def telemetry(self) -> Optional[TelemetryPayload]:
        """The latest telemetry frame, or ``None`` if no data received yet."""
        if self._telemetry is None:
            return None
        return self._telemetry.latest

    def on_telemetry(self, callback: Callable[[TelemetryPayload], None]) -> None:
        """Register a callback for every telemetry frame.

        Args:
            callback: Function that receives each ``TelemetryPayload``.

        Example::

            def print_position(frame):
                print(f"Position: {frame.position}")

            motor.on_telemetry(print_position)
        """
        if self._telemetry is not None:
            self._telemetry.callback = callback
        self._user_telemetry_callback = callback

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------

    def move_to(
        self,
        position: int,
        speed: int = 100,
        torque: int = 200,
        acc: int = 5000,
        decc: int = 5000,
    ) -> None:
        """Move to an absolute position in degrees.

        Args:
            position: Target position in degrees.
            speed: Target speed value.
            torque: Torque limit (**mNm**, same unit as object dictionary ``demandedTorque``).
            acc: Acceleration ramp value.
            decc: Deceleration ramp value.
        """
        cmd = build_control_command(
            position=position,
            speed=speed,
            torque=torque,
            mode=Mode.POSITION_CONTROL,
            acc=acc,
            decc=decc,
        )
        self._send(cmd)

    def set_speed(
        self,
        speed: int,
        torque: int = 200,
        acc: int = 5000,
        decc: int = 5000,
    ) -> None:
        """Run at a constant speed (velocity mode).

        Args:
            speed: Target speed value.
            torque: Torque limit (**mNm**). Default 200.
            acc: Acceleration ramp value.
            decc: Deceleration ramp value.
        """
        cmd = build_control_command(
            speed=speed,
            torque=torque,
            mode=Mode.VELOCITY_CONTROL,
            acc=acc,
            decc=decc,
        )
        self._send(cmd)

    def set_torque(self, torque: int) -> None:
        """Run in torque-only mode.

        Args:
            torque: Torque setpoint (**mNm**; written to ``demandedTorque``).
        """
        # Explicit zero pos/speed: build_control_command defaults include speed=500,
        # which would otherwise stay on the wire and can keep velocity/position loops engaged.
        cmd = build_control_command(
            position=0,
            speed=0,
            torque=torque,
            mode=Mode.TORQUE_CONTROL,
            acc=0,
            decc=0,
        )
        self._send(cmd)

    def stop(self) -> None:
        """Stop the motor by setting mode to 0."""
        cmd = build_control_command(
            position=0,
            speed=0,
            torque=0,
            mode=0,
            acc=0,
            decc=0,
        )
        self._send(cmd)

    # ------------------------------------------------------------------
    # CAN TCP tickets (firmware CommRX_Tickets: canPos, canC2, canConf)
    # ------------------------------------------------------------------

    def stop_master_and_can_slaves(self, *chain_slots: int) -> None:
        self.stop()
        if not chain_slots:
            return
        try:
            self._send(build_can_conf_reset_command())
        except Exception:
            logger.exception("stop_master_and_can_slaves failed")

    def set_can_master_and_slave_target_profile(
        self,
        master_triplet: Tuple[int, int, int],
        *slave_triplets: Tuple[int, int, int],
    ) -> None:
        """Send ``<canC2/>``: ``master_triplet`` is Ethernet master ``(speed, acc, dec)``.

        Each ``slave_triplets`` entry is slave 1, slave 2, … in order (at most eight).
        Omitted slaves are sent as ``(0, 0, 0)``.
        """
        if len(master_triplet) != 3:
            raise ValueError(f"master_triplet must be length 3, got {master_triplet!r}")
        if len(slave_triplets) > 8:
            raise ValueError(f"at most 8 slave triplets, got {len(slave_triplets)}")
        for t in slave_triplets:
            if len(t) != 3:
                raise ValueError(f"each slave triplet must be length 3, got {t!r}")

        self._can_c2_master = (
            int(master_triplet[0]),
            int(master_triplet[1]),
            int(master_triplet[2]),
        )

        new_slaves = [(0, 0, 0) for _ in range(8)]
        for i, t in enumerate(slave_triplets):
            if i >= 8:
                break
            new_slaves[i] = (int(t[0]), int(t[1]), int(t[2]))
        self._can_c2_slaves = new_slaves

        self._send(
            build_can_c2_command(
                self._can_c2_master[0],
                self._can_c2_master[1],
                self._can_c2_master[2],
                self._can_c2_slaves,
            )
        )

    def send_can_pos(self, master_deg: float, *slave_deg: float) -> None:
        self._send(build_can_pos_command(master_deg, *slave_deg))

    def send_can_c2(
        self,
        master_speed: int,
        master_acc: int,
        master_decc: int,
        *slave_triplets: Tuple[int, int, int],
    ) -> None:
        self._send(
            build_can_c2_command(
                master_speed, master_acc, master_decc, list(slave_triplets)
            )
        )

    def set_can_master_and_slave_configuration(
        self,
        master_mode: int,
        master_torque: int,
        slave_mode: Sequence[int],
        slave_torque: Sequence[int],
    ) -> None:
        """Send ``<canConf/>`` (firmware ``RXConfigTicketCAN`` / ``parseTicketInt32s``).

        Wire layout (26 ints, ``RXConfigTicketCAN.h``): ``demandedTorque``, ``demandedMode``,
        then **eight** ``slaveN_targetCurrent`` (**mNm**), then **eight** ``slaveN_targetMode``,
        then eight ``slaveN_specialCommand`` (use ``0`` / ``NoCommand``).

        Pass **slave modes** then **slave torques** (same order as master mode/torque) to avoid
        swapping the two lists at the call site. Each list pads to eight entries.
        """
        sm = [int(x) for x in slave_mode]
        st = [int(x) for x in slave_torque]
        if len(st) > 8 or len(sm) > 8:
            raise ValueError(
                f"slave_mode and slave_torque must have at most 8 entries, got {len(sm)} and {len(st)}"
            )
        while len(st) < 8:
            st.append(0)
        while len(sm) < 8:
            sm.append(0)
        special = [0] * 8
        values = [int(master_torque), int(master_mode), *st, *sm, *special]
        self._send(build_can_conf_command(values))

    def set_can_stop(self) -> None:
        self.set_can_master_and_slave_configuration(0, 0, [0] * 8, [0] * 8)

    # ------------------------------------------------------------------
    # Object read / write
    # ------------------------------------------------------------------

    def read_object(self, index: int, subindex: int) -> int:
        """Read a single object from the drive (blocking).

        Automatically reconnects the TCP socket if the drive closed
        the connection (the embedded TCP stack may close after each
        read response).

        Args:
            index: Object index.
            subindex: Object sub-index.

        Returns:
            The integer value of the object.

        Raises:
            CommandError: If the request fails or the drive returns an error.
        """
        # Try up to 2 times — reconnect once if the connection was closed.
        for attempt in range(2):
            resp = self._try_read_object(index, subindex)
            if resp is not None:
                break
            # Connection was closed — reconnect and retry
            logger.debug("Reconnecting TCP for objRead m%ds%d (attempt %d) ...",
                         index, subindex, attempt + 2)
            self._reconnect_tcp()
        else:
            raise CommandError(
                f"Failed to read m{index}s{subindex} after reconnect"
            )

        import re
        logger.debug("objRead m%ds%d response: %s", index, subindex, resp)

        if "error=" in resp:
            raise CommandError(f"Drive returned error for m{index}s{subindex}: {resp}")

        # New format: <r a="4" b="22" v="3" />
        value_match = re.search(r'v="(-?\d+)"', resp)
        if value_match:
            return int(value_match.group(1))

        raise CommandError(
            f"Failed to parse read response for m{index}s{subindex}: {resp}"
        )

    def _try_read_object(self, index: int, subindex: int) -> Optional[str]:
        """Send an objRead request and return the response, or None if the
        connection was closed."""
        import re
        xml = f'<objRead a="{index}" b="{subindex}" />'
        with self._lock:
            sock = self._socket
            prev_timeout = sock.gettimeout()
            try:
                sock.settimeout(5.0)
                sock.sendall(xml.encode("ascii"))

                buf = ""
                while True:
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        logger.debug("objRead m%ds%d: recv timed out", index, subindex)
                        return None
                    if not chunk:
                        # Drive closed the connection
                        logger.debug("objRead m%ds%d: connection closed by drive",
                                     index, subindex)
                        return None
                    buf += chunk.decode("ascii", errors="replace")

                    # Match new format: <r a="..." b="..." v="..." />
                    match = re.search(r'<r\s[^>]*/>', buf)
                    if match:
                        return match.group(0)
            except OSError as exc:
                logger.debug("objRead m%ds%d: OSError %s", index, subindex, exc)
                return None
            finally:
                sock.settimeout(prev_timeout)

    def _reconnect_tcp(self) -> None:
        """Close and re-open the TCP socket."""
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._socket.settimeout(5.0)
            self._socket.connect((self.ip, self.tcp_port))
            self._socket.settimeout(None)
            self._connected = True
        except OSError as exc:
            self._connected = False
            raise CommandError(
                f"Failed to reconnect TCP to {self.ip}:{self.tcp_port} — {exc}"
            ) from exc

    def write_object(self, index: int, subindex: int, value: int) -> None:
        """Write a single object to the drive.

        Args:
            index: Object index.
            subindex: Object sub-index.
            value: Value to write.

        Raises:
            CommandError: If the request fails.
        """
        xml = f'<objWrite a="{index}" b="{subindex}" c="{value}" />'
        with self._lock:
            try:
                self._socket.sendall(xml.encode("ascii"))
                # Write handler returns 0 (no response).
                # Small delay so the embedded TCP stack can process before
                # the next command arrives.
                time.sleep(0.001)
            except OSError as exc:
                self._connected = False
                raise CommandError(
                    f"Failed to write object m{index}s{subindex}={value} — {exc}"
                ) from exc

    # ------------------------------------------------------------------
    # CAN slave object gateway (HTTP — firmware handles transaction)
    # ------------------------------------------------------------------

    @staticmethod
    def _slvobj_http_get(master_ip: str, ticket: str, timeout: float) -> str:
        url = f"http://{master_ip}/getData.cgi?slvobj={ticket}"
        try:
            with urlopen(url, timeout=timeout) as resp:
                return resp.read().decode("utf-8", errors="replace")
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise CommandError(f"slvobj HTTP {exc.code}: {body}") from exc
        except URLError as exc:
            raise ConnectionError(f"slvobj HTTP GET failed: {exc}") from exc

    @staticmethod
    def read_slvobj(
        master_ip: str,
        slot: int,
        main_key: int,
        sub_key: int,
        timeout: float,
    ) -> str:
        """GET ``getData.cgi?slvobj=r_<slot>_<mainKey>_<subKey>`` (synchronous in firmware)."""
        ticket = f"r_{int(slot)}_{int(main_key)}_{int(sub_key)}"
        return HDriveETH._slvobj_http_get(master_ip, ticket, timeout)

    @staticmethod
    def write_slvobj(
        master_ip: str,
        slot: int,
        main_key: int,
        sub_key: int,
        value: int,
        timeout: float,
    ) -> str:
        """GET ``getData.cgi?slvobj=w_<slot>_<mainKey>_<subKey>_<value>`` (synchronous in firmware)."""
        ticket = f"w_{int(slot)}_{int(main_key)}_{int(sub_key)}_{int(value)}"
        return HDriveETH._slvobj_http_get(master_ip, ticket, timeout)

    def send_raw(
        self,
        position: int = 0,
        speed: int = 0,
        torque: int = 200,
        mode: int = Mode.POSITION_CONTROL,
        acc: int = 0,
        decc: int = 0,
    ) -> None:
        """Send a raw control command with all parameters.

        Use this if the high-level methods don't cover your use case.

        Args:
            position: Position setpoint in degrees (encoded as ``degrees × 10`` on the wire).
            speed: Speed setpoint.
            torque: Torque limit (**mNm**).
            mode: Control mode byte (``Mode`` constants in ``hdrive_eth.protocol``).
            acc: Acceleration ramp value.
            decc: Deceleration ramp value.
        """
        cmd = build_control_command(
            position=position,
            speed=speed,
            torque=torque,
            mode=mode,
            acc=acc,
            decc=decc,
        )
        self._send(cmd)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send(self, data: bytes) -> None:
        """Send raw bytes over TCP (thread-safe)."""
        if not self._connected or self._socket is None:
            raise NotConnectedError("Not connected to HDriveETH. Call connect() first.")

        with self._lock:
            try:
                self._socket.sendall(data)
            except OSError as exc:
                self._connected = False
                raise CommandError(f"Failed to send command — {exc}") from exc
