"""Robot arm link server.

All socket handling lives here. The ESP32 is the client:

    1. ESP32 opens a TCP connection to this server and sends HELLO.
    2. Server replies HELLO_ACK and registers the client.
    3. Server may push parameters (SET_PARAMS) at any time.
    4. When ready, server sends ACTIVATE_REQ; the client answers
       ACTIVATE_ACK and UDP streaming begins.
    5. While active: client sends FEEDBACK periodically, server sends
       SETPOINTS periodically.
    6. Dropping the TCP session deactivates everything.

main.py imports MotorLink from here and does nothing with sockets itself.
"""

import os
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# --- Configuration -----------------------------------------------------
TCP_PORT = int(os.environ.get("TCP_PORT", "8001"))
UDP_PORT = int(os.environ.get("UDP_PORT", "8002"))

# A device that loses power outright never sends a FIN, so a plain recv()
# on its socket blocks forever and "connected" would stay true for a dead
# link. TCP keepalive makes the kernel probe the peer and fail the recv()
# if it stops answering: idle 2s, then a probe every 1s, 3 misses = dead
# within ~5s of the outage starting.
TCP_KEEPALIVE_IDLE_S = 2
TCP_KEEPALIVE_INTERVAL_S = 1
TCP_KEEPALIVE_COUNT = 3
AXIS_NUM = 6
PROTOCOL_VERSION = 1

SETPOINT_PERIOD_S = 0.02   # 50 Hz
FEEDBACK_STALE_S = 0.5     # no feedback for this long -> stale

# --- Message types (must match motor_comm_api.h) -----------------------
MSG_HELLO = 0x10
MSG_HELLO_ACK = 0x11
MSG_SET_PARAMS = 0x12
MSG_PARAMS_ACK = 0x13
MSG_ACTIVATE_REQ = 0x14
MSG_ACTIVATE_ACK = 0x15
MSG_SETPOINTS = 0x20
MSG_FEEDBACK = 0x21

# --- Framing -----------------------------------------------------------
HEADER_FMT = "<BBHI"            # type, version, length, seq       -> 8
HEADER_SIZE = struct.calcsize(HEADER_FMT)

HELLO_FMT = "<B"                # axis_count                       -> 1
PARAMS_FMT = "<3f2h"            # kp, ki, kd, out_min, out_max     -> 16
ACK_FMT = "<B"                  # ok                               -> 1
ACTIVATE_REQ_FMT = "<B"         # enable                           -> 1
ACTIVATE_ACK_FMT = "<2B"        # accepted, active                 -> 2

SETPOINTS_DATA_FMT = "<6i6h"    # positions, speed limits          -> 36
FEEDBACK_DATA_FMT = "<6i"       # positions                        -> 24
CHECKSUM_FMT = "<H"             # reserved, always 0               -> 2

FEEDBACK_SIZE = HEADER_SIZE + struct.calcsize(FEEDBACK_DATA_FMT) + 2  # 34


def pack(msg_type: int, data: bytes, seq: int, checksum: bool = False) -> bytes:
    """header + data (+ reserved checksum for UDP frames)."""
    frame = struct.pack(HEADER_FMT, msg_type, PROTOCOL_VERSION, len(data), seq) + data
    if checksum:
        frame += struct.pack(CHECKSUM_FMT, 0)  # reserved, not computed yet
    return frame


@dataclass
class Parameters:
    kp: float = 1.0
    ki: float = 0.2
    kd: float = 0.0
    output_min: int = -5000
    output_max: int = 5000

    def pack(self) -> bytes:
        return struct.pack(PARAMS_FMT, self.kp, self.ki, self.kd,
                           self.output_min, self.output_max)


@dataclass
class Status:
    connected: bool = False
    active: bool = False
    client_ip: Optional[str] = None
    axis_count: int = 0
    feedback_ok: bool = False
    positions: List[int] = field(default_factory=lambda: [0] * AXIS_NUM)
    setpoints: List[int] = field(default_factory=lambda: [0] * AXIS_NUM)
    parameters: Parameters = field(default_factory=Parameters)


class MotorLink:
    def __init__(self) -> None:
        self._lock = threading.Lock()

        self._client_sock: Optional[socket.socket] = None
        self._client_ip: Optional[str] = None
        self._axis_count = 0
        self._active = False

        self._positions = [0] * AXIS_NUM
        self._setpoints = [0] * AXIS_NUM
        self._speed_limits = [0] * AXIS_NUM  # reserved
        self._parameters = Parameters()
        self._last_feedback: Optional[float] = None

        self._tcp_seq = 0
        self._udp_seq = 0

        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.bind(("0.0.0.0", UDP_PORT))
        self._udp_sock.settimeout(0.5)

    # --- lifecycle ------------------------------------------------------
    def start(self) -> None:
        for target in (self._accept_loop, self._udp_rx_loop, self._udp_tx_loop):
            threading.Thread(target=target, daemon=True).start()

    # --- public API used by main.py --------------------------------------
    def status(self) -> Status:
        with self._lock:
            fresh = (self._last_feedback is not None
                     and time.monotonic() - self._last_feedback < FEEDBACK_STALE_S)
            return Status(
                connected=self._client_sock is not None,
                active=self._active,
                client_ip=self._client_ip,
                axis_count=self._axis_count,
                feedback_ok=fresh,
                positions=list(self._positions),
                setpoints=list(self._setpoints),
                parameters=self._parameters,
            )

    def set_setpoints(self, positions: List[int]) -> None:
        if len(positions) != AXIS_NUM:
            raise ValueError(f"expected {AXIS_NUM} positions")
        with self._lock:
            self._setpoints = list(positions)

    def set_parameters(self, params: Parameters) -> bool:
        """Pushes parameters to the client over TCP."""
        with self._lock:
            self._parameters = params
        return self._tcp_send(MSG_SET_PARAMS, params.pack())

    def activate(self, enable: bool) -> bool:
        """Asks the client to start or stop motor control."""
        if enable:
            # Never command a jump: start from where the arm actually is.
            with self._lock:
                self._setpoints = list(self._positions)
        return self._tcp_send(MSG_ACTIVATE_REQ,
                              struct.pack(ACTIVATE_REQ_FMT, 1 if enable else 0))

    # --- TCP ------------------------------------------------------------
    def _tcp_send(self, msg_type: int, data: bytes) -> bool:
        with self._lock:
            sock = self._client_sock
            seq = self._tcp_seq
            self._tcp_seq += 1
        if sock is None:
            return False
        try:
            sock.sendall(pack(msg_type, data, seq))
            return True
        except OSError:
            return False

    @staticmethod
    def _enable_keepalive(sock: socket.socket) -> None:
        """Makes a dead-without-closing peer (e.g. sudden power loss)
        show up as a recv() failure within a few seconds instead of
        hanging forever. TCP_KEEPIDLE/INTVL/CNT are Linux-only, so this
        degrades to plain SO_KEEPALIVE (OS default timing) elsewhere."""
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, TCP_KEEPALIVE_IDLE_S)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, TCP_KEEPALIVE_INTERVAL_S)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, TCP_KEEPALIVE_COUNT)
        except (AttributeError, OSError):
            pass

    def _accept_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", TCP_PORT))
        server.listen(1)
        print(f"[link] TCP server listening on :{TCP_PORT}")

        while True:
            try:
                sock, addr = server.accept()
            except OSError as exc:
                print(f"[link] accept failed: {exc}")
                time.sleep(1.0)
                continue

            self._enable_keepalive(sock)

            # One controller at a time; a new connection replaces the old.
            with self._lock:
                old = self._client_sock
                self._client_sock = sock
                self._client_ip = addr[0]
                self._active = False
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass

            print(f"[link] client connected from {addr[0]}")
            threading.Thread(target=self._client_loop, args=(sock,), daemon=True).start()

    def _recv_exact(self, sock: socket.socket, n: int) -> Optional[bytes]:
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _client_loop(self, sock: socket.socket) -> None:
        try:
            while True:
                head = self._recv_exact(sock, HEADER_SIZE)
                if head is None:
                    break
                msg_type, version, length, _seq = struct.unpack(HEADER_FMT, head)
                if version != PROTOCOL_VERSION or length > 64:
                    print(f"[link] bad frame (v{version} len {length}), closing")
                    break
                data = self._recv_exact(sock, length) if length else b""
                if data is None:
                    break
                self._handle_tcp(msg_type, data)
        except OSError as exc:
            print(f"[link] session error: {exc}")
        finally:
            with self._lock:
                if self._client_sock is sock:
                    self._client_sock = None
                    self._client_ip = None
                    self._active = False
                    self._axis_count = 0
            try:
                sock.close()
            except OSError:
                pass
            print("[link] client disconnected")

    def _handle_tcp(self, msg_type: int, data: bytes) -> None:
        if msg_type == MSG_HELLO:
            (axis_count,) = struct.unpack(HELLO_FMT, data)
            with self._lock:
                self._axis_count = axis_count
            print(f"[link] HELLO, {axis_count} axes")
            self._tcp_send(MSG_HELLO_ACK, b"")
            # Push the current parameters so both sides agree from the start.
            with self._lock:
                params = self._parameters
            self._tcp_send(MSG_SET_PARAMS, params.pack())

        elif msg_type == MSG_PARAMS_ACK:
            (ok,) = struct.unpack(ACK_FMT, data)
            print(f"[link] params {'accepted' if ok else 'rejected'}")

        elif msg_type == MSG_ACTIVATE_ACK:
            accepted, active = struct.unpack(ACTIVATE_ACK_FMT, data)
            with self._lock:
                self._active = bool(active)
            print(f"[link] activation {'accepted' if accepted else 'refused'}, "
                  f"active={bool(active)}")

        else:
            print(f"[link] ignoring TCP message type 0x{msg_type:02x}")

    # --- UDP ------------------------------------------------------------
    def _udp_rx_loop(self) -> None:
        while True:
            try:
                packet, addr = self._udp_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError as exc:
                print(f"[link] UDP recv error: {exc}")
                time.sleep(0.5)
                continue

            if len(packet) != FEEDBACK_SIZE:
                continue
            msg_type, version, length, _seq = struct.unpack(
                HEADER_FMT, packet[:HEADER_SIZE])
            if msg_type != MSG_FEEDBACK or version != PROTOCOL_VERSION:
                continue

            positions = struct.unpack(
                FEEDBACK_DATA_FMT, packet[HEADER_SIZE:HEADER_SIZE + length])
            # checksum is the trailing 2 bytes; reserved, not verified yet.

            with self._lock:
                if self._client_ip is None or addr[0] == self._client_ip:
                    self._positions = list(positions)
                    self._last_feedback = time.monotonic()

    def _udp_tx_loop(self) -> None:
        while True:
            time.sleep(SETPOINT_PERIOD_S)
            with self._lock:
                if not self._active or self._client_ip is None:
                    continue
                target = (self._client_ip, UDP_PORT)
                data = struct.pack(SETPOINTS_DATA_FMT,
                                   *self._setpoints, *self._speed_limits)
                seq = self._udp_seq
                self._udp_seq += 1
            try:
                self._udp_sock.sendto(
                    pack(MSG_SETPOINTS, data, seq, checksum=True), target)
            except OSError as exc:
                print(f"[link] UDP send error: {exc}")