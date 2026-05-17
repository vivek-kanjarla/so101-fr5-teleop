"""
quest3.py — reads 6-DOF controller pose from Meta Quest 3.

Two transport modes (set QUEST3_MODE in config.py):

  "oculus_reader"
      Uses the Oculus Reader APK (rail-berkeley/oculus_reader).
      Setup:
        1. Sideload the APK:  adb install oculus_reader.apk
        2. Forward the port:  adb forward tcp:5555 tcp:5555
        3. Open the app on Quest 3 — it starts streaming immediately.
      The APK sends length-prefixed pickled Python dicts at ~60 Hz.

  "udp"
      Receives a compact binary packet from a Quest streamer app
      (Hand Tracking Streamer or a custom Unity/OpenXR app).
      Setup:
        - Point your Quest app at this machine's IP on QUEST3_UDP_PORT.
        - The expected packet format is documented in _UDPTransport.

ControllerState is updated in a background thread at the source frame rate
(~60 Hz). The teleop loop runs at 125 Hz and reads the latest cached state.
If no new packet has arrived in QUEST3_STALE_MS ms, valid is set to False so
the teleop loop can hold position rather than issuing a stale command.
"""

import pickle
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from config import QUEST3_MODE, QUEST3_ADB_PORT, QUEST3_UDP_PORT, QUEST3_ACTIVE_HAND

QUEST3_STALE_MS = 200   # ms before a cached state is treated as stale


@dataclass
class ControllerState:
    """Pose + input state for one Quest 3 controller, in OpenXR convention.

    pos    — position in metres (x=right, y=up, z=back toward user)
    quat   — orientation quaternion [qx, qy, qz, qw]
    trigger — right/left trigger analog [0.0, 1.0]
    grip    — grip button analog        [0.0, 1.0]
    buttons — dict of named booleans (A, B, X, Y, menu, thumbstick_click)
    valid   — False until the first packet is received or if data has gone stale
    """
    pos:     np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat:    np.ndarray = field(default_factory=lambda: np.array([0., 0., 0., 1.]))
    trigger: float = 0.0
    grip:    float = 0.0
    buttons: dict  = field(default_factory=dict)
    valid:   bool  = False


# ── Oculus Reader transport ───────────────────────────────────────────────────

class _OculusReaderTransport:
    """
    Connects to the Oculus Reader APK via a TCP socket on localhost
    (port-forwarded from the Quest with `adb forward tcp:PORT tcp:PORT`).

    Wire format (big-endian):
      [uint32 length][bytes: pickled dict]

    The pickled dict has keys 'r' (right) and 'l' (left), each containing:
      pos    — list[float] length-3, metres
      rot    — list[float] length-4, quaternion [qx, qy, qz, qw]
      analog — dict: 'rightTrig'/'leftTrig' and 'rightGrip'/'leftGrip'
               each a single-element list [float]
      digital — dict of bool button states
    """

    def __init__(self, port: int):
        self._port   = port
        self._sock   = None
        self._buf    = b""

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        self._sock.connect(("127.0.0.1", self._port))
        self._sock.settimeout(0.5)
        self._buf = b""
        print(f"[QUEST3] Oculus Reader connected on localhost:{self._port}")

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def read_packet(self) -> dict | None:
        """Block until one complete packet is available; return decoded dict or None."""
        # Read 4-byte length header
        while len(self._buf) < 4:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionResetError("Oculus Reader socket closed")
            self._buf += chunk

        length = struct.unpack(">I", self._buf[:4])[0]
        self._buf = self._buf[4:]

        while len(self._buf) < length:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionResetError("Oculus Reader socket closed")
            self._buf += chunk

        data      = self._buf[:length]
        self._buf = self._buf[length:]
        return pickle.loads(data)

    @staticmethod
    def parse(raw: dict, hand: str) -> ControllerState:
        entry = raw.get(hand[0], {})  # 'r' or 'l'
        if not entry:
            return ControllerState()

        pos  = np.array(entry.get("pos", [0., 0., 0.]), dtype=np.float64)
        quat = np.array(entry.get("rot", [0., 0., 0., 1.]), dtype=np.float64)

        analog   = entry.get("analog", {})
        trig_key = "rightTrig" if hand == "right" else "leftTrig"
        grip_key = "rightGrip" if hand == "right" else "leftGrip"
        trigger  = float(analog.get(trig_key, [0.0])[0])
        grip     = float(analog.get(grip_key, [0.0])[0])

        digital  = entry.get("digital", {})
        buttons  = {k: bool(v) for k, v in digital.items()}

        return ControllerState(pos=pos, quat=quat, trigger=trigger, grip=grip,
                               buttons=buttons, valid=True)


# ── UDP transport ─────────────────────────────────────────────────────────────

# UDP packet format (little-endian, 52 bytes):
#   magic      uint32   0x51455354  ("QEST")
#   timestamp  float64  seconds since epoch
#   pos_x/y/z  float32  position in metres
#   qx/qy/qz/qw float32  quaternion
#   trigger    float32  [0, 1]
#   grip       float32  [0, 1]
#   buttons    uint32   bitmask (bit 0=A/X, 1=B/Y, 2=menu, 3=thumbstick)
_UDP_MAGIC  = 0x51455354
_UDP_FMT    = "<IdfffffffI"   # I d fffffff I
_UDP_SIZE   = struct.calcsize(_UDP_FMT)
_BTN_NAMES  = ["A_X", "B_Y", "menu", "thumbstick"]


class _UDPTransport:
    def __init__(self, port: int):
        self._port = port
        self._sock = None

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.settimeout(0.5)
        print(f"[QUEST3] UDP transport listening on 0.0.0.0:{self._port}")

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def read_packet(self) -> dict | None:
        try:
            data, _ = self._sock.recvfrom(256)
        except socket.timeout:
            return None
        if len(data) < _UDP_SIZE:
            return None
        fields = struct.unpack_from(_UDP_FMT, data)
        magic, ts, px, py, pz, qx, qy, qz, qw, trig, grip, btn_mask = fields
        if magic != _UDP_MAGIC:
            return None
        return dict(
            pos=[px, py, pz], rot=[qx, qy, qz, qw],
            trigger=trig, grip=grip,
            buttons={name: bool(btn_mask & (1 << i)) for i, name in enumerate(_BTN_NAMES)},
        )

    @staticmethod
    def parse(raw: dict, _hand: str) -> ControllerState:
        pos     = np.array(raw["pos"], dtype=np.float64)
        quat    = np.array(raw["rot"], dtype=np.float64)
        trigger = float(raw.get("trigger", 0.0))
        grip    = float(raw.get("grip",    0.0))
        buttons = raw.get("buttons", {})
        return ControllerState(pos=pos, quat=quat, trigger=trigger, grip=grip,
                               buttons=buttons, valid=True)


# ── Public reader class ───────────────────────────────────────────────────────

class Quest3Reader:
    """
    Thread-safe Quest 3 controller reader.

    Usage (context manager mirrors SO101Reader):
        with Quest3Reader() as quest:
            state = quest.get_state()   # ControllerState
    """

    def __init__(self):
        mode = QUEST3_MODE
        if mode == "oculus_reader":
            self._transport = _OculusReaderTransport(QUEST3_ADB_PORT)
        elif mode == "udp":
            self._transport = _UDPTransport(QUEST3_UDP_PORT)
        else:
            raise ValueError(f"Unknown QUEST3_MODE {mode!r} — expected 'oculus_reader' or 'udp'")

        self._hand       = QUEST3_ACTIVE_HAND
        self._state      = ControllerState()
        self._last_ts    = 0.0
        self._lock       = threading.Lock()
        self._stop_evt   = threading.Event()
        self._thread     = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def open(self):
        self._transport.connect()
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="quest3-reader")
        self._thread.start()
        # Block until first valid packet arrives (max 5 s)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._state.valid:
                    break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                "[QUEST3] No data received within 5 s — check ADB forward / Quest app is running"
            )

    def close(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._transport.close()

    # ── public API ────────────────────────────────────────────────────────────

    def get_state(self) -> ControllerState:
        """Return a copy of the latest ControllerState. valid=False if data is stale."""
        with self._lock:
            state = ControllerState(
                pos=self._state.pos.copy(),
                quat=self._state.quat.copy(),
                trigger=self._state.trigger,
                grip=self._state.grip,
                buttons=dict(self._state.buttons),
                valid=self._state.valid,
            )
        age_ms = (time.time() - self._last_ts) * 1000
        if age_ms > QUEST3_STALE_MS:
            state.valid = False
        return state

    # ── background thread ─────────────────────────────────────────────────────

    def _loop(self):
        consecutive_errors = 0
        while not self._stop_evt.is_set():
            try:
                raw = self._transport.read_packet()
                if raw is None:
                    continue
                state = self._transport.parse(raw, self._hand)
                with self._lock:
                    self._state   = state
                    self._last_ts = time.time()
                consecutive_errors = 0
            except (ConnectionResetError, BrokenPipeError) as exc:
                if self._stop_evt.is_set():
                    break
                print(f"[QUEST3] Connection lost ({exc}) — reconnecting in 1 s")
                self._transport.close()
                time.sleep(1.0)
                try:
                    self._transport.connect()
                except Exception as e:
                    print(f"[QUEST3] Reconnect failed: {e}")
            except Exception as exc:
                if self._stop_evt.is_set():
                    break
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 30 == 0:
                    print(f"[QUEST3] Read error #{consecutive_errors}: {exc!r}")

    # ── context manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
