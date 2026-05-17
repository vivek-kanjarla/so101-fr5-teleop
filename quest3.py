"""
quest3.py — reads 6-DOF controller pose from Meta Quest 3.

Three transport modes (set QUEST3_MODE in config.py):

  "oculus_reader"
      Uses the jborbik/oculus_reader APK (Quest 3 compatible fork).
      Setup:
        1. Sideload the APK:  adb install oculus_reader.apk
        2. Forward the port:  adb forward tcp:5555 tcp:5555
        3. Open the app on Quest 3 — it starts streaming immediately.
      Wire format: [uint32 length][pickled dict] over TCP.

  "vuer"
      Uses the Vuer WebXR library (pip install vuer).
      No APK needed — Quest 3 browser connects to a local HTTPS server.
      Setup:
        1. pip install vuer
        2. Generate SSL cert (see docs/quest3_setup.md)
        3. Point the Quest 3 browser at https://<your-pc-ip>:8012
        4. Accept the self-signed cert warning and grant controller access.
      Event: CONTROLLER_MOVE — 4×4 column-major SE(3) matrix per controller.

  "udp"
      Custom binary packet streamer. Packet format documented in _UDPTransport.

ControllerState is updated by a background worker (thread or process depending
on mode). If no new packet has arrived in QUEST3_STALE_MS ms, valid is set to
False so the teleop loop holds position rather than issuing a stale command.
"""

import multiprocessing
import pickle
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from config import (
    QUEST3_MODE, QUEST3_ADB_PORT, QUEST3_UDP_PORT, QUEST3_ACTIVE_HAND,
    QUEST3_VUER_PORT, QUEST3_VUER_CERT, QUEST3_VUER_KEY,
)

QUEST3_STALE_MS = 200   # ms before a cached state is treated as stale

# Shared-array layout for Vuer process ↔ reader thread:
#   [0..2]  pos x, y, z   (metres)
#   [3..6]  quat qx,qy,qz,qw
#   [7]     trigger value  [0, 1]
#   [8]     squeeze value  [0, 1]
#   [9]     valid flag     (1.0 once first event received)
_VUER_SHARED_LEN = 10


# ── Quaternion / rotation helpers ────────────────────────────────────────────

@dataclass
class ControllerState:
    """Pose + input state for one Quest 3 controller, in OpenXR convention.

    pos     — position in metres (x=right, y=up, z=back toward user)
    quat    — orientation quaternion [qx, qy, qz, qw]
    trigger — analog trigger [0.0 released → 1.0 fully pulled]
    grip    — grip button analog [0.0, 1.0]
    buttons — dict of named booleans (A/X, B/Y, menu, thumbstick)
    valid   — False until first packet received or if data has gone stale
    """
    pos:     np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat:    np.ndarray = field(default_factory=lambda: np.array([0., 0., 0., 1.]))
    trigger: float = 0.0
    grip:    float = 0.0
    buttons: dict  = field(default_factory=dict)
    valid:   bool  = False


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3×3 rotation matrix to quaternion [qx, qy, qz, qw] (Shepperd method)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


# ── Vuer server process (top-level so multiprocessing can pickle it) ─────────

def _run_vuer_server(port: int, cert: str, key: str, hand: str,
                     shared: "multiprocessing.Array") -> None:  # noqa: F821
    """
    Runs inside a separate Process. Starts a Vuer HTTPS WebXR server, listens
    for CONTROLLER_MOVE events, and writes parsed pose into shared memory.

    shared layout: [px, py, pz, qx, qy, qz, qw, trigger, squeeze, valid_flag]
    """
    import numpy as _np
    from vuer import Vuer

    hand_key  = "right"      if hand == "right" else "left"
    state_key = "rightState" if hand == "right" else "leftState"

    app = Vuer(host="0.0.0.0", port=port, cert=cert, key=key, queries=dict(grid=False))

    @app.add_handler("CONTROLLER_MOVE")
    async def on_controller_move(event, session):  # noqa: ANN
        try:
            raw   = event.value.get(hand_key)
            state = event.value.get(state_key, {})
            if not raw or len(raw) < 16:
                return

            # 16-element column-major SE(3) matrix → numpy (row-major)
            mat = _np.array(raw, dtype=_np.float64).reshape(4, 4, order="F")
            pos  = mat[:3, 3]                   # metres (OpenXR convention)
            quat = _rotmat_to_quat(mat[:3, :3]) # [qx, qy, qz, qw]

            # triggerValue: 0.0 = not pressed, 1.0 = fully pressed (WebXR spec)
            trigger = float(state.get("triggerValue", 0.0))
            squeeze = float(state.get("squeezeValue", 0.0))

            with shared.get_lock():
                shared[0] = pos[0];  shared[1] = pos[1];  shared[2] = pos[2]
                shared[3] = quat[0]; shared[4] = quat[1]
                shared[5] = quat[2]; shared[6] = quat[3]
                shared[7] = trigger; shared[8] = squeeze
                shared[9] = 1.0     # valid flag
        except Exception:
            pass

    app.run()


# ── Oculus Reader transport (TCP/ADB) ─────────────────────────────────────────

class _OculusReaderTransport:
    """
    Connects to the jborbik/oculus_reader APK (Quest 3 compatible fork)
    via a TCP socket port-forwarded from the Quest with:
        adb forward tcp:5555 tcp:5555

    Wire format (big-endian):
      [uint32 length][bytes: pickled dict]

    Pickled dict keys per controller ('r' / 'l'):
      pos    — list[float] length-3, metres
      rot    — list[float] length-4, quaternion [qx, qy, qz, qw]
      analog — {'rightTrig': [float], 'rightGrip': [float], ...}
      digital — dict of bool button states
    """

    def __init__(self, port: int):
        self._port = port
        self._sock = None
        self._buf  = b""

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
        while len(self._buf) < 4:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionResetError("Oculus Reader socket closed")
            self._buf += chunk

        length    = struct.unpack(">I", self._buf[:4])[0]
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
        entry = raw.get(hand[0], {})
        if not entry:
            return ControllerState()

        pos  = np.array(entry.get("pos", [0., 0., 0.]), dtype=np.float64)
        quat = np.array(entry.get("rot", [0., 0., 0., 1.]), dtype=np.float64)

        analog   = entry.get("analog", {})
        trig_key = "rightTrig" if hand == "right" else "leftTrig"
        grip_key = "rightGrip" if hand == "right" else "leftGrip"
        trigger  = float(analog.get(trig_key, [0.0])[0])
        grip     = float(analog.get(grip_key, [0.0])[0])

        buttons = {k: bool(v) for k, v in entry.get("digital", {}).items()}
        return ControllerState(pos=pos, quat=quat, trigger=trigger, grip=grip,
                               buttons=buttons, valid=True)


# ── Vuer transport (WebXR via Quest browser) ──────────────────────────────────

class _VuerTransport:
    """
    Starts a Vuer HTTPS server in a subprocess. Quest 3 browser navigates to
    https://<pc-ip>:<QUEST3_VUER_PORT>, enters an XR session, and streams
    CONTROLLER_MOVE events. Pose is written into a multiprocessing.Array so
    the main process can read it without blocking.

    SSL is mandatory for WebXR. Generate a self-signed cert once:
        openssl req -x509 -newkey rsa:4096 -nodes \\
          -out ssl/cert.pem -keyout ssl/key.pem -days 365 \\
          -subj "/CN=quest-teleop"
    Then accept the security warning in the Quest browser (once per cert).

    See docs/quest3_setup.md → Vuer section for full instructions.
    """

    def __init__(self, port: int, cert: str, key: str, hand: str):
        self._port    = port
        self._cert    = cert
        self._key     = key
        self._hand    = hand
        self._shared  = multiprocessing.Array("d", _VUER_SHARED_LEN)
        self._process = None

    def connect(self):
        try:
            import vuer  # noqa: F401
        except ImportError:
            raise ImportError(
                "[QUEST3] Vuer not installed — run: pip install vuer\n"
                "         Then follow docs/quest3_setup.md → Vuer section."
            )

        import os
        for path, label in [(self._cert, "QUEST3_VUER_CERT"), (self._key, "QUEST3_VUER_KEY")]:
            if not os.path.isfile(path):
                raise FileNotFoundError(
                    f"[QUEST3] SSL file not found: {path!r} (set {label} in config.py)\n"
                    "         Generate with: openssl req -x509 -newkey rsa:4096 -nodes "
                    "-out ssl/cert.pem -keyout ssl/key.pem -days 365 -subj '/CN=quest-teleop'"
                )

        self._process = multiprocessing.Process(
            target=_run_vuer_server,
            args=(self._port, self._cert, self._key, self._hand, self._shared),
            daemon=True,
        )
        self._process.start()
        print(
            f"[QUEST3] Vuer server started — open on Quest 3 browser:\n"
            f"         https://<this-pc-ip>:{self._port}\n"
            f"         Accept the self-signed cert warning, then tap 'Enter VR'."
        )

    def close(self):
        if self._process and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2)
        self._process = None

    def read_packet(self) -> dict | None:
        time.sleep(0.01)   # ~100 Hz max poll — Vuer events arrive at ~60–90 Hz
        with self._shared.get_lock():
            valid = self._shared[9] > 0.5
            if not valid:
                return None
            data = list(self._shared)
        return {
            "pos":     data[0:3],
            "rot":     data[3:7],
            "trigger": data[7],
            "squeeze": data[8],
        }

    @staticmethod
    def parse(raw: dict, _hand: str) -> ControllerState:
        pos     = np.array(raw["pos"],  dtype=np.float64)
        quat    = np.array(raw["rot"],  dtype=np.float64)
        trigger = float(raw["trigger"])
        grip    = float(raw["squeeze"])
        return ControllerState(pos=pos, quat=quat, trigger=trigger, grip=grip,
                               buttons={}, valid=True)


# ── UDP transport (custom streamer app) ───────────────────────────────────────

# Packet format (little-endian, 52 bytes):
#   magic      uint32   0x51455354  ("QEST")
#   timestamp  float64  seconds since epoch
#   pos_x/y/z  float32  position in metres
#   qx/qy/qz/qw float32  quaternion
#   trigger    float32  [0, 1]
#   grip       float32  [0, 1]
#   buttons    uint32   bitmask (bit 0=A/X, 1=B/Y, 2=menu, 3=thumbstick)
_UDP_MAGIC  = 0x51455354
_UDP_FMT    = "<IdfffffffI"
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

    Usage (context manager):
        with Quest3Reader() as quest:
            state = quest.get_state()   # ControllerState
    """

    def __init__(self):
        mode = QUEST3_MODE
        if mode == "oculus_reader":
            self._transport = _OculusReaderTransport(QUEST3_ADB_PORT)
        elif mode == "vuer":
            self._transport = _VuerTransport(
                QUEST3_VUER_PORT, QUEST3_VUER_CERT, QUEST3_VUER_KEY, QUEST3_ACTIVE_HAND,
            )
        elif mode == "udp":
            self._transport = _UDPTransport(QUEST3_UDP_PORT)
        else:
            raise ValueError(
                f"Unknown QUEST3_MODE {mode!r} — expected 'oculus_reader', 'vuer', or 'udp'"
            )

        self._hand     = QUEST3_ACTIVE_HAND
        self._state    = ControllerState()
        self._last_ts  = 0.0
        self._lock     = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread   = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def open(self):
        self._transport.connect()
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="quest3-reader"
        )
        self._thread.start()

        if QUEST3_MODE == "vuer":
            print("[QUEST3] Waiting for Quest 3 to connect to Vuer server...")
            print("         Navigate the Quest 3 browser to the URL shown above.")
            deadline = time.monotonic() + 60.0    # longer wait — user needs to navigate
        else:
            deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            with self._lock:
                if self._state.valid:
                    break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                "[QUEST3] No data received within timeout.\n"
                + self._timeout_hint()
            )

    def _timeout_hint(self) -> str:
        if QUEST3_MODE == "oculus_reader":
            return ("         Check: adb forward tcp:5555 tcp:5555\n"
                    "                Oculus Reader APK is open on Quest 3")
        if QUEST3_MODE == "vuer":
            return ("         Check: Quest 3 browser opened the Vuer URL\n"
                    "                SSL cert warning was accepted\n"
                    "                'Enter VR' was tapped")
        return "         Check: Quest 3 streamer app is running and pointing to this PC"

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
        if (time.time() - self._last_ts) * 1000 > QUEST3_STALE_MS:
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
