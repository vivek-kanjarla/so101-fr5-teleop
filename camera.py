"""
camera.py — Intel RealSense capture for one or more cameras.

Each RealSenseCamera runs its own pipeline + background thread, selected by
serial number so the wrist (eye-in-hand) D405 and the fixed (eye-to-hand) D435i
are never confused. Color is always captured; depth is optional and, when
enabled, is aligned to the color stream so depth[y, x] corresponds to color[y, x].

While recording, frames are buffered as (timestamp, color_bgr, depth_or_None)
tuples. The logger calls start_recording() / stop_recording() to bracket each
episode and writes one MP4 (+ optional depth .npz) per camera.

Color intrinsics come from the on-chip factory calibration — no checkerboard
needed. They (and the depth scale) are stored in the episode JSON via the logger.

D405Camera is kept as a thin wrist_cam alias for backward compatibility.
"""

import threading
import time

import numpy as np

from config import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_USE_HW_TIMESTAMP


class RealSenseCamera:
    def __init__(
        self,
        serial: str | None = None,
        name: str = "cam",
        width: int = CAMERA_WIDTH,
        height: int = CAMERA_HEIGHT,
        fps: int = CAMERA_FPS,
        enable_depth: bool = False,
    ):
        self.name         = name
        self._serial      = serial
        self._width       = width
        self._height      = height
        self._fps         = fps
        self._enable_depth = enable_depth

        self._pipeline   = None
        self._align      = None      # rs.align(color) when depth is enabled
        self._intrinsics = None
        self._depth_scale = None     # metres per depth unit (None if no depth)
        self._use_hw_ts  = CAMERA_USE_HW_TIMESTAMP

        self._thread     = None
        self._stop_evt   = threading.Event()

        self._recording   = False
        self._frames      = []       # list of (ts, color_bgr_uint8, depth_uint16_or_None)
        self._frames_lock = threading.Lock()

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        import pyrealsense2 as rs

        cfg = rs.config()
        if self._serial:
            cfg.enable_device(self._serial)
        cfg.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        if self._enable_depth:
            cfg.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)

        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(cfg)

        # Enable global-time so per-frame hardware timestamps are reported on the
        # host epoch (comparable to time.time() used by the joint log).
        if self._use_hw_ts:
            try:
                for s in profile.get_device().query_sensors():
                    if s.supports(rs.option.global_time_enabled):
                        s.set_option(rs.option.global_time_enabled, 1)
            except Exception as exc:
                print(f"[CAMERA:{self.name}] Could not enable global time ({exc}) — using arrival time.")
                self._use_hw_ts = False

        ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # Save distortion model name alongside coefficients — the RealSense color
        # stream uses inverse-Brown-Conrady, which differs from OpenCV's default
        # Brown-Conrady. Knowing the model prevents wrong undistortion later.
        self._intrinsics = {
            "width":            ci.width,
            "height":           ci.height,
            "fx":               ci.fx,
            "fy":               ci.fy,
            "cx":               ci.ppx,
            "cy":               ci.ppy,
            "distortion_model": str(ci.model).split(".")[-1],
            "dist_coeffs":      list(ci.coeffs),
        }

        depth_info = ""
        if self._enable_depth:
            # Align depth → color so the two streams are pixel-registered.
            self._align       = rs.align(rs.stream.color)
            self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth_info = f"  +depth(scale={self._depth_scale:.6f} m/unit, aligned→color)"

        # Warm up — let auto-exposure settle before starting the capture thread.
        for _ in range(5):
            try:
                self._pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                pass

        print(
            f"[CAMERA:{self.name}] {'serial ' + self._serial + ' ' if self._serial else ''}"
            f"ready — {self._width}×{self._height} @ {self._fps} fps  "
            f"fx={ci.fx:.1f} fy={ci.fy:.1f} cx={ci.ppx:.1f} cy={ci.ppy:.1f}{depth_info}"
        )

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None

    # ── recording control (called by EpisodeLogger) ───────────────────────────

    def start_recording(self):
        with self._frames_lock:
            self._frames    = []
            self._recording = True

    def stop_recording(self) -> list:
        """Stop buffering and return [(ts, color_bgr, depth_or_None), ...]."""
        with self._frames_lock:
            self._recording = False
            frames = list(self._frames)
            self._frames    = []
        return frames

    @property
    def intrinsics(self) -> dict | None:
        return self._intrinsics

    @property
    def depth_scale(self) -> float | None:
        return self._depth_scale

    @property
    def has_depth(self) -> bool:
        return self._enable_depth

    # ── background capture thread ─────────────────────────────────────────────

    def _loop(self):
        import pyrealsense2 as rs

        # Timestamp domains that live on the host epoch (so they're comparable to
        # time.time()). hardware_clock is device-uptime → not usable for joint sync.
        host_epoch_domains = (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time)
        logged_ts_source = False

        consecutive_errors = 0
        while not self._stop_evt.is_set():
            try:
                framesets = self._pipeline.wait_for_frames(timeout_ms=1000)
                if self._align is not None:
                    framesets = self._align.process(framesets)

                color = framesets.get_color_frame()
                if not color:
                    continue

                depth_img = None
                if self._enable_depth:
                    depth = framesets.get_depth_frame()
                    if not depth:
                        continue
                    depth_img = np.asanyarray(depth.get_data()).copy()  # H×W uint16

                consecutive_errors = 0

                # Prefer the frame's hardware (global-time) timestamp; it's in ms
                # on the host epoch. Fall back to arrival time if the device only
                # reports its raw hardware clock.
                ts = time.time()
                if self._use_hw_ts and color.get_frame_timestamp_domain() in host_epoch_domains:
                    ts = color.get_timestamp() / 1000.0
                    if not logged_ts_source:
                        print(f"[CAMERA:{self.name}] using hardware (global-time) timestamps")
                        logged_ts_source = True
                elif not logged_ts_source:
                    print(f"[CAMERA:{self.name}] using arrival-time (time.time) timestamps")
                    logged_ts_source = True

                img = np.asanyarray(color.get_data()).copy()   # H×W×3 BGR uint8

                with self._frames_lock:
                    if self._recording:
                        self._frames.append((ts, img, depth_img))

            except Exception as exc:
                if self._stop_evt.is_set():
                    break
                exc_str = str(exc)
                # Pipeline was stopped externally (USB drop, cleanup race) — exit
                # the thread rather than spinning millions of times per second.
                if "before start" in exc_str or self._pipeline is None:
                    print(f"[CAMERA:{self.name}] Pipeline stopped unexpectedly — capture thread exiting.")
                    break
                consecutive_errors += 1
                # Print at first error then every 30th to avoid flooding the console
                # during sustained USB issues (which usually mean the camera dropped).
                if consecutive_errors == 1 or consecutive_errors % 30 == 0:
                    print(f"[CAMERA:{self.name}] Frame error #{consecutive_errors}: {exc}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()


class D405Camera(RealSenseCamera):
    """Backward-compatible wrist (eye-in-hand) D405 — color only, name 'wrist_cam'."""

    def __init__(self, serial: str | None = None):
        super().__init__(serial=serial, name="wrist_cam", enable_depth=False)
