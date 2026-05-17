"""
camera.py — Intel RealSense D405 wrist camera capture.

Runs a background thread that reads RGB frames at CAMERA_FPS continuously.
When recording is active, frames are buffered as (timestamp, bgr_array) pairs.
The logger calls start_recording() / stop_recording() to bracket each episode.

Intrinsics are read from the D405 on-chip factory calibration — no checkerboard
needed. They are stored in the episode JSON sidecar via logger._flush().
"""

import threading
import time

import numpy as np

from config import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS


class D405Camera:
    def __init__(self):
        self._pipeline   = None
        self._intrinsics = None

        self._thread     = None
        self._stop_evt   = threading.Event()

        self._recording     = False
        self._frames        = []   # list of (timestamp_float, H×W×3 uint8 BGR)
        self._frames_lock   = threading.Lock()

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        import pyrealsense2 as rs

        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, CAMERA_FPS)

        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(cfg)

        ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        # Save distortion model name alongside coefficients — the D405 color
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

        # Warm up — let auto-exposure settle before starting the capture thread.
        for _ in range(5):
            try:
                self._pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                pass

        print(
            f"[CAMERA] D405 ready — {CAMERA_WIDTH}×{CAMERA_HEIGHT} @ {CAMERA_FPS} fps  "
            f"fx={ci.fx:.1f} fy={ci.fy:.1f} cx={ci.ppx:.1f} cy={ci.ppy:.1f}"
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
        """Stop buffering and return [(timestamp, bgr_frame), ...] for this episode."""
        with self._frames_lock:
            self._recording = False
            frames = list(self._frames)
            self._frames    = []
        return frames

    @property
    def intrinsics(self) -> dict | None:
        return self._intrinsics

    # ── background capture thread ─────────────────────────────────────────────

    def _loop(self):
        consecutive_errors = 0
        while not self._stop_evt.is_set():
            try:
                framesets = self._pipeline.wait_for_frames(timeout_ms=1000)
                color     = framesets.get_color_frame()
                if not color:
                    continue

                consecutive_errors = 0
                ts  = time.time()
                img = np.asanyarray(color.get_data()).copy()   # H×W×3 BGR uint8

                with self._frames_lock:
                    if self._recording:
                        self._frames.append((ts, img))

            except Exception as exc:
                if self._stop_evt.is_set():
                    break
                consecutive_errors += 1
                # Print at first error then every 30th to avoid flooding the console
                # during sustained USB issues (which usually mean the camera dropped).
                if consecutive_errors == 1 or consecutive_errors % 30 == 0:
                    print(f"[CAMERA] Frame error #{consecutive_errors}: {exc}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()
