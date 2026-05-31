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
        self._open_pipeline()

        # Warm up — let auto-exposure settle before starting the capture thread.
        for _ in range(5):
            try:
                self._pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                pass

        ci = self._intrinsics
        depth_info = (f"  +depth(scale={self._depth_scale:.6f} m/unit, aligned→color)"
                      if self._enable_depth else "")
        print(
            f"[CAMERA:{self.name}] {'serial ' + self._serial + ' ' if self._serial else ''}"
            f"ready — {self._width}×{self._height} @ {self._fps} fps  "
            f"fx={ci['fx']:.1f} fy={ci['fy']:.1f} cx={ci['cx']:.1f} cy={ci['cy']:.1f}{depth_info}"
        )

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open_pipeline(self) -> None:
        """(Re)create and start the pipeline; set intrinsics / align / depth scale /
        global-time. Used by start() and by the capture thread to self-heal after a
        device error, so a transient USB hiccup does not end the recording."""
        import pyrealsense2 as rs

        cfg = rs.config()
        if self._serial:
            cfg.enable_device(self._serial)
        cfg.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        if self._enable_depth:
            cfg.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)

        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(cfg)

        # Global-time → per-frame hardware timestamps on the host epoch.
        if self._use_hw_ts:
            try:
                for s in profile.get_device().query_sensors():
                    if s.supports(rs.option.global_time_enabled):
                        s.set_option(rs.option.global_time_enabled, 1)
            except Exception as exc:
                print(f"[CAMERA:{self.name}] Could not enable global time ({exc}) — using arrival time.")
                self._use_hw_ts = False

        ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # inverse-Brown-Conrady distortion model name saved for correct undistortion.
        self._intrinsics = {
            "width": ci.width, "height": ci.height,
            "fx": ci.fx, "fy": ci.fy, "cx": ci.ppx, "cy": ci.ppy,
            "distortion_model": str(ci.model).split(".")[-1],
            "dist_coeffs": list(ci.coeffs),
        }
        if self._enable_depth:
            self._align       = rs.align(rs.stream.color)   # align depth → color
            self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    def _reopen(self) -> bool:
        """Stop and restart the pipeline after a device error. Returns success."""
        try:
            if self._pipeline:
                try:
                    self._pipeline.stop()
                except Exception:
                    pass
            time.sleep(0.3)
            self._open_pipeline()
            for _ in range(3):                       # re-warm
                try:
                    self._pipeline.wait_for_frames(timeout_ms=1000)
                except Exception:
                    pass
            return True
        except Exception as exc:
            print(f"[CAMERA:{self.name}] reopen failed: {exc}")
            return False

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
        restarts = 0
        MAX_RESTARTS = 8          # cap so a truly unplugged camera eventually stops
        RESTART_EVERY = 15        # consecutive errors before attempting a restart
        while not self._stop_evt.is_set():
            try:
                # Generous timeout: tolerate transient USB stalls without counting
                # a frame as lost (a real 30 fps frame arrives in ~33 ms).
                framesets = self._pipeline.wait_for_frames(timeout_ms=2000)
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
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % RESTART_EVERY == 0:
                    print(f"[CAMERA:{self.name}] frame error #{consecutive_errors}: {exc}")

                # Self-heal: a burst of consecutive errors means the device dropped
                # (USB stall/reset). Restart the pipeline instead of giving up, so a
                # transient hiccup does not silently truncate the rest of the episode.
                if consecutive_errors % RESTART_EVERY == 0:
                    if restarts < MAX_RESTARTS:
                        restarts += 1
                        print(f"[CAMERA:{self.name}] restarting pipeline "
                              f"(attempt {restarts}/{MAX_RESTARTS})...")
                        if self._reopen():
                            consecutive_errors = 0
                            print(f"[CAMERA:{self.name}] pipeline recovered")
                        else:
                            time.sleep(0.5)
                    else:
                        print(f"[CAMERA:{self.name}] gave up after {MAX_RESTARTS} restarts "
                              "— capture thread exiting.")
                        break
                else:
                    time.sleep(0.005)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()


class D405Camera(RealSenseCamera):
    """Backward-compatible wrist (eye-in-hand) D405 — color only, name 'wrist_cam'."""

    def __init__(self, serial: str | None = None):
        super().__init__(serial=serial, name="wrist_cam", enable_depth=False)
