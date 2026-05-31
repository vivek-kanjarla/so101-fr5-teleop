"""
logger.py — saves teleoperation episodes to CSV + JSON metadata + MP4 video.

Per-timestep columns logged:
  timestamp                    — wall-clock float64 (seconds)
  so101_<joint>                — SO-101 leader joint positions (deg)
  fr5_cmd_j[1..6]             — FR5 commanded joint positions (deg)
  fr5_actual_j[1..6]          — FR5 actual joint positions (deg)
  fr5_eef_x/y/z_mm            — TCP position in mm
  fr5_eef_rx/ry/rz_deg        — TCP Euler orientation (deg)
  gripper_norm                 — SO-101 gripper normalised [0.0, 1.0]
  fr5_vel_j[1..6]             — FR5 actual joint velocities (deg/s)

Each episode is saved in its own folder, numbered sequentially:
  episodes/episode_{NNN}/
    data.csv             — timestep data (above)
    meta.json            — metadata + per-camera intrinsics
    {name}.mp4           — RGB video at CAMERA_FPS, one per attached camera
    {name}_ts.npy        — per-frame timestamps as float64 array
    {name}_depth.npz     — aligned depth stack (uint16), depth cameras only
"""

import glob
import json
import os
import re
import time

import numpy as np
import pandas as pd

from config import (LOG_DIR, INSTRUCTION_FILE, CAMERA_FPS, CAMERA_WIDTH, CAMERA_HEIGHT,
                    SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD,
                    QUALITY_W_SMOOTHNESS, QUALITY_W_DURATION, QUALITY_W_EFFICIENCY,
                    QUALITY_TARGET_DURATION_S, QUALITY_JERK_REF,
                    QUALITY_PAUSE_VEL_THRESH, QUALITY_PAUSE_MIN_S)


class EpisodeLogger:
    def __init__(self):
        self._rows: list[dict] = []
        self._episode_id = int(time.time() * 1000)   # millisecond id (traceability)
        self._episode_index = 0                       # sequential folder number
        self._start_time: float = 0.0
        self._instruction: str = ""
        self._recording = False
        self._cameras: list = []   # list of RealSenseCamera, each with a unique .name

    @staticmethod
    def _next_episode_index() -> int:
        """Next sequential index = highest existing episode_NNN folder + 1."""
        indices = [-1]
        for path in glob.glob(os.path.join(LOG_DIR, "episode_*")):
            if os.path.isdir(path):
                m = re.fullmatch(r"episode_(\d+)", os.path.basename(path))
                if m:
                    indices.append(int(m.group(1)))
        return max(indices) + 1

    @property
    def recording(self) -> bool:
        return self._recording

    def set_camera(self, camera) -> None:
        """Attach a single camera (back-compat). Call before the teleop loop starts."""
        self._cameras = [camera] if camera is not None else []

    def set_cameras(self, cameras) -> None:
        """Attach multiple cameras. Call before the teleop loop starts."""
        self._cameras = list(cameras)

    def start(self):
        self._rows = []
        self._episode_id = int(time.time() * 1000)
        self._episode_index = self._next_episode_index()
        self._start_time = time.time()
        self._instruction = self._read_instruction()
        self._recording = True
        for cam in self._cameras:
            cam.start_recording()

    def stop(self) -> str | None:
        """Stop recording and flush to disk. Returns CSV path or None."""
        self._recording = False
        # Always drain every camera so its buffer is cleared even if we bail out.
        frames_by_cam = {cam.name: cam.stop_recording() for cam in self._cameras}
        if not self._rows:
            return None
        return self._flush(frames_by_cam)

    def log(
        self,
        timestamp: float,
        so101: dict[str, float],
        fr5_cmd: list[float],
        fr5_actual: list[float] | None = None,
        fr5_eef: list[float] | None = None,
        gripper_norm: float | None = None,
        fr5_vel: list[float] | None = None,
    ):
        if not self._recording:
            return

        row: dict = {"timestamp": timestamp}

        for k, v in so101.items():
            row[f"so101_{k}"] = v

        for i, v in enumerate(fr5_cmd, start=1):
            row[f"fr5_cmd_j{i}"] = v

        if fr5_actual is not None:
            for i, v in enumerate(fr5_actual, start=1):
                row[f"fr5_actual_j{i}"] = v

        if fr5_eef is not None and len(fr5_eef) == 6:
            row["fr5_eef_x_mm"]  = fr5_eef[0]
            row["fr5_eef_y_mm"]  = fr5_eef[1]
            row["fr5_eef_z_mm"]  = fr5_eef[2]
            row["fr5_eef_rx_deg"] = fr5_eef[3]
            row["fr5_eef_ry_deg"] = fr5_eef[4]
            row["fr5_eef_rz_deg"] = fr5_eef[5]

        if gripper_norm is not None:
            row["gripper_norm"] = gripper_norm

        if fr5_vel is not None:
            for i, v in enumerate(fr5_vel, start=1):
                row[f"fr5_vel_j{i}"] = v

        self._rows.append(row)

    # ── internal ──────────────────────────────────────────────────────────────

    def _read_instruction(self) -> str:
        try:
            with open(INSTRUCTION_FILE) as f:
                return f.read().strip()
        except Exception:
            return ""

    def _flush(self, frames_by_cam: dict) -> str:
        # Each episode gets its own folder: episodes/episode_NNN/
        ep_dir = os.path.join(LOG_DIR, f"episode_{self._episode_index:03d}")
        os.makedirs(ep_dir, exist_ok=True)

        df = pd.DataFrame(self._rows)
        csv_path = os.path.join(ep_dir, "data.csv")
        df.to_csv(csv_path, index=False)

        # Per-camera metadata (intrinsics, depth scale, frame count). File names
        # are relative to the episode folder.
        cameras_meta: dict = {}
        for cam in self._cameras:
            frames = frames_by_cam.get(cam.name, [])
            cameras_meta[cam.name] = {
                "intrinsics":  cam.intrinsics,
                "has_depth":   cam.has_depth,
                "depth_scale": cam.depth_scale,
                "num_frames":  len(frames),
                "video_file":  f"{cam.name}.mp4",
                "ts_file":     f"{cam.name}_ts.npy",
                "depth_file":  (f"{cam.name}_depth.npz" if cam.has_depth else None),
            }

        # Back-compat: keep the old top-level keys pointing at the first camera.
        primary = self._cameras[0] if self._cameras else None
        primary_frames = frames_by_cam.get(primary.name, []) if primary else []

        duration = (self._rows[-1]["timestamp"] - self._rows[0]["timestamp"]) if self._rows else 0.0

        # Quality metrics (smoothness, jerk, efficiency, grasp events, score) so
        # episodes can be ranked/filtered for ACT without re-reading every CSV.
        quality_dict = None
        try:
            from quality import compute_quality, QualityWeights
            qm = compute_quality(
                df,
                QualityWeights(
                    smoothness=QUALITY_W_SMOOTHNESS, duration=QUALITY_W_DURATION,
                    efficiency=QUALITY_W_EFFICIENCY, target_duration_s=QUALITY_TARGET_DURATION_S,
                    jerk_ref=QUALITY_JERK_REF, pause_vel_thresh=QUALITY_PAUSE_VEL_THRESH,
                    pause_min_s=QUALITY_PAUSE_MIN_S,
                ),
                gripper_open_thr=SO101_GRIPPER_OPEN_THRESHOLD,
                gripper_close_thr=SO101_GRIPPER_CLOSE_THRESHOLD,
            )
            quality_dict = qm.to_dict()
            print(f"[LOGGER] quality_score={qm.quality_score:.1f}  smoothness={qm.smoothness:.3f}  "
                  f"grasps={qm.grasp_count}  pauses={qm.pauses}")
        except Exception as exc:
            print(f"[LOGGER] quality metrics skipped: {exc!r}")

        meta = {
            "episode_id":           self._episode_id,
            "episode_index":        self._episode_index,
            "start_time":           self._start_time,
            "language_instruction": self._instruction,
            "num_steps":            len(self._rows),
            "duration_s":           round(duration, 3),
            "cameras":              cameras_meta,
            "camera_intrinsics":    primary.intrinsics if primary else None,
            "camera_num_frames":    len(primary_frames),
            "quality":              quality_dict,
        }
        with open(os.path.join(ep_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        for cam in self._cameras:
            frames = frames_by_cam.get(cam.name, [])
            if frames:
                self._save_camera(ep_dir, cam, frames)

        print(f"[LOGGER] Episode {self._episode_index:03d} saved → {ep_dir}/")
        return csv_path

    def _save_camera(self, ep_dir: str, cam, frames: list) -> None:
        """Write one camera's color MP4 + timestamps (+ depth stack) into ep_dir."""
        import cv2

        name       = cam.name
        timestamps = np.array([ts for ts, _, _ in frames], dtype=np.float64)
        np.save(os.path.join(ep_dir, f"{name}_ts.npy"), timestamps)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        path   = os.path.join(ep_dir, f"{name}.mp4")
        writer = cv2.VideoWriter(path, fourcc, CAMERA_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT))
        if not writer.isOpened():
            print(f"[LOGGER:{name}] Could not open video writer for {path} — skipping MP4.")
            return

        written = 0
        for _, frame, _ in frames:
            # Ensure frame is a contiguous uint8 BGR array before writing —
            # VideoWriter silently skips frames with wrong dtype or layout.
            if not isinstance(frame, np.ndarray):
                continue
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            if not frame.flags['C_CONTIGUOUS']:
                frame = np.ascontiguousarray(frame)
            writer.write(frame)
            written += 1
        writer.release()
        print(f"[LOGGER:{name}] Video saved: {written}/{len(frames)} frames → {path}")

        # Depth stack (uint16, aligned to color) — compressed; per-frame timestamps
        # are identical to the color timestamps above since they share a frameset.
        if cam.has_depth:
            depth_stack = [d for _, _, d in frames if d is not None]
            if depth_stack:
                arr = np.stack(depth_stack).astype(np.uint16)   # (N, H, W)
                dpath = os.path.join(ep_dir, f"{name}_depth.npz")
                np.savez_compressed(dpath, depth=arr)
                print(f"[LOGGER:{name}] Depth saved: {arr.shape} → {dpath}")
