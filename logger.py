"""
logger.py — saves teleoperation episodes to CSV + JSON metadata.

Per-timestep columns logged:
  timestamp                    — wall-clock float64 (seconds)
  so101_<joint>                — SO-101 leader joint positions (deg)
  fr5_cmd_j[1..6]             — FR5 commanded joint positions (deg)
  fr5_actual_j[1..6]          — FR5 actual joint positions (deg)
  fr5_eef_x/y/z_mm            — TCP position in mm
  fr5_eef_rx/ry/rz_deg        — TCP Euler orientation (deg)
  gripper_norm                 — SO-101 gripper normalised [0.0, 1.0]
  fr5_vel_j[1..6]             — FR5 actual joint velocities (deg/s)

Episode metadata (JSON sidecar):
  episode_id, start_time, language_instruction, num_steps, duration_s
"""

import json
import os
import time

import pandas as pd

from config import LOG_DIR, INSTRUCTION_FILE


class EpisodeLogger:
    def __init__(self):
        self._rows: list[dict] = []
        self._episode_id = int(time.time())
        self._start_time: float = 0.0
        self._instruction: str = ""
        self._recording = False

    @property
    def recording(self) -> bool:
        return self._recording

    def start(self):
        self._rows = []
        self._episode_id = int(time.time())
        self._start_time = time.time()
        self._instruction = self._read_instruction()
        self._recording = True

    def stop(self) -> str | None:
        """Stop recording and flush to disk. Returns CSV path or None."""
        self._recording = False
        if not self._rows:
            return None
        return self._flush()

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
        except FileNotFoundError:
            return ""

    def _flush(self) -> str:
        os.makedirs(LOG_DIR, exist_ok=True)
        base = os.path.join(LOG_DIR, f"episode_{self._episode_id}")

        pd.DataFrame(self._rows).to_csv(f"{base}.csv", index=False)

        duration = (self._rows[-1]["timestamp"] - self._rows[0]["timestamp"]) if self._rows else 0.0
        meta = {
            "episode_id":           self._episode_id,
            "start_time":           self._start_time,
            "language_instruction": self._instruction,
            "num_steps":            len(self._rows),
            "duration_s":           round(duration, 3),
        }
        with open(f"{base}.json", "w") as f:
            json.dump(meta, f, indent=2)

        return f"{base}.csv"
