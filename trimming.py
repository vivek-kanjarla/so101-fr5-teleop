"""
trimming.py — automatic idle trimming for recorded episodes.

Finds the active span of a demonstration (first to last real motion) and pads it
with a configurable context margin, so dataset export can drop the dead air before
the operator starts and after the task completes.

Why for ACT: leading/trailing idle frames teach the policy to predict "no motion",
diluting the useful action distribution and wasting chunk capacity. Keeping a 2 s
margin preserves the natural approach/retreat dynamics around the task.

Activity = (joint-velocity norm > vel_norm_thresh) OR (gripper rate > gripper_rate_thresh).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

ACTUAL_COLS = [f"fr5_actual_j{i}" for i in range(1, 7)]
CMD_COLS    = [f"fr5_cmd_j{i}"    for i in range(1, 7)]
VEL_COLS    = [f"fr5_vel_j{i}"    for i in range(1, 7)]


@dataclass
class TrimConfig:
    vel_norm_thresh: float = 8.0       # deg/s
    gripper_rate_thresh: float = 0.05  # normalized units/s
    keep_before_s: float = 2.0
    keep_after_s: float = 2.0


def _joint_velocity_norm(df: pd.DataFrame, t: np.ndarray) -> np.ndarray:
    """Per-sample joint-velocity norm (deg/s). Prefer logged velocities; else
    differentiate the actual (or commanded) joint positions."""
    if all(c in df.columns for c in VEL_COLS) and df[VEL_COLS].abs().to_numpy().sum() > 0:
        v = df[VEL_COLS].to_numpy(dtype=np.float64)
        return np.linalg.norm(v, axis=1)

    cols = ACTUAL_COLS if all(c in df.columns for c in ACTUAL_COLS) else CMD_COLS
    q  = df[cols].to_numpy(dtype=np.float64)
    dt = np.gradient(t)
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 1.0
    dq = np.gradient(q, axis=0)
    v  = dq / dt[:, None]
    return np.linalg.norm(v, axis=1)


def _gripper_rate(df: pd.DataFrame, t: np.ndarray) -> np.ndarray:
    if "gripper_norm" not in df.columns:
        return np.zeros(len(df))
    g  = df["gripper_norm"].to_numpy(dtype=np.float64)
    dt = np.gradient(t)
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 1.0
    return np.abs(np.gradient(g) / dt)


def compute_active_range(df: pd.DataFrame, cfg: TrimConfig) -> tuple[int, int]:
    """Return (start_idx, end_idx) half-open range to keep. (0, len) if no motion
    is detected (never trims to empty)."""
    n = len(df)
    if n < 3 or "timestamp" not in df.columns:
        return 0, n

    t = df["timestamp"].to_numpy(dtype=np.float64)
    vel_norm = _joint_velocity_norm(df, t)
    grip_rate = _gripper_rate(df, t)

    active = (vel_norm > cfg.vel_norm_thresh) | (grip_rate > cfg.gripper_rate_thresh)
    if not np.any(active):
        return 0, n   # nothing crossed threshold — keep everything, don't trim to nil

    idx = np.flatnonzero(active)
    first, last = int(idx[0]), int(idx[-1])

    dt_med = float(np.median(np.diff(t))) if n > 1 else 1.0 / 125.0
    dt_med = dt_med if dt_med > 0 else 1.0 / 125.0
    before = int(round(cfg.keep_before_s / dt_med))
    after  = int(round(cfg.keep_after_s / dt_med))

    start = max(0, first - before)
    end   = min(n, last + 1 + after)
    return start, end
