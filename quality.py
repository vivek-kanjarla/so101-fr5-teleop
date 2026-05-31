"""
quality.py — per-episode demonstration quality metrics for ACT data curation.

ACT (Action Chunking Transformer) learns short action chunks from human demos. Its
output quality is bounded by demo quality: jerky, hesitant, or meandering demos
produce a policy that is jerky, hesitant, or meandering. These metrics quantify
those properties so episodes can be scored, ranked, and filtered before training.

Computed from an episode DataFrame (the recorded data.csv):
  duration_s, path_length (joint deg + cartesian mm), avg/peak joint velocity,
  rms_jerk, log-dimensionless-jerk (LDLJ), smoothness (0..1), motion_efficiency
  (cartesian straightness 0..1), pauses, grasp_count, gripper open/close events,
  and a weighted quality_score (0..100).
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np
import pandas as pd

ACTUAL_COLS = [f"fr5_actual_j{i}" for i in range(1, 7)]
CMD_COLS    = [f"fr5_cmd_j{i}"    for i in range(1, 7)]
VEL_COLS    = [f"fr5_vel_j{i}"    for i in range(1, 7)]
EEF_XYZ     = ["fr5_eef_x_mm", "fr5_eef_y_mm", "fr5_eef_z_mm"]


@dataclass
class QualityWeights:
    smoothness: float = 0.5
    duration: float = 0.2
    efficiency: float = 0.3
    target_duration_s: float = 12.0
    jerk_ref: float = 5000.0        # deg/s^3, smoothness normalisation scale
    pause_vel_thresh: float = 5.0   # deg/s
    pause_min_s: float = 0.3


@dataclass
class QualityMetrics:
    duration_s: float
    num_steps: int
    path_length_joint_deg: float
    path_length_cart_mm: float
    avg_joint_vel_deg_s: float
    peak_joint_vel_deg_s: float
    rms_jerk_deg_s3: float
    ldlj: float                 # log dimensionless jerk (higher = smoother)
    smoothness: float           # 0..1 (higher = smoother)
    motion_efficiency: float    # 0..1 (cartesian net / path)
    pauses: int
    grasp_count: int
    gripper_open_events: int
    gripper_close_events: int
    duration_score: float       # 0..1
    quality_score: float        # 0..100

    def to_dict(self) -> dict:
        return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in asdict(self).items()}


def _joints(df: pd.DataFrame) -> np.ndarray:
    # Prefer the COMMANDED joints: they are logged every control cycle (125 Hz,
    # dense), so derivatives (velocity/jerk/smoothness) are faithful. fr5_actual
    # is read sparsely (~20 Hz staggered) and forward-filled into a staircase,
    # which would inflate jerk by ~20x and wreck the smoothness metric.
    cols = CMD_COLS if all(c in df.columns for c in CMD_COLS) else ACTUAL_COLS
    return df[cols].to_numpy(dtype=np.float64)


def _safe_dt(t: np.ndarray) -> np.ndarray:
    dt = np.gradient(t)
    good = dt[dt > 0]
    dt[dt <= 0] = np.median(good) if good.size else 1.0 / 125.0
    return dt


def _gripper_events(g: np.ndarray, open_thr: float, close_thr: float) -> tuple[int, int]:
    """Count close→open and open→close transitions with hysteresis."""
    state = None
    opens = closes = 0
    for v in g:
        if v >= open_thr:
            if state == "closed":
                opens += 1
            state = "open"
        elif v <= close_thr:
            if state == "open":
                closes += 1
            state = "closed"
    return opens, closes


def compute_quality(
    df: pd.DataFrame,
    weights: QualityWeights | None = None,
    gripper_open_thr: float = 0.65,
    gripper_close_thr: float = 0.35,
) -> QualityMetrics:
    w = weights or QualityWeights()
    n = len(df)

    # Actual/eef/vel columns are logged sparsely (staggered RPC reads), so the
    # first rows can be NaN. Forward/back-fill numeric columns so metrics stay
    # finite (otherwise jerk/smoothness/score come out NaN).
    df = df.copy()
    num_cols = df.select_dtypes(include=[np.number]).columns
    if len(num_cols):
        df[num_cols] = df[num_cols].ffill().bfill().fillna(0.0)

    t = df["timestamp"].to_numpy(dtype=np.float64) if "timestamp" in df.columns else np.arange(n) / 125.0
    duration = float(t[-1] - t[0]) if n > 1 else 0.0

    q  = _joints(df)
    dt = _safe_dt(t)

    # Derive velocity from the dense commanded trajectory rather than the logged
    # fr5_vel (also staggered + ffilled) so velocity/jerk stay artifact-free.
    vel = np.gradient(q, axis=0) / dt[:, None]
    vel_norm = np.linalg.norm(vel, axis=1)
    avg_v = float(np.mean(vel_norm))
    peak_v = float(np.max(vel_norm)) if n else 0.0

    # Path lengths.
    path_joint = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1))) if n > 1 else 0.0
    if all(c in df.columns for c in EEF_XYZ) and n > 1:
        xyz = df[EEF_XYZ].to_numpy(dtype=np.float64)
        path_cart = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
        net_cart  = float(np.linalg.norm(xyz[-1] - xyz[0]))
        efficiency = float(np.clip(net_cart / path_cart, 0.0, 1.0)) if path_cart > 1e-6 else 0.0
    else:
        path_cart = 0.0
        efficiency = 0.0

    # Jerk + smoothness. LDLJ (log dimensionless jerk) is a standard movement-
    # smoothness metric; higher (less negative) = smoother. We also map an
    # RMS-jerk into a bounded 0..1 smoothness for the weighted score.
    if n > 3:
        accel = np.gradient(vel, axis=0) / dt[:, None]
        jerk  = np.gradient(accel, axis=0) / dt[:, None]
        jerk_norm = np.linalg.norm(jerk, axis=1)
        rms_jerk = float(np.sqrt(np.mean(jerk_norm ** 2)))
        peak_v_safe = max(peak_v, 1e-6)
        dlj = (duration ** 3 / peak_v_safe ** 2) * float(np.mean(jerk_norm ** 2)) if duration > 0 else 0.0
        ldlj = float(-np.log(dlj)) if dlj > 0 else 0.0
    else:
        rms_jerk = 0.0
        ldlj = 0.0
    smoothness = float(1.0 / (1.0 + rms_jerk / max(w.jerk_ref, 1e-6)))

    # Pauses: contiguous low-velocity segments long enough to count as hesitation.
    pauses = 0
    if n > 1:
        low = vel_norm < w.pause_vel_thresh
        min_len = max(1, int(round(w.pause_min_s / max(float(np.median(dt)), 1e-6))))
        run = 0
        for flag in low:
            run = run + 1 if flag else 0
            if run == min_len:
                pauses += 1

    opens, closes = _gripper_events(
        df["gripper_norm"].to_numpy(dtype=np.float64) if "gripper_norm" in df.columns else np.zeros(n),
        gripper_open_thr, gripper_close_thr,
    )

    # Weighted score. duration_score = 1 if at/under target, decaying when slower.
    duration_score = float(np.clip(w.target_duration_s / duration, 0.0, 1.0)) if duration > 0 else 0.0
    wsum = w.smoothness + w.duration + w.efficiency
    score01 = (w.smoothness * smoothness + w.duration * duration_score + w.efficiency * efficiency) / max(wsum, 1e-6)
    quality_score = float(round(100.0 * score01, 2))

    return QualityMetrics(
        duration_s=round(duration, 3),
        num_steps=n,
        path_length_joint_deg=round(path_joint, 2),
        path_length_cart_mm=round(path_cart, 2),
        avg_joint_vel_deg_s=round(avg_v, 3),
        peak_joint_vel_deg_s=round(peak_v, 3),
        rms_jerk_deg_s3=round(rms_jerk, 3),
        ldlj=round(ldlj, 4),
        smoothness=round(smoothness, 4),
        motion_efficiency=round(efficiency, 4),
        pauses=int(pauses),
        grasp_count=int(closes),
        gripper_open_events=int(opens),
        gripper_close_events=int(closes),
        duration_score=round(duration_score, 4),
        quality_score=quality_score,
    )


def camera_sync_gap_ms(ts_by_cam: dict[str, np.ndarray], reference: str | None = None) -> float:
    """Median nearest-frame timestamp gap (ms) between the reference camera and
    the others — a proxy for cross-camera synchronisation quality. 0 if <2 cams."""
    names = [k for k, v in ts_by_cam.items() if v is not None and len(v) > 0]
    if len(names) < 2:
        return 0.0
    ref = reference if reference in names else ("wrist_cam" if "wrist_cam" in names else names[0])
    ref_ts = np.sort(np.asarray(ts_by_cam[ref], dtype=np.float64))
    gaps: list[float] = []
    for name in names:
        if name == ref:
            continue
        other = np.sort(np.asarray(ts_by_cam[name], dtype=np.float64))
        for tt in ref_ts:
            gaps.append(float(np.min(np.abs(other - tt))))
    return float(np.median(gaps) * 1000.0) if gaps else 0.0
