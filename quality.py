"""
quality.py — per-episode demonstration quality metrics for ACT data curation.

ACT (Action Chunking Transformer) learns **task success**, not minimum-jerk
trajectories. Successful manipulation routinely contains pauses, alignment
corrections, and the occasional re-grasp. So the scalar quality_score is
success-dominated; smoothness is kept only as a small, diagnostic-weight term,
and cartesian "efficiency" (straightness) is computed for inspection but NOT
included in the score (it is a poor proxy for multi-waypoint pick-and-place).

quality_score (0..100) = 100 × Σ wᵢ·termᵢ / Σ wᵢ, with terms:
  success_quality  successful=1.0 / partial=0.5 / failed(or unlabeled)=0.0
  duration_score   min(1, target_duration / duration)
  grasp_quality    1 − |grasp_count − expected_grasps| / expected_grasps   (clipped)
  pause_score      1 − max(0, pauses − pause_free) / (pause_ref − pause_free) (clipped)
  smoothness       1 / (1 + rms_jerk / jerk_ref)            (from dense fr5_cmd)

Diagnostics (not scored): rms_jerk, ldlj, motion_efficiency, path lengths,
velocities, regrasp_count, gripper open/close events.
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
    # Scalar-score weights (normalised by their sum).
    success: float = 0.40
    duration: float = 0.25
    grasp: float = 0.15
    pause: float = 0.10
    smoothness: float = 0.10
    # Task / calibration parameters.
    target_duration_s: float = 35.0
    expected_grasps: int = 3
    jerk_ref: float = 12000.0
    pause_vel_thresh: float = 5.0
    pause_min_s: float = 0.3
    pause_free: int = 3            # alignment pauses allowed before penalty
    pause_ref: int = 12           # pauses beyond free that drive pause_score → 0


def weights_from(cfg) -> QualityWeights:
    """Build QualityWeights from the config module (single source of truth)."""
    return QualityWeights(
        success=cfg.QUALITY_W_SUCCESS, duration=cfg.QUALITY_W_DURATION,
        grasp=cfg.QUALITY_W_GRASP, pause=cfg.QUALITY_W_PAUSE,
        smoothness=cfg.QUALITY_W_SMOOTHNESS,
        target_duration_s=cfg.QUALITY_TARGET_DURATION_S,
        expected_grasps=cfg.QUALITY_EXPECTED_GRASPS,
        jerk_ref=cfg.QUALITY_JERK_REF,
        pause_vel_thresh=cfg.QUALITY_PAUSE_VEL_THRESH,
        pause_min_s=cfg.QUALITY_PAUSE_MIN_S,
        pause_free=cfg.QUALITY_PAUSE_FREE, pause_ref=cfg.QUALITY_PAUSE_REF,
    )


@dataclass
class QualityMetrics:
    # Labels echoed for convenience.
    success: bool | None
    partial_success: bool | None
    # Scored sub-terms (0..1).
    success_quality: float
    duration_score: float
    grasp_quality: float
    pause_score: float
    smoothness: float
    # Diagnostics (not scored).
    motion_efficiency: float
    duration_s: float
    num_steps: int
    path_length_joint_deg: float
    path_length_cart_mm: float
    avg_joint_vel_deg_s: float
    peak_joint_vel_deg_s: float
    rms_jerk_deg_s3: float
    ldlj: float
    pauses: int
    grasp_count: int
    regrasp_count: int
    gripper_open_events: int
    gripper_close_events: int
    # Final scalar.
    quality_score: float

    def to_dict(self) -> dict:
        return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in asdict(self).items()}


def _joints(df: pd.DataFrame) -> np.ndarray:
    # Prefer the COMMANDED joints: logged every control cycle (125 Hz, dense), so
    # derivatives are faithful. fr5_actual is sub-sampled + forward-filled into a
    # staircase that would inflate jerk ~20x.
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
    success: bool | None = None,
    partial_success: bool | None = None,
    gripper_open_thr: float = 0.65,
    gripper_close_thr: float = 0.35,
) -> QualityMetrics:
    w = weights or QualityWeights()
    n = len(df)

    # Sparse actual/eef/vel columns → fill so metrics stay finite.
    df = df.copy()
    num_cols = df.select_dtypes(include=[np.number]).columns
    if len(num_cols):
        df[num_cols] = df[num_cols].ffill().bfill().fillna(0.0)

    t = df["timestamp"].to_numpy(dtype=np.float64) if "timestamp" in df.columns else np.arange(n) / 125.0
    duration = float(t[-1] - t[0]) if n > 1 else 0.0

    q  = _joints(df)
    dt = _safe_dt(t)
    vel = np.gradient(q, axis=0) / dt[:, None] if n > 1 else np.zeros_like(q)
    vel_norm = np.linalg.norm(vel, axis=1)
    avg_v  = float(np.mean(vel_norm)) if n else 0.0
    peak_v = float(np.max(vel_norm)) if n else 0.0

    path_joint = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1))) if n > 1 else 0.0

    # Cartesian efficiency — diagnostic only (NOT in the scalar score).
    if all(c in df.columns for c in EEF_XYZ) and n > 1:
        xyz = df[EEF_XYZ].to_numpy(dtype=np.float64)
        path_cart = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
        net_cart  = float(np.linalg.norm(xyz[-1] - xyz[0]))
        efficiency = float(np.clip(net_cart / path_cart, 0.0, 1.0)) if path_cart > 1e-6 else 0.0
    else:
        path_cart, efficiency = 0.0, 0.0

    # Jerk + smoothness (from dense command trajectory).
    if n > 3:
        accel = np.gradient(vel, axis=0) / dt[:, None]
        jerk  = np.gradient(accel, axis=0) / dt[:, None]
        jerk_norm = np.linalg.norm(jerk, axis=1)
        rms_jerk = float(np.sqrt(np.mean(jerk_norm ** 2)))
        dlj  = (duration ** 3 / max(peak_v, 1e-6) ** 2) * float(np.mean(jerk_norm ** 2)) if duration > 0 else 0.0
        ldlj = float(-np.log(dlj)) if dlj > 0 else 0.0
    else:
        rms_jerk, ldlj = 0.0, 0.0
    smoothness = float(1.0 / (1.0 + rms_jerk / max(w.jerk_ref, 1e-6)))

    # Pauses (hesitation segments).
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
    grasp_count = closes
    regrasp_count = max(0, grasp_count - w.expected_grasps)

    # ── scored sub-terms ──────────────────────────────────────────────────────
    success_quality = 1.0 if success else (0.5 if partial_success else 0.0)
    duration_score  = float(np.clip(w.target_duration_s / duration, 0.0, 1.0)) if duration > 0 else 0.0
    # Penalise deviation from the expected grasp count (missing OR extra re-grasps).
    grasp_quality   = float(np.clip(1.0 - abs(grasp_count - w.expected_grasps) / max(w.expected_grasps, 1), 0.0, 1.0))
    # Reward few pauses; a small free allowance for alignment corrections.
    span = max(w.pause_ref - w.pause_free, 1)
    pause_score     = float(np.clip(1.0 - max(0, pauses - w.pause_free) / span, 0.0, 1.0))

    wsum = w.success + w.duration + w.grasp + w.pause + w.smoothness
    score01 = (w.success * success_quality + w.duration * duration_score +
               w.grasp * grasp_quality + w.pause * pause_score +
               w.smoothness * smoothness) / max(wsum, 1e-6)
    quality_score = float(round(100.0 * score01, 2))

    return QualityMetrics(
        success=success, partial_success=partial_success,
        success_quality=round(success_quality, 4),
        duration_score=round(duration_score, 4),
        grasp_quality=round(grasp_quality, 4),
        pause_score=round(pause_score, 4),
        smoothness=round(smoothness, 4),
        motion_efficiency=round(efficiency, 4),
        duration_s=round(duration, 3),
        num_steps=n,
        path_length_joint_deg=round(path_joint, 2),
        path_length_cart_mm=round(path_cart, 2),
        avg_joint_vel_deg_s=round(avg_v, 3),
        peak_joint_vel_deg_s=round(peak_v, 3),
        rms_jerk_deg_s3=round(rms_jerk, 3),
        ldlj=round(ldlj, 4),
        pauses=int(pauses),
        grasp_count=int(grasp_count),
        regrasp_count=int(regrasp_count),
        gripper_open_events=int(opens),
        gripper_close_events=int(closes),
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
