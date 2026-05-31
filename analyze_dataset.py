"""
analyze_dataset.py — ACT readiness report for a recorded episode set.

Aggregates per-episode quality + dataset-level properties (idle fraction, camera
synchronisation, demonstration diversity) into an ACT readiness score with concrete
recommendations, so you know whether the data is worth training on before you spend
GPU hours.

Usage:
    python analyze_dataset.py
    python analyze_dataset.py --input ./episodes
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from config import (SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD,
                    TRIM_VEL_NORM_THRESH, TRIM_GRIPPER_RATE_THRESH,
                    TRIM_KEEP_BEFORE_S, TRIM_KEEP_AFTER_S, ACT_FPS)
from convert_to_lerobot import _find_episodes, _load_episode
from quality import compute_quality, camera_sync_gap_ms
from trimming import TrimConfig, compute_active_range

# ACT readiness thresholds (heuristic, tunable).
MIN_EPISODES        = 50      # ACT typically wants dozens+ of demos
MAX_IDLE_FRACTION   = 0.40    # fraction of frames that are dead air
MIN_SMOOTHNESS      = 0.50    # 0..1
MAX_CAMERA_SYNC_MS  = 33.0    # ~1 frame at 30 fps
MIN_DIVERSITY_MM    = 30.0    # spatial spread of cartesian endpoints


@dataclass
class DatasetReport:
    num_episodes: int
    avg_duration_s: float
    avg_path_length_joint_deg: float
    avg_smoothness: float
    avg_quality_score: float
    recorded_hz: float
    avg_idle_fraction: float
    avg_camera_sync_ms: float
    endpoint_diversity_mm: float
    readiness_score: float
    recommendations: list[str]


def analyze(input_dir: Path) -> DatasetReport | None:
    handles = _find_episodes(input_dir)
    if not handles:
        return None

    trim_cfg = TrimConfig(TRIM_VEL_NORM_THRESH, TRIM_GRIPPER_RATE_THRESH,
                          TRIM_KEEP_BEFORE_S, TRIM_KEEP_AFTER_S)
    durations, paths, smooths, scores, hzs, idles, syncs = [], [], [], [], [], [], []
    endpoints = []

    for h in handles:
        try:
            meta, df, cams = _load_episode(h)
        except Exception:
            continue
        qm = compute_quality(df, gripper_open_thr=SO101_GRIPPER_OPEN_THRESHOLD,
                             gripper_close_thr=SO101_GRIPPER_CLOSE_THRESHOLD)
        durations.append(qm.duration_s)
        paths.append(qm.path_length_joint_deg)
        smooths.append(qm.smoothness)
        scores.append(qm.quality_score)

        if "timestamp" in df.columns and len(df) > 1:
            dt = np.diff(df["timestamp"].to_numpy(dtype=np.float64))
            dt = dt[dt > 0]
            if dt.size:
                hzs.append(1.0 / float(np.median(dt)))

        start, end = compute_active_range(df, trim_cfg)
        idles.append(1.0 - (end - start) / max(len(df), 1))

        ts_by_cam = {c["name"]: np.load(c["ts"]) for c in cams if c.get("ts") and Path(c["ts"]).exists()}
        s = camera_sync_gap_ms(ts_by_cam)
        if s > 0:
            syncs.append(s)

        # Diversity proxy: where in cartesian space each demo ends.
        for col in (["fr5_eef_x_mm", "fr5_eef_y_mm", "fr5_eef_z_mm"],):
            if all(c in df.columns for c in col) and len(df):
                endpoints.append(df[col].to_numpy(dtype=np.float64)[-1])

    n = len(durations)
    if n == 0:
        return None

    avg_idle = float(np.mean(idles)) if idles else 0.0
    avg_smooth = float(np.mean(smooths))
    avg_sync = float(np.mean(syncs)) if syncs else 0.0
    diversity = float(np.mean(np.std(np.array(endpoints), axis=0))) if len(endpoints) > 1 else 0.0

    # Readiness score: weighted, each component clamped to 0..1.
    c_count   = min(1.0, n / MIN_EPISODES)
    c_idle    = float(np.clip(1.0 - avg_idle / MAX_IDLE_FRACTION, 0.0, 1.0))
    c_smooth  = float(np.clip(avg_smooth / max(MIN_SMOOTHNESS, 1e-6), 0.0, 1.0)) if avg_smooth < MIN_SMOOTHNESS else 1.0
    c_sync    = 1.0 if not syncs else float(np.clip(MAX_CAMERA_SYNC_MS / max(avg_sync, 1e-6), 0.0, 1.0))
    c_diverse = float(np.clip(diversity / MIN_DIVERSITY_MM, 0.0, 1.0))
    readiness = 100.0 * (0.30 * c_count + 0.25 * c_idle + 0.20 * c_smooth +
                         0.10 * c_sync + 0.15 * c_diverse)

    recs: list[str] = []
    if n < MIN_EPISODES:
        recs.append(f"Insufficient demonstrations: {n} (recommend ≥ {MIN_EPISODES} for ACT).")
    if avg_idle > MAX_IDLE_FRACTION:
        recs.append(f"Too much idle motion ({avg_idle*100:.0f}% of frames) — export with --act to auto-trim.")
    if avg_smooth < MIN_SMOOTHNESS:
        recs.append(f"Demonstrations are jerky (smoothness {avg_smooth:.2f}) — slow down or raise One-Euro/ServoJ filtering.")
    if syncs and avg_sync > MAX_CAMERA_SYNC_MS:
        recs.append(f"Camera synchronisation poor ({avg_sync:.0f} ms > {MAX_CAMERA_SYNC_MS:.0f} ms).")
    if len(endpoints) > 1 and diversity < MIN_DIVERSITY_MM:
        recs.append(f"Insufficient demonstration diversity (endpoint spread {diversity:.0f} mm) — vary object/target placement.")
    if not recs:
        recs.append("Dataset suitable for ACT.")

    return DatasetReport(
        num_episodes=n,
        avg_duration_s=round(float(np.mean(durations)), 2),
        avg_path_length_joint_deg=round(float(np.mean(paths)), 1),
        avg_smoothness=round(avg_smooth, 3),
        avg_quality_score=round(float(np.mean(scores)), 1),
        recorded_hz=round(float(np.median(hzs)), 1) if hzs else 0.0,
        avg_idle_fraction=round(avg_idle, 3),
        avg_camera_sync_ms=round(avg_sync, 2),
        endpoint_diversity_mm=round(diversity, 1),
        readiness_score=round(readiness, 1),
        recommendations=recs,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="ACT readiness report for an episode set.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    args = ap.parse_args()

    rep = analyze(Path(args.input))
    if rep is None:
        print("No episodes found.")
        return

    print("=" * 60)
    print("  ACT DATASET READINESS REPORT")
    print("=" * 60)
    print(f"  Episodes                 : {rep.num_episodes}")
    print(f"  Avg duration             : {rep.avg_duration_s} s")
    print(f"  Avg joint path length    : {rep.avg_path_length_joint_deg} deg")
    print(f"  Avg smoothness (0..1)    : {rep.avg_smoothness}")
    print(f"  Avg quality score        : {rep.avg_quality_score} / 100")
    print(f"  Recorded action rate     : {rep.recorded_hz} Hz  (ACT export → {ACT_FPS} Hz)")
    print(f"  Avg idle fraction        : {rep.avg_idle_fraction*100:.0f} %")
    print(f"  Camera sync (median gap) : {rep.avg_camera_sync_ms} ms")
    print(f"  Endpoint diversity       : {rep.endpoint_diversity_mm} mm")
    print("-" * 60)
    print(f"  ACT READINESS SCORE      : {rep.readiness_score} / 100")
    print("-" * 60)
    print("  Recommendations:")
    for r in rep.recommendations:
        print(f"    • {r}")
    print("=" * 60)


if __name__ == "__main__":
    main()
