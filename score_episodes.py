"""
score_episodes.py — rank recorded demonstrations by quality for ACT curation.

Computes per-episode quality metrics (smoothness, jerk, pauses, efficiency,
duration, grasp count, camera sync) and writes a ranked episode_scores.csv. Use
it to see which demonstrations are dragging down dataset quality before training
ACT, then prune them with filter_episodes.py.

Usage:
    python score_episodes.py
    python score_episodes.py --input ./episodes --output episode_scores.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from config import (SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD,
                    QUALITY_W_SMOOTHNESS, QUALITY_W_DURATION, QUALITY_W_EFFICIENCY,
                    QUALITY_TARGET_DURATION_S, QUALITY_JERK_REF,
                    QUALITY_PAUSE_VEL_THRESH, QUALITY_PAUSE_MIN_S)
from convert_to_lerobot import _find_episodes, _load_episode
from quality import compute_quality, camera_sync_gap_ms, QualityWeights

COLUMNS = ["episode", "quality_score", "smoothness", "ldlj", "rms_jerk_deg_s3",
           "duration_s", "motion_efficiency", "pauses", "grasp_count",
           "gripper_open_events", "num_steps", "camera_sync_ms"]


def _label(handle: Path) -> str:
    return handle.parent.name if handle.name == "meta.json" else handle.stem


def _weights() -> QualityWeights:
    return QualityWeights(
        smoothness=QUALITY_W_SMOOTHNESS, duration=QUALITY_W_DURATION,
        efficiency=QUALITY_W_EFFICIENCY, target_duration_s=QUALITY_TARGET_DURATION_S,
        jerk_ref=QUALITY_JERK_REF, pause_vel_thresh=QUALITY_PAUSE_VEL_THRESH,
        pause_min_s=QUALITY_PAUSE_MIN_S,
    )


def score_all(input_dir: Path) -> list[dict]:
    handles = _find_episodes(input_dir)
    weights = _weights()
    rows: list[dict] = []
    for h in handles:
        try:
            meta, df, cams = _load_episode(h)
        except Exception as exc:
            print(f"  [SKIP] {_label(h)}: {exc}")
            continue
        qm = compute_quality(df, weights, SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD)
        ts_by_cam = {c["name"]: np.load(c["ts"]) for c in cams if c.get("ts") and Path(c["ts"]).exists()}
        sync = camera_sync_gap_ms(ts_by_cam)
        rows.append({
            "episode": _label(h),
            "quality_score": qm.quality_score,
            "smoothness": qm.smoothness,
            "ldlj": qm.ldlj,
            "rms_jerk_deg_s3": qm.rms_jerk_deg_s3,
            "duration_s": qm.duration_s,
            "motion_efficiency": qm.motion_efficiency,
            "pauses": qm.pauses,
            "grasp_count": qm.grasp_count,
            "gripper_open_events": qm.gripper_open_events,
            "num_steps": qm.num_steps,
            "camera_sync_ms": round(sync, 2),
        })
    rows.sort(key=lambda r: r["quality_score"], reverse=True)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description="Rank episodes by demonstration quality.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    ap.add_argument("--output", default="episode_scores.csv", help="Output CSV path")
    args = ap.parse_args()

    rows = score_all(Path(args.input))
    if not rows:
        print("No episodes found.")
        return

    with open(args.output, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)

    print(f"\nRanked {len(rows)} episode(s) → {args.output}\n")
    print(f"  {'rank':<5}{'episode':<16}{'score':>7}{'smooth':>8}{'jerk':>9}{'dur(s)':>8}{'eff':>6}{'pauses':>8}")
    for i, r in enumerate(rows, 1):
        print(f"  {i:<5}{r['episode']:<16}{r['quality_score']:>7.1f}{r['smoothness']:>8.3f}"
              f"{r['rms_jerk_deg_s3']:>9.0f}{r['duration_s']:>8.1f}{r['motion_efficiency']:>6.2f}{r['pauses']:>8}")


if __name__ == "__main__":
    main()
