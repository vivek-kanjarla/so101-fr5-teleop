"""
score_episodes.py — rank demonstrations by success-centric quality for ACT.

Reads each episode's manual success label (from meta.json; set with
annotate_episodes.py), computes the quality metrics, and writes a ranked
episode_scores.csv. Success dominates the score, so a successful demo with a few
pauses outranks a smooth failure.

Usage:
    python score_episodes.py
    python score_episodes.py --input ./episodes --output episode_scores.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

import config
from config import SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD
from convert_to_lerobot import _find_episodes, _load_episode
from quality import compute_quality, camera_sync_gap_ms, weights_from

COLUMNS = ["episode", "status", "quality_score", "success_quality", "duration_s",
           "grasp_count", "regrasp_count", "pauses", "smoothness",
           "motion_efficiency", "num_steps", "camera_sync_ms"]


def _label(handle: Path) -> str:
    return handle.parent.name if handle.name == "meta.json" else handle.stem


def _status(meta: dict) -> str:
    if meta.get("success") is True:
        return "success"
    if meta.get("partial_success") is True:
        return "partial"
    if meta.get("success") is False:
        return "fail"
    return "unlabeled"


def score_all(input_dir: Path) -> list[dict]:
    handles = _find_episodes(input_dir)
    weights = weights_from(config)
    rows: list[dict] = []
    for h in handles:
        try:
            meta, df, cams = _load_episode(h)
        except Exception as exc:
            print(f"  [SKIP] {_label(h)}: {exc}")
            continue
        qm = compute_quality(
            df, weights,
            success=meta.get("success"), partial_success=meta.get("partial_success"),
            gripper_open_thr=SO101_GRIPPER_OPEN_THRESHOLD,
            gripper_close_thr=SO101_GRIPPER_CLOSE_THRESHOLD,
        )
        ts_by_cam = {c["name"]: np.load(c["ts"]) for c in cams if c.get("ts") and Path(c["ts"]).exists()}
        rows.append({
            "episode": _label(h),
            "status": _status(meta),
            "quality_score": qm.quality_score,
            "success_quality": qm.success_quality,
            "duration_s": qm.duration_s,
            "grasp_count": qm.grasp_count,
            "regrasp_count": qm.regrasp_count,
            "pauses": qm.pauses,
            "smoothness": qm.smoothness,
            "motion_efficiency": qm.motion_efficiency,   # diagnostic only
            "num_steps": qm.num_steps,
            "camera_sync_ms": round(camera_sync_gap_ms(ts_by_cam), 2),
        })
    rows.sort(key=lambda r: r["quality_score"], reverse=True)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description="Rank episodes by success-centric quality.")
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

    n_unlabeled = sum(1 for r in rows if r["status"] == "unlabeled")
    print(f"\nRanked {len(rows)} episode(s) → {args.output}"
          + (f"   ({n_unlabeled} unlabeled — run annotate_episodes.py)" if n_unlabeled else "") + "\n")
    print(f"  {'rank':<5}{'episode':<16}{'status':<11}{'score':>7}{'dur(s)':>8}"
          f"{'grasp':>7}{'regr':>6}{'paus':>6}{'smooth':>8}")
    for i, r in enumerate(rows, 1):
        print(f"  {i:<5}{r['episode']:<16}{r['status']:<11}{r['quality_score']:>7.1f}"
              f"{r['duration_s']:>8.1f}{r['grasp_count']:>7}{r['regrasp_count']:>6}"
              f"{r['pauses']:>6}{r['smoothness']:>8.3f}")


if __name__ == "__main__":
    main()
