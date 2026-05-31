"""
analyze_dataset.py — success-driven ACT readiness report.

ACT quality is bounded by how many *successful* demonstrations exist, not by jerk.
This report is gated primarily on success rate / count of successful episodes;
smoothness, pauses, and camera sync are reported as secondary diagnostics.

Usage:
    python analyze_dataset.py
    python analyze_dataset.py --input ./episodes
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import config
from config import (SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD, ACT_FPS,
                    QUALITY_EXPECTED_GRASPS)
from convert_to_lerobot import _find_episodes, _load_episode
from quality import compute_quality, camera_sync_gap_ms, weights_from

MIN_SUCCESSFUL    = 50      # successful demos ACT typically wants
MIN_SUCCESS_RATE  = 0.6     # of labeled attempts
MAX_CAMERA_SYNC_MS = 33.0


@dataclass
class DatasetReport:
    total: int
    n_success: int
    n_partial: int
    n_fail: int
    n_unlabeled: int
    success_rate: float            # success / labeled
    avg_duration_s: float          # over successful
    avg_grasps: float
    avg_regrasps: float
    avg_pauses: float
    avg_smoothness: float
    avg_camera_sync_ms: float
    readiness: str
    recommendations: list[str]


def analyze(input_dir: Path) -> DatasetReport | None:
    handles = _find_episodes(input_dir)
    if not handles:
        return None
    weights = weights_from(config)

    n_success = n_partial = n_fail = n_unlabeled = 0
    durs, grasps, regrasps, pauses, smooths, syncs = [], [], [], [], [], []

    for h in handles:
        try:
            meta, df, cams = _load_episode(h)
        except Exception:
            continue
        s, p = meta.get("success"), meta.get("partial_success")
        is_success = s is True
        if is_success:        n_success += 1
        elif p is True:       n_partial += 1
        elif s is False:      n_fail += 1
        else:                 n_unlabeled += 1

        qm = compute_quality(df, weights, success=s, partial_success=p,
                             gripper_open_thr=SO101_GRIPPER_OPEN_THRESHOLD,
                             gripper_close_thr=SO101_GRIPPER_CLOSE_THRESHOLD)
        # Aggregate kinematic stats over SUCCESSFUL demos (the data ACT will use).
        if is_success:
            durs.append(qm.duration_s); grasps.append(qm.grasp_count)
            regrasps.append(qm.regrasp_count); pauses.append(qm.pauses)
            smooths.append(qm.smoothness)
        ts_by_cam = {c["name"]: np.load(c["ts"]) for c in cams if c.get("ts") and Path(c["ts"]).exists()}
        sg = camera_sync_gap_ms(ts_by_cam)
        if sg > 0:
            syncs.append(sg)

    total = n_success + n_partial + n_fail + n_unlabeled
    labeled = n_success + n_partial + n_fail
    success_rate = (n_success / labeled) if labeled else 0.0
    mean = lambda xs: round(float(np.mean(xs)), 3) if xs else 0.0

    # Verdict — PRIMARILY on successful-demo count, then success rate.
    recs: list[str] = []
    if n_unlabeled:
        recs.append(f"{n_unlabeled} unlabeled episode(s) — run annotate_episodes.py "
                    "(they don't count toward readiness).")
    if n_success >= MIN_SUCCESSFUL and success_rate >= MIN_SUCCESS_RATE:
        readiness = "ACT-ready"
    else:
        readiness = "Needs more successful demonstrations"
        if n_success < MIN_SUCCESSFUL:
            recs.append(f"Only {n_success} successful demos (recommend ≥ {MIN_SUCCESSFUL}).")
        if labeled and success_rate < MIN_SUCCESS_RATE:
            recs.append(f"Low success rate {success_rate*100:.0f}% (target ≥ {MIN_SUCCESS_RATE*100:.0f}%) "
                        "— task or teleop reliability needs work.")
    # Secondary diagnostics (do NOT gate readiness).
    if regrasps and float(np.mean(regrasps)) > 1.0:
        recs.append(f"High re-grasp rate (avg {np.mean(regrasps):.1f} extra grasps vs "
                    f"{QUALITY_EXPECTED_GRASPS} expected) — diagnostic only.")
    if syncs and float(np.mean(syncs)) > MAX_CAMERA_SYNC_MS:
        recs.append(f"Camera sync {np.mean(syncs):.0f} ms > {MAX_CAMERA_SYNC_MS:.0f} ms (diagnostic).")
    if not recs:
        recs.append("Dataset suitable for ACT.")

    return DatasetReport(
        total=total, n_success=n_success, n_partial=n_partial, n_fail=n_fail,
        n_unlabeled=n_unlabeled, success_rate=round(success_rate, 3),
        avg_duration_s=mean(durs), avg_grasps=mean(grasps), avg_regrasps=mean(regrasps),
        avg_pauses=mean(pauses), avg_smoothness=mean(smooths),
        avg_camera_sync_ms=mean(syncs), readiness=readiness, recommendations=recs,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Success-driven ACT readiness report.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    args = ap.parse_args()

    rep = analyze(Path(args.input))
    if rep is None:
        print("No episodes found.")
        return

    print("=" * 60)
    print("  ACT DATASET READINESS REPORT")
    print("=" * 60)
    print(f"  Episodes (total)         : {rep.total}")
    print(f"    success / partial / fail / unlabeled : "
          f"{rep.n_success} / {rep.n_partial} / {rep.n_fail} / {rep.n_unlabeled}")
    print(f"  Success rate (of labeled): {rep.success_rate*100:.0f} %")
    print(f"  -- over successful demos --")
    print(f"  Avg duration             : {rep.avg_duration_s} s")
    print(f"  Avg grasps               : {rep.avg_grasps}  (expected {QUALITY_EXPECTED_GRASPS})")
    print(f"  Avg re-grasps            : {rep.avg_regrasps}")
    print(f"  Avg pauses               : {rep.avg_pauses}")
    print(f"  Avg smoothness (diag)    : {rep.avg_smoothness}")
    print(f"  Camera sync (diag)       : {rep.avg_camera_sync_ms} ms  (export → {ACT_FPS} Hz)")
    print("-" * 60)
    print(f"  VERDICT: {rep.readiness}")
    print("-" * 60)
    for r in rep.recommendations:
        print(f"    • {r}")
    print("=" * 60)


if __name__ == "__main__":
    main()
