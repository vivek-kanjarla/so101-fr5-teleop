"""
filter_episodes.py — keep only the highest-quality demonstrations for ACT.

Ranks episodes by quality_score (via score_episodes) and selects the top N%.
Non-destructive: the originals are never touched — selected episodes are linked
(or copied) into a separate directory you then point convert_to_lerobot.py at.

Why for ACT: a few sloppy demos (jerky, hesitant, wrong-grasp) measurably degrade
a behaviour-cloning policy. Training on a clean top-percentile subset usually beats
training on everything.

Usage:
    python filter_episodes.py --top-percent 70
    python filter_episodes.py --top-percent 50 --output ./episodes_filtered --copy
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

from score_episodes import score_all


def _select(rows: list[dict], top_percent: float) -> tuple[list[dict], list[dict]]:
    keep_n = max(1, round(len(rows) * top_percent / 100.0))
    return rows[:keep_n], rows[keep_n:]   # rows are pre-sorted desc by score


def _link_episode(input_dir: Path, label: str, out_dir: Path, copy: bool) -> None:
    """Materialise one episode (folder or flat file group) into out_dir."""
    folder = input_dir / label
    if folder.is_dir():
        dst = out_dir / label
        if dst.exists() or dst.is_symlink():
            return
        if copy:
            shutil.copytree(folder, dst)
        else:
            os.symlink(folder.resolve(), dst)
        return
    # Legacy flat layout: link every sibling file sharing the episode stem.
    for src in input_dir.glob(f"{label}*"):
        dst = out_dir / src.name
        if dst.exists() or dst.is_symlink():
            continue
        (shutil.copy2 if copy else (lambda s, d: os.symlink(Path(s).resolve(), d)))(src, dst)


def main() -> None:
    ap = argparse.ArgumentParser(description="Keep the top-percentile demonstrations.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    ap.add_argument("--output", default="./episodes_filtered", help="Filtered output directory")
    ap.add_argument("--top-percent", type=float, default=70.0, help="Percent of episodes to keep")
    ap.add_argument("--copy", action="store_true", help="Copy episodes instead of symlinking")
    args = ap.parse_args()

    input_dir = Path(args.input)
    rows = score_all(input_dir)
    if not rows:
        print("No episodes found.")
        return

    keep, drop = _select(rows, args.top_percent)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    for r in keep:
        _link_episode(input_dir, r["episode"], out_dir, args.copy)

    cutoff = keep[-1]["quality_score"]
    print(f"Kept top {args.top_percent:.0f}% → {len(keep)}/{len(rows)} episodes "
          f"(score ≥ {cutoff:.1f}) into {out_dir}/  [{'copied' if args.copy else 'symlinked'}]")
    print("  KEPT: " + ", ".join(f"{r['episode']}({r['quality_score']:.0f})" for r in keep))
    if drop:
        print("  DROPPED: " + ", ".join(f"{r['episode']}({r['quality_score']:.0f})" for r in drop))
    print(f"\nNext: python convert_to_lerobot.py --input {out_dir} --act")


if __name__ == "__main__":
    main()
