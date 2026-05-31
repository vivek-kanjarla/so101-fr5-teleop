"""
filter_episodes.py — keep the demonstrations worth training ACT on.

Success-driven policy:
  • DISCARD failed episodes (and unlabeled ones — annotate them first).
  • KEEP successful episodes (add --include-partial to also keep partials).
  • RANK kept episodes by quality_score, which for successes is driven by
    duration → re-grasps → pauses → smoothness (success is equal among them).
  • Smoothness alone never removes a successful demo: by default ALL successful
    episodes are kept; --top-percent only trims the lowest-ranked successes, and
    even then ranking uses the full score (smoothness is just 10%).

Non-destructive: selected episodes are symlinked (or --copy) into --output.

Usage:
    python filter_episodes.py                       # keep all successful
    python filter_episodes.py --top-percent 70      # keep best 70% of successful
    python filter_episodes.py --include-partial --copy
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

from score_episodes import score_all


def _link_episode(input_dir: Path, label: str, out_dir: Path, copy: bool) -> None:
    folder = input_dir / label
    if folder.is_dir():
        dst = out_dir / label
        if dst.exists() or dst.is_symlink():
            return
        shutil.copytree(folder, dst) if copy else os.symlink(folder.resolve(), dst)
        return
    for src in input_dir.glob(f"{label}*"):          # legacy flat layout
        dst = out_dir / src.name
        if dst.exists() or dst.is_symlink():
            continue
        (shutil.copy2 if copy else (lambda s, d: os.symlink(Path(s).resolve(), d)))(src, dst)


def main() -> None:
    ap = argparse.ArgumentParser(description="Keep successful demonstrations for ACT.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    ap.add_argument("--output", default="./episodes_filtered", help="Filtered output directory")
    ap.add_argument("--top-percent", type=float, default=100.0,
                    help="Keep this %% of SUCCESSFUL episodes (ranked by score). Default 100.")
    ap.add_argument("--include-partial", action="store_true", help="Also keep partial successes")
    ap.add_argument("--copy", action="store_true", help="Copy instead of symlink")
    args = ap.parse_args()

    input_dir = Path(args.input)
    rows = score_all(input_dir)
    if not rows:
        print("No episodes found.")
        return

    accept = {"success"} | ({"partial"} if args.include_partial else set())
    keepable = [r for r in rows if r["status"] in accept]          # already score-sorted desc
    failed    = [r for r in rows if r["status"] == "fail"]
    unlabeled = [r for r in rows if r["status"] == "unlabeled"]

    if not keepable:
        print(f"No {'/'.join(sorted(accept))} episodes to keep.")
        if unlabeled:
            print(f"  {len(unlabeled)} unlabeled — run: python annotate_episodes.py")
        return

    # Trim only if the user explicitly asked for < 100%.
    keep_n = max(1, round(len(keepable) * args.top_percent / 100.0)) if args.top_percent < 100 else len(keepable)
    keep, trimmed = keepable[:keep_n], keepable[keep_n:]

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    for r in keep:
        _link_episode(input_dir, r["episode"], out_dir, args.copy)

    how = "copied" if args.copy else "symlinked"
    print(f"Kept {len(keep)} episode(s) into {out_dir}/  [{how}]")
    print("  KEPT:    " + ", ".join(f"{r['episode']}({r['quality_score']:.0f})" for r in keep))
    if trimmed:
        print("  TRIMMED (low-ranked successes): " +
              ", ".join(f"{r['episode']}({r['quality_score']:.0f})" for r in trimmed))
    if failed:
        print(f"  DISCARDED {len(failed)} failed: " + ", ".join(r["episode"] for r in failed))
    if unlabeled:
        print(f"  EXCLUDED {len(unlabeled)} unlabeled (run annotate_episodes.py): "
              + ", ".join(r["episode"] for r in unlabeled))
    print(f"\nNext: python convert_to_lerobot.py --input {out_dir} --act")


if __name__ == "__main__":
    main()
