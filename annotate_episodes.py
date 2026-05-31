"""
annotate_episodes.py — label demonstrations with success / partial / fail.

ACT data curation is success-driven, so each episode needs a manual outcome
label. This tool writes `success` and `partial_success` into the episode's
meta.json (folder layout) or the flat episode_*.json.

Interactive (default): steps through episodes, shows quick stats + the wrist
video path to review, and prompts for a label.

    python annotate_episodes.py                 # label all unlabeled episodes
    python annotate_episodes.py --all            # re-label every episode
    python annotate_episodes.py --list           # show current labels

Non-interactive (scriptable):

    python annotate_episodes.py --episode episode_000 --success
    python annotate_episodes.py --episode episode_000 --partial
    python annotate_episodes.py --episode episode_000 --fail
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from convert_to_lerobot import _find_episodes


def _label_of(handle: Path) -> str:
    return handle.parent.name if handle.name == "meta.json" else handle.stem


def _load(handle: Path) -> dict:
    with open(handle) as f:
        return json.load(f)


def _save(handle: Path, meta: dict) -> None:
    with open(handle, "w") as f:
        json.dump(meta, f, indent=2)


def _set_label(meta: dict, success: bool, partial: bool) -> None:
    meta["success"] = success
    meta["partial_success"] = partial


def _status_str(meta: dict) -> str:
    s, p = meta.get("success"), meta.get("partial_success")
    if s is True:
        return "SUCCESS"
    if p is True:
        return "PARTIAL"
    if s is False:
        return "FAIL"
    return "unlabeled"


def _interactive(handles: list[Path], relabel_all: bool) -> None:
    todo = [h for h in handles if relabel_all or _load(h).get("success") is None
            and _load(h).get("partial_success") is None]
    if not todo:
        print("All episodes already labeled. Use --all to re-label or --list to view.")
        return

    print(f"Labeling {len(todo)} episode(s).  [s]uccess  [p]artial  [f]ail  [k]skip  [q]uit\n")
    for h in todo:
        meta = _load(h)
        q = meta.get("quality") or {}
        vid = h.parent / "wrist_cam.mp4" if h.name == "meta.json" else h.parent / f"{h.stem}_wrist_cam.mp4"
        print(f"── {_label_of(h)}  ({_status_str(meta)})")
        print(f"   instruction: {meta.get('language_instruction','')!r}")
        print(f"   dur={meta.get('duration_s','?')}s  grasps={q.get('grasp_count','?')} "
              f"(regrasps={q.get('regrasp_count','?')})  pauses={q.get('pauses','?')}  "
              f"smoothness={q.get('smoothness','?')}")
        print(f"   review: {vid}")
        while True:
            ans = input("   label [s/p/f/k/q]: ").strip().lower()
            if ans in ("s", "p", "f", "k", "q"):
                break
            print("   (enter s, p, f, k, or q)")
        if ans == "q":
            print("Stopped.")
            return
        if ans == "k":
            continue
        _set_label(meta, success=(ans == "s"), partial=(ans == "p"))
        _save(h, meta)
        print(f"   → {_status_str(meta)}\n")
    print("Done labeling.")


def main() -> None:
    ap = argparse.ArgumentParser(description="Label demonstrations success/partial/fail.")
    ap.add_argument("--input", default="./episodes", help="Episodes directory")
    ap.add_argument("--episode", help="Label one episode by name (e.g. episode_000)")
    ap.add_argument("--success", action="store_true")
    ap.add_argument("--partial", action="store_true")
    ap.add_argument("--fail", action="store_true")
    ap.add_argument("--all", action="store_true", help="Interactive: re-label every episode")
    ap.add_argument("--list", action="store_true", help="Show current labels and exit")
    args = ap.parse_args()

    handles = _find_episodes(Path(args.input))
    if not handles:
        print("No episodes found.")
        return

    if args.list:
        for h in handles:
            print(f"  {_label_of(h):<16} {_status_str(_load(h))}")
        return

    # Non-interactive single-episode labeling.
    if args.episode or args.success or args.partial or args.fail:
        if not args.episode:
            print("Specify --episode when using --success/--partial/--fail.")
            return
        match = [h for h in handles if _label_of(h) == args.episode]
        if not match:
            print(f"Episode '{args.episode}' not found in {args.input}.")
            return
        h = match[0]
        meta = _load(h)
        _set_label(meta, success=args.success, partial=args.partial)  # --fail → both False
        _save(h, meta)
        print(f"{_label_of(h)} → {_status_str(meta)}")
        return

    _interactive(handles, relabel_all=args.all)


if __name__ == "__main__":
    main()
