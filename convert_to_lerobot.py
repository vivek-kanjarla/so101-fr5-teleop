"""
convert_to_lerobot.py — convert recorded episodes to HuggingFace LeRobot v2.0 format.

Reads from ./episodes/ (CSV + JSON + MP4 + .npy bundles written by logger.py) and writes
a LeRobot v2.0 dataset to ./lerobot_dataset/:

    lerobot_dataset/
    ├── meta/info.json
    ├── data/chunk-000/episode_XXXXXX.parquet
    └── videos/observation.images.wrist_cam/chunk-000/episode_XXXXXX.mp4

Usage:
    python convert_to_lerobot.py
    python convert_to_lerobot.py --input ./episodes --output ./lerobot_dataset --fps 30
"""

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


# ── constants ─────────────────────────────────────────────────────────────────

CHUNKS_SIZE = 1000   # episodes per chunk folder

# CSV column groups (must match logger.py)
SO101_COLS = [
    "so101_shoulder_pan", "so101_shoulder_lift", "so101_elbow_flex",
    "so101_wrist_flex",   "so101_wrist_roll",
]
CMD_COLS    = [f"fr5_cmd_j{i}"    for i in range(1, 7)]
ACTUAL_COLS = [f"fr5_actual_j{i}" for i in range(1, 7)]
EEF_COLS    = ["fr5_eef_x_mm", "fr5_eef_y_mm", "fr5_eef_z_mm",
               "fr5_eef_rx_deg", "fr5_eef_ry_deg", "fr5_eef_rz_deg"]
VEL_COLS    = [f"fr5_vel_j{i}"   for i in range(1, 7)]


# ── helpers ───────────────────────────────────────────────────────────────────

def _find_episodes(input_dir: Path) -> list[Path]:
    """Return sorted list of episode JSON sidecar paths."""
    return sorted(input_dir.glob("episode_*.json"))


def _load_episode(json_path: Path) -> tuple[dict, pd.DataFrame, np.ndarray | None]:
    """Load JSON sidecar, CSV, and optional camera timestamps for one episode."""
    with open(json_path) as f:
        meta = json.load(f)

    stem = json_path.stem   # e.g. "episode_1716123456789"
    base = json_path.parent / stem

    csv_path = base.with_suffix(".csv")
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)

    ts_path = Path(str(base) + "_camera_ts.npy")
    cam_ts = np.load(ts_path) if ts_path.exists() else None

    return meta, df, cam_ts


def _fill_sparse(df: pd.DataFrame) -> pd.DataFrame:
    """Forward-fill then backward-fill sparse actual/eef/vel columns."""
    sparse = ACTUAL_COLS + EEF_COLS + VEL_COLS
    present = [c for c in sparse if c in df.columns]
    if present:
        df[present] = df[present].ffill().bfill()
    return df


def _downsample(df: pd.DataFrame, cam_ts: np.ndarray | None, target_fps: int) -> pd.DataFrame:
    """Return a downsampled DataFrame aligned to camera frames when available."""
    if cam_ts is not None and len(cam_ts) > 0:
        # Nearest-neighbour merge: one output row per camera frame
        cam_df = pd.DataFrame({"cam_ts": cam_ts})
        cam_df = cam_df.sort_values("cam_ts").reset_index(drop=True)
        df = df.sort_values("timestamp").reset_index(drop=True)
        merged = pd.merge_asof(
            cam_df, df,
            left_on="cam_ts", right_on="timestamp",
            direction="nearest",
        )
        merged = merged.drop(columns=["cam_ts"])
        return merged.reset_index(drop=True)
    else:
        # Uniform stride — keep every N-th row
        stride = max(1, round(len(df) / max(1, int(len(df) / (1.0 / target_fps)
                                              * (df["timestamp"].iloc[-1] - df["timestamp"].iloc[0])
                                              + 0.5))))
        # Simpler: derive stride from recorded loop rate
        duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
        if duration > 0:
            recorded_hz = (len(df) - 1) / duration
            stride = max(1, round(recorded_hz / target_fps))
        else:
            stride = 1
        return df.iloc[::stride].reset_index(drop=True)


def _to_float32_list(df: pd.DataFrame, cols: list[str]) -> list[list[float]]:
    """Extract columns as list of float32 Python lists (one per row)."""
    arr = df[cols].to_numpy(dtype=np.float32)
    return arr.tolist()


def _build_parquet(
    df: pd.DataFrame,
    ep_idx: int,
    task_idx: int,
    t0: float,
) -> pa.Table:
    """Build a PyArrow table for one episode."""
    n = len(df)

    # Ensure all required columns exist; fill with zeros if missing
    for col in ACTUAL_COLS + EEF_COLS + VEL_COLS:
        if col not in df.columns:
            df[col] = 0.0

    obs_state = _to_float32_list(df, ACTUAL_COLS)

    # action = fr5_cmd_j1-6 + gripper_norm (7D)
    gripper_col = df["gripper_norm"].to_numpy(dtype=np.float32) if "gripper_norm" in df.columns \
                  else np.zeros(n, dtype=np.float32)
    cmd_arr = df[CMD_COLS].to_numpy(dtype=np.float32)
    action = np.column_stack([cmd_arr, gripper_col]).tolist()

    eef_pose   = _to_float32_list(df, EEF_COLS)
    joint_vel  = _to_float32_list(df, VEL_COLS)
    timestamps = (df["timestamp"].to_numpy(dtype=np.float64) - t0).astype(np.float32).tolist()
    next_done  = [False] * n
    next_done[-1] = True

    table = pa.table({
        "observation.state":        pa.array(obs_state,   type=pa.list_(pa.float32(), 6)),
        "action":                   pa.array(action,      type=pa.list_(pa.float32(), 7)),
        "observation.eef_pose":     pa.array(eef_pose,    type=pa.list_(pa.float32(), 6)),
        "observation.joint_vel":    pa.array(joint_vel,   type=pa.list_(pa.float32(), 6)),
        "timestamp":                pa.array(timestamps,  type=pa.float32()),
        "episode_index":            pa.array([ep_idx] * n, type=pa.int64()),
        "frame_index":              pa.array(list(range(n)), type=pa.int64()),
        "task_index":               pa.array([task_idx] * n, type=pa.int64()),
        "next.done":                pa.array(next_done,  type=pa.bool_()),
    })
    return table


# ── main ──────────────────────────────────────────────────────────────────────

def convert(input_dir: Path, output_dir: Path, target_fps: int) -> None:
    ep_json_paths = _find_episodes(input_dir)
    if not ep_json_paths:
        print(f"No episode_*.json files found in {input_dir}. Nothing to convert.")
        sys.exit(0)

    print(f"Found {len(ep_json_paths)} episode(s) in {input_dir}")

    # First pass: collect all unique language instructions → stable task index
    instructions: list[str] = []
    for jp in ep_json_paths:
        try:
            with open(jp) as f:
                m = json.load(f)
            instr = m.get("language_instruction", "").strip()
            if instr and instr not in instructions:
                instructions.append(instr)
        except Exception:
            pass
    if not instructions:
        instructions = [""]
    instructions.sort()
    task_map = {instr: idx for idx, instr in enumerate(instructions)}

    # Prepare output directories
    meta_dir = output_dir / "meta"
    data_dir = output_dir / "data"
    vid_dir  = output_dir / "videos" / "observation.images.wrist_cam"
    meta_dir.mkdir(parents=True, exist_ok=True)

    total_frames    = 0
    total_videos    = 0
    episode_records = []   # (ep_idx, num_frames, task_idx, has_video)

    for ep_idx, jp in enumerate(ep_json_paths):
        chunk_name  = f"chunk-{ep_idx // CHUNKS_SIZE:03d}"
        ep_name     = f"episode_{ep_idx:06d}"

        parquet_out = data_dir / chunk_name / f"{ep_name}.parquet"
        video_out   = vid_dir  / chunk_name / f"{ep_name}.mp4"

        try:
            meta, df, cam_ts = _load_episode(jp)
        except Exception as exc:
            print(f"  [SKIP] {jp.name}: {exc}")
            continue

        df = _fill_sparse(df)
        df = _downsample(df, cam_ts, target_fps)

        if len(df) == 0:
            print(f"  [SKIP] {jp.name}: empty after downsampling")
            continue

        t0       = float(df["timestamp"].iloc[0])
        instr    = meta.get("language_instruction", "").strip()
        task_idx = task_map.get(instr, 0)

        table = _build_parquet(df, ep_idx, task_idx, t0)

        parquet_out.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, parquet_out, compression="snappy")

        # Copy video if present
        src_video = jp.parent / f"{jp.stem}_camera.mp4"
        has_video = False
        if src_video.exists():
            video_out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_video, video_out)
            has_video = True
            total_videos += 1

        total_frames += len(df)
        episode_records.append((ep_idx, len(df), task_idx, has_video))
        print(f"  [{ep_idx+1}/{len(ep_json_paths)}] {ep_name}  "
              f"frames={len(df)}  video={'yes' if has_video else 'no'}  task={task_idx}")

    total_episodes = len(episode_records)
    total_chunks   = max(1, (total_episodes + CHUNKS_SIZE - 1) // CHUNKS_SIZE)

    # ── meta/info.json ────────────────────────────────────────────────────────
    has_any_video = total_videos > 0
    features: dict = {
        "observation.state": {
            "dtype": "float32", "shape": [6],
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        },
        "action": {
            "dtype": "float32", "shape": [7],
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
        },
        "observation.eef_pose": {
            "dtype": "float32", "shape": [6],
            "names": ["x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"],
        },
        "observation.joint_vel": {
            "dtype": "float32", "shape": [6],
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        },
        "timestamp":     {"dtype": "float32", "shape": [1]},
        "frame_index":   {"dtype": "int64",   "shape": [1]},
        "episode_index": {"dtype": "int64",   "shape": [1]},
        "task_index":    {"dtype": "int64",   "shape": [1]},
        "next.done":     {"dtype": "bool",    "shape": [1]},
    }
    if has_any_video:
        features["observation.images.wrist_cam"] = {
            "dtype": "video", "shape": [480, 640, 3],
            "names": ["height", "width", "channel"],
            "info": {
                "video.fps":          target_fps,
                "video.codec":        "mp4v",
                "video.pix_fmt":      "yuv420p",
                "video.is_depth_map": False,
            },
        }

    info = {
        "codebase_version": "v2.0",
        "robot_type":       "fr5",
        "fps":              target_fps,
        "video":            has_any_video,
        "total_episodes":   total_episodes,
        "total_frames":     total_frames,
        "total_tasks":      len(instructions),
        "total_videos":     total_videos,
        "total_chunks":     total_chunks,
        "chunks_size":      CHUNKS_SIZE,
        "features":         features,
        "tasks": [
            {"task_index": idx, "task": instr}
            for instr, idx in sorted(task_map.items(), key=lambda kv: kv[1])
        ],
    }

    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    print(f"\nDone. {total_episodes} episodes → {output_dir}")
    print(f"  total_frames={total_frames}  total_videos={total_videos}  "
          f"fps={target_fps}  tasks={len(instructions)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert SO-101→FR5 episodes to HuggingFace LeRobot v2.0 format."
    )
    parser.add_argument("--input",  default="./episodes",        help="Input episodes directory")
    parser.add_argument("--output", default="./lerobot_dataset", help="Output dataset directory")
    parser.add_argument("--fps",    default=30, type=int,        help="Target FPS (default 30)")
    args = parser.parse_args()

    convert(Path(args.input), Path(args.output), args.fps)


if __name__ == "__main__":
    main()
