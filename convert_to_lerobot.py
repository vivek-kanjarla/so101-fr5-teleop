"""
convert_to_lerobot.py — convert recorded episodes to HuggingFace LeRobot v3.0 format.

Reads from ./episodes/ (CSV + JSON + MP4 + .npy bundles written by logger.py) and writes
a LeRobot v3.0 dataset to ./lerobot_dataset/ that can be loaded with:

    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset("my_robot/my_dataset", root="./lerobot_dataset")

Output layout:
    lerobot_dataset/
    ├── meta/
    │   ├── info.json
    │   ├── tasks.parquet
    │   └── episodes/chunk-000/file-000.parquet
    ├── data/chunk-000/file-000.parquet
    └── videos/observation.images.wrist_cam/chunk-000/file-{ep:03d}.mp4

Usage:
    python convert_to_lerobot.py
    python convert_to_lerobot.py --input ./episodes --output ./lerobot_dataset --fps 30
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


# ── constants ─────────────────────────────────────────────────────────────────

CODEBASE_VERSION       = "v3.0"
CHUNKS_SIZE            = 1000    # max files per chunk directory
DATA_FILE_SIZE_MB      = 100
VIDEO_FILE_SIZE_MB     = 200

VIDEO_KEY = "observation.images.wrist_cam"

# CSV column groups — must match logger.py
CMD_COLS    = [f"fr5_cmd_j{i}"    for i in range(1, 7)]
ACTUAL_COLS = [f"fr5_actual_j{i}" for i in range(1, 7)]
EEF_COLS    = ["fr5_eef_x_mm", "fr5_eef_y_mm", "fr5_eef_z_mm",
               "fr5_eef_rx_deg", "fr5_eef_ry_deg", "fr5_eef_rz_deg"]
VEL_COLS    = [f"fr5_vel_j{i}"   for i in range(1, 7)]


# ── helpers ───────────────────────────────────────────────────────────────────

def _find_episodes(input_dir: Path) -> list[Path]:
    return sorted(input_dir.glob("episode_*.json"))


def _load_episode(json_path: Path) -> tuple[dict, pd.DataFrame, np.ndarray | None]:
    with open(json_path) as f:
        meta = json.load(f)

    base     = json_path.parent / json_path.stem
    csv_path = base.with_suffix(".csv")
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)

    ts_path = Path(str(base) + "_camera_ts.npy")
    cam_ts  = np.load(ts_path) if ts_path.exists() else None
    return meta, df, cam_ts


def _fill_sparse(df: pd.DataFrame) -> pd.DataFrame:
    """Forward-fill then backward-fill sparse actual/eef/vel columns."""
    sparse  = ACTUAL_COLS + EEF_COLS + VEL_COLS
    present = [c for c in sparse if c in df.columns]
    if present:
        df[present] = df[present].ffill().bfill()
    return df


def _downsample(df: pd.DataFrame, cam_ts: np.ndarray | None, target_fps: int) -> pd.DataFrame:
    if cam_ts is not None and len(cam_ts) > 0:
        cam_df = pd.DataFrame({"cam_ts": cam_ts}).sort_values("cam_ts").reset_index(drop=True)
        df     = df.sort_values("timestamp").reset_index(drop=True)
        merged = pd.merge_asof(cam_df, df,
                               left_on="cam_ts", right_on="timestamp",
                               direction="nearest")
        return merged.drop(columns=["cam_ts"]).reset_index(drop=True)
    else:
        duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
        stride   = max(1, round((len(df) - 1) / duration / target_fps)) if duration > 0 else 1
        return df.iloc[::stride].reset_index(drop=True)


def _to_float32_list(df: pd.DataFrame, cols: list[str]) -> list[list[float]]:
    return df[cols].to_numpy(dtype=np.float32).tolist()


def _build_data_rows(
    df: pd.DataFrame,
    ep_idx: int,
    task_idx: int,
    t0: float,
    global_index_start: int,
) -> dict:
    """Return a dict of column→list for one episode's data rows."""
    n = len(df)

    for col in ACTUAL_COLS + EEF_COLS + VEL_COLS:
        if col not in df.columns:
            df[col] = 0.0

    gripper_col = df["gripper_norm"].to_numpy(dtype=np.float32) \
                  if "gripper_norm" in df.columns else np.zeros(n, dtype=np.float32)
    cmd_arr     = df[CMD_COLS].to_numpy(dtype=np.float32)
    action      = np.column_stack([cmd_arr, gripper_col]).tolist()

    obs_state  = _to_float32_list(df, ACTUAL_COLS)
    eef_pose   = _to_float32_list(df, EEF_COLS)
    joint_vel  = _to_float32_list(df, VEL_COLS)
    timestamps = (df["timestamp"].to_numpy(dtype=np.float64) - t0).astype(np.float32).tolist()

    next_done       = [False] * n
    next_done[-1]   = True

    return {
        "observation.state":     obs_state,
        "action":                action,
        "observation.eef_pose":  eef_pose,
        "observation.joint_vel": joint_vel,
        "timestamp":             timestamps,
        "index":                 list(range(global_index_start, global_index_start + n)),
        "episode_index":         [ep_idx] * n,
        "frame_index":           list(range(n)),
        "task_index":            [task_idx] * n,
        "next.done":             next_done,
    }


# ── main ──────────────────────────────────────────────────────────────────────

def convert(input_dir: Path, output_dir: Path, target_fps: int) -> None:
    ep_json_paths = _find_episodes(input_dir)
    if not ep_json_paths:
        print(f"No episode_*.json files found in {input_dir}. Nothing to convert.")
        sys.exit(0)

    print(f"Found {len(ep_json_paths)} episode(s) in {input_dir}")

    # First pass: collect unique instructions for stable task index map.
    # Empty/missing instructions get their own "" entry so they are never
    # silently merged with a real task.
    instructions: list[str] = []
    for jp in ep_json_paths:
        try:
            with open(jp) as f:
                m = json.load(f)
            instr = m.get("language_instruction", "").strip()
            if instr not in instructions:
                instructions.append(instr)
        except Exception:
            pass
    if not instructions:
        instructions = [""]
    instructions.sort()
    task_map = {instr: idx for idx, instr in enumerate(instructions)}

    # ── output directory layout ───────────────────────────────────────────────
    (output_dir / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "data"  / "chunk-000").mkdir(parents=True, exist_ok=True)

    # Accumulate all data rows and episode metadata rows across episodes
    all_data_rows:    dict[str, list] = {}
    episode_meta_rows: list[dict]     = []
    total_episodes  = 0
    total_videos    = 0
    global_frame    = 0   # running global frame index

    for jp in ep_json_paths:
        ep_idx = total_episodes   # consecutive; only advances on success

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
        n        = len(df)

        # Build data rows for this episode
        rows = _build_data_rows(df, ep_idx, task_idx, t0, global_frame)
        for col, vals in rows.items():
            all_data_rows.setdefault(col, []).extend(vals)

        # Copy video if present
        src_video = jp.parent / f"{jp.stem}_camera.mp4"
        has_video = src_video.exists()
        if has_video:
            vid_dir = output_dir / "videos" / VIDEO_KEY / "chunk-000"
            vid_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_video, vid_dir / f"file-{ep_idx:03d}.mp4")
            total_videos += 1

        # Episode metadata row
        ep_meta: dict = {
            "episode_index":                ep_idx,
            "tasks":                        [instr],
            "length":                       n,
            "dataset_from_index":           global_frame,
            "dataset_to_index":             global_frame + n,
            "meta/episodes/chunk_index":    0,
            "meta/episodes/file_index":     0,
            "data/chunk_index":             0,
            "data/file_index":              0,
        }
        if has_video:
            ep_meta[f"videos/{VIDEO_KEY}/chunk_index"]    = 0
            ep_meta[f"videos/{VIDEO_KEY}/file_index"]     = ep_idx
            ep_meta[f"videos/{VIDEO_KEY}/from_timestamp"] = 0.0

        episode_meta_rows.append(ep_meta)

        global_frame   += n
        total_episodes += 1
        print(f"  [{total_episodes}/{len(ep_json_paths)}] episode_{ep_idx:06d}  "
              f"frames={n}  video={'yes' if has_video else 'no'}  task={task_idx}")

    if total_episodes == 0:
        print("No valid episodes to write.")
        sys.exit(0)

    # ── write data parquet ────────────────────────────────────────────────────
    has_any_video = total_videos > 0
    data_table = pa.table({
        "observation.state":     pa.array(all_data_rows["observation.state"],
                                          type=pa.list_(pa.float32(), 6)),
        "action":                pa.array(all_data_rows["action"],
                                          type=pa.list_(pa.float32(), 7)),
        "observation.eef_pose":  pa.array(all_data_rows["observation.eef_pose"],
                                          type=pa.list_(pa.float32(), 6)),
        "observation.joint_vel": pa.array(all_data_rows["observation.joint_vel"],
                                          type=pa.list_(pa.float32(), 6)),
        "timestamp":             pa.array(all_data_rows["timestamp"],     type=pa.float32()),
        "index":                 pa.array(all_data_rows["index"],         type=pa.int64()),
        "episode_index":         pa.array(all_data_rows["episode_index"], type=pa.int64()),
        "frame_index":           pa.array(all_data_rows["frame_index"],   type=pa.int64()),
        "task_index":            pa.array(all_data_rows["task_index"],    type=pa.int64()),
        "next.done":             pa.array(all_data_rows["next.done"],     type=pa.bool_()),
    })
    pq.write_table(data_table,
                   output_dir / "data" / "chunk-000" / "file-000.parquet",
                   compression="snappy")

    # ── write episodes parquet ────────────────────────────────────────────────
    ep_schema_fields = [
        ("episode_index",             pa.int64()),
        ("tasks",                     pa.list_(pa.string())),
        ("length",                    pa.int64()),
        ("dataset_from_index",        pa.int64()),
        ("dataset_to_index",          pa.int64()),
        ("meta/episodes/chunk_index", pa.int64()),
        ("meta/episodes/file_index",  pa.int64()),
        ("data/chunk_index",          pa.int64()),
        ("data/file_index",           pa.int64()),
    ]
    if has_any_video:
        ep_schema_fields += [
            (f"videos/{VIDEO_KEY}/chunk_index",    pa.int64()),
            (f"videos/{VIDEO_KEY}/file_index",     pa.int64()),
            (f"videos/{VIDEO_KEY}/from_timestamp", pa.float64()),
        ]

    ep_arrays = {}
    for col, dtype in ep_schema_fields:
        vals = [row.get(col, 0) for row in episode_meta_rows]
        if dtype == pa.list_(pa.string()):
            ep_arrays[col] = pa.array(vals, type=pa.list_(pa.string()))
        elif dtype == pa.float64():
            ep_arrays[col] = pa.array(vals, type=pa.float64())
        else:
            ep_arrays[col] = pa.array(vals, type=pa.int64())

    pq.write_table(
        pa.table(ep_arrays),
        output_dir / "meta" / "episodes" / "chunk-000" / "file-000.parquet",
        compression="snappy",
    )

    # ── write tasks parquet ───────────────────────────────────────────────────
    tasks_df = pd.DataFrame(
        {"task_index": list(task_map.values())},
        index=pd.Index(list(task_map.keys()), name="task"),
    )
    tasks_df.to_parquet(output_dir / "meta" / "tasks.parquet")

    # ── write meta/info.json ──────────────────────────────────────────────────
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
        "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
        "index":         {"dtype": "int64",   "shape": [1], "names": None},
        "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
        "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
        "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
        "next.done":     {"dtype": "bool",    "shape": [1], "names": None},
    }
    if has_any_video:
        features[VIDEO_KEY] = {
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
        "codebase_version":     CODEBASE_VERSION,
        "robot_type":           "fr5",
        "total_episodes":       total_episodes,
        "total_frames":         global_frame,
        "total_tasks":          len(instructions),
        "chunks_size":          CHUNKS_SIZE,
        "data_files_size_in_mb":  DATA_FILE_SIZE_MB,
        "video_files_size_in_mb": VIDEO_FILE_SIZE_MB,
        "fps":                  target_fps,
        "splits":               {"train": f"0:{total_episodes}"},
        "data_path":            "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path":           "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        "features":             features,
    }

    with open(output_dir / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    print(f"\nDone. {total_episodes} episodes → {output_dir}")
    print(f"  total_frames={global_frame}  total_videos={total_videos}  "
          f"fps={target_fps}  tasks={len(instructions)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert SO-101→FR5 episodes to HuggingFace LeRobot v3.0 format."
    )
    parser.add_argument("--input",  default="./episodes",        help="Input episodes directory")
    parser.add_argument("--output", default="./lerobot_dataset", help="Output dataset directory")
    parser.add_argument("--fps",    default=30, type=int,        help="Target FPS (default 30)")
    args = parser.parse_args()
    convert(Path(args.input), Path(args.output), args.fps)


if __name__ == "__main__":
    main()
