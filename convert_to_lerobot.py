"""
convert_to_lerobot.py — convert recorded episodes to HuggingFace LeRobot v3.0 format.

Reads from ./episodes/ (CSV + JSON + per-camera MP4 / ts.npy / depth.npz bundles
written by logger.py) and writes a LeRobot v3.0 dataset to ./lerobot_dataset/
that can be loaded with:

    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset("my_robot/my_dataset", root="./lerobot_dataset")

Cameras → video keys (one per camera <name>):
    observation.images.wrist_cam   ← D405  (eye-in-hand)
    observation.images.scene_cam   ← D435i (eye-to-hand)

Multi-camera alignment:
    All streams must have one frame per data row. We pick a REFERENCE camera
    (wrist_cam if present), align the joint data to its frame timestamps, then
    resample every other camera + any depth stack onto that same timeline by
    nearest timestamp. This guarantees N data rows == N frames for every camera.

Scene depth:
    Aligned uint16 depth is written losslessly as a per-episode .npy under
    depth/<video_key>/chunk-000/file-{ep:03d}.npy and described in
    meta/info.json["depth_maps"] (depth_scale in metres/unit). It is kept OUT of
    the strict LeRobot `features` dict so the dataset still loads on stock
    LeRobot; wire it into training from the sidecar files as needed.

Output layout:
    lerobot_dataset/
    ├── meta/{info.json, tasks.parquet, episodes/chunk-000/file-000.parquet}
    ├── data/chunk-000/file-000.parquet
    ├── videos/observation.images.<name>/chunk-000/file-{ep:03d}.mp4
    └── depth/observation.images.<name>/chunk-000/file-{ep:03d}.npy   (depth cams)

Usage:
    python convert_to_lerobot.py
    python convert_to_lerobot.py --input ./episodes --output ./lerobot_dataset --fps 30
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

import act_config
from config import (TRIM_ENABLED, TRIM_VEL_NORM_THRESH, TRIM_GRIPPER_RATE_THRESH,
                    TRIM_KEEP_BEFORE_S, TRIM_KEEP_AFTER_S,
                    ACT_FPS, ACT_CHUNK_SIZE, ACT_POLICY_FREQUENCY)
from trimming import TrimConfig, compute_active_range


# ── constants ─────────────────────────────────────────────────────────────────

CODEBASE_VERSION       = "v3.0"
CHUNKS_SIZE            = 1000    # max files per chunk directory
DATA_FILE_SIZE_MB      = 100
VIDEO_FILE_SIZE_MB     = 200

# CSV column groups — must match logger.py
CMD_COLS    = [f"fr5_cmd_j{i}"    for i in range(1, 7)]
ACTUAL_COLS = [f"fr5_actual_j{i}" for i in range(1, 7)]
EEF_COLS    = ["fr5_eef_x_mm", "fr5_eef_y_mm", "fr5_eef_z_mm",
               "fr5_eef_rx_deg", "fr5_eef_ry_deg", "fr5_eef_rz_deg"]
VEL_COLS    = [f"fr5_vel_j{i}"   for i in range(1, 7)]


def _video_key(cam_name: str) -> str:
    return f"observation.images.{cam_name}"


# ── episode discovery + camera resolution ──────────────────────────────────────

def _find_episodes(input_dir: Path) -> list[Path]:
    """Return episode handles, newest layout first.

    New layout : episodes/episode_NNN/meta.json   → returns the meta.json path
    Legacy flat: episodes/episode_*.json          → returns the .json path
    """
    handles: list[Path] = []
    for d in sorted(input_dir.glob("episode_*")):
        if d.is_dir() and (d / "meta.json").exists():
            handles.append(d / "meta.json")
    # Legacy flat episodes (skip any meta.json already collected above).
    for j in sorted(input_dir.glob("episode_*.json")):
        handles.append(j)
    return handles


def _episode_cameras(meta: dict, json_path: Path) -> list[dict]:
    """Resolve the camera bundle files for one episode.

    Folder layout (meta.json): files are {name}.mp4 / {name}_ts.npy / {name}_depth.npz.
    Legacy flat layout: {stem}_{name}.mp4 etc., or a single {stem}_camera.mp4.
    Returns dicts with absolute file paths; only cameras whose video+ts exist
    are returned.
    """
    parent     = json_path.parent
    folder_mode = json_path.name == "meta.json"
    stem        = "" if folder_mode else json_path.stem

    def _paths(name: str):
        if folder_mode:
            return (parent / f"{name}.mp4", parent / f"{name}_ts.npy", parent / f"{name}_depth.npz")
        return (parent / f"{stem}_{name}.mp4", parent / f"{stem}_{name}_ts.npy",
                parent / f"{stem}_{name}_depth.npz")

    cams: list[dict] = []
    cams_meta = meta.get("cameras")
    if cams_meta:
        for name, cm in cams_meta.items():
            video, ts, depth = _paths(name)
            if video.exists() and ts.exists():
                cams.append({
                    "name": name, "video": video, "ts": ts,
                    "depth": depth if (cm.get("has_depth") and depth.exists()) else None,
                    "intrinsics": cm.get("intrinsics"),
                    "depth_scale": cm.get("depth_scale"),
                })
    else:
        # Oldest legacy: single unnamed camera as {stem}_camera.mp4.
        video = parent / f"{stem}_camera.mp4"
        ts    = parent / f"{stem}_camera_ts.npy"
        if video.exists() and ts.exists():
            cams.append({
                "name": "wrist_cam", "video": video, "ts": ts, "depth": None,
                "intrinsics": meta.get("camera_intrinsics"), "depth_scale": None,
            })
    return cams


def _order_cameras(names: set[str]) -> list[str]:
    """wrist_cam first (the natural reference), then the rest alphabetically."""
    rest = sorted(n for n in names if n != "wrist_cam")
    return (["wrist_cam"] if "wrist_cam" in names else []) + rest


# ── data shaping ────────────────────────────────────────────────────────────────

def _load_episode(json_path: Path) -> tuple[dict, pd.DataFrame, list[dict]]:
    with open(json_path) as f:
        meta = json.load(f)
    # Folder layout: data.csv next to meta.json. Legacy flat: same stem + .csv.
    csv_path = (json_path.parent / "data.csv") if json_path.name == "meta.json" \
               else json_path.with_suffix(".csv")
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    cams = _episode_cameras(meta, json_path)
    return meta, df, cams


def _fill_sparse(df: pd.DataFrame) -> pd.DataFrame:
    """Forward-fill then backward-fill sparse actual/eef/vel columns."""
    sparse  = ACTUAL_COLS + EEF_COLS + VEL_COLS
    present = [c for c in sparse if c in df.columns]
    if present:
        df[present] = df[present].ffill().bfill()
    return df


def _resample(df: pd.DataFrame, ref_ts: np.ndarray | None, target_fps: int):
    """Align joint data to the reference timeline.

    Returns (resampled_df, ref_timeline). When ref_ts (reference camera frame
    timestamps) is given, every reference frame becomes one row (nearest joint
    sample). Otherwise the data is strided to approximate target_fps and the
    reference timeline is the kept timestamps.
    """
    if ref_ts is not None and len(ref_ts) > 0:
        ref = np.sort(np.asarray(ref_ts, dtype=np.float64))
        cam_df = pd.DataFrame({"cam_ts": ref})
        df     = df.sort_values("timestamp").reset_index(drop=True)
        merged = pd.merge_asof(cam_df, df,
                               left_on="cam_ts", right_on="timestamp",
                               direction="nearest")
        merged = merged.drop(columns=["cam_ts"]).reset_index(drop=True)
        return merged, ref
    else:
        duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
        stride   = max(1, round((len(df) - 1) / duration / target_fps)) if duration > 0 else 1
        out      = df.iloc[::stride].reset_index(drop=True)
        return out, out["timestamp"].to_numpy(dtype=np.float64)


def _nearest_indices(src_ts: np.ndarray, ref_ts: np.ndarray) -> np.ndarray:
    """For each ref timestamp, the index of the nearest src timestamp.

    Result is non-decreasing (both inputs sorted), so a single forward pass over
    the source video suffices when re-encoding.
    """
    src = np.asarray(src_ts, dtype=np.float64)
    if len(src) == 1:
        return np.zeros(len(ref_ts), dtype=np.int64)
    pos  = np.searchsorted(src, ref_ts)
    pos  = np.clip(pos, 1, len(src) - 1)
    left = src[pos - 1]
    right = src[pos]
    choose_left = (ref_ts - left) <= (right - ref_ts)
    return pos - choose_left.astype(np.int64)


def _reencode_video(src_mp4: Path, frame_indices: np.ndarray, out_mp4: Path, fps: int):
    """Write out_mp4 with exactly len(frame_indices) frames, selected (with
    possible repeats) from src_mp4 by the given non-decreasing indices.
    Returns (written, width, height)."""
    import cv2

    cap = cv2.VideoCapture(str(src_mp4))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open source video {src_mp4}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out_mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"cannot open writer {out_mp4}")

    written = 0
    cur = -1
    frame = None
    for fi in frame_indices:
        while cur < fi:
            ok, f = cap.read()
            if not ok:
                break
            frame = f
            cur += 1
        if frame is not None:
            writer.write(frame)
            written += 1
    writer.release()
    cap.release()
    return written, w, h


def _to_float32_list(df: pd.DataFrame, cols: list[str]) -> list[list[float]]:
    return df[cols].to_numpy(dtype=np.float32).tolist()


def _build_data_rows(df, ep_idx, task_idx, t0, global_index_start) -> dict:
    n = len(df)
    for col in ACTUAL_COLS + EEF_COLS + VEL_COLS:
        if col not in df.columns:
            df[col] = 0.0

    gripper_col = df["gripper_norm"].to_numpy(dtype=np.float32) \
                  if "gripper_norm" in df.columns else np.zeros(n, dtype=np.float32)
    cmd_arr     = df[CMD_COLS].to_numpy(dtype=np.float32)
    action      = np.column_stack([cmd_arr, gripper_col]).tolist()

    timestamps  = (df["timestamp"].to_numpy(dtype=np.float64) - t0).astype(np.float32).tolist()
    next_done       = [False] * n
    next_done[-1]   = True

    return {
        "observation.state":     _to_float32_list(df, ACTUAL_COLS),
        "action":                action,
        # Cartesian state, stored both combined (eef_pose, back-compat) and split.
        # ACT policies often condition on / predict end-effector pose; keeping
        # position and orientation as separate keys lets configs weight them
        # independently (e.g. mm vs deg scaling) without re-slicing.
        "observation.eef_pose":        _to_float32_list(df, EEF_COLS),
        "observation.eef_position":    _to_float32_list(df, EEF_COLS[:3]),
        "observation.eef_orientation": _to_float32_list(df, EEF_COLS[3:]),
        "observation.joint_vel": _to_float32_list(df, VEL_COLS),
        "timestamp":             timestamps,
        "index":                 list(range(global_index_start, global_index_start + n)),
        "episode_index":         [ep_idx] * n,
        "frame_index":           list(range(n)),
        "task_index":            [task_idx] * n,
        "next.done":             next_done,
    }


# ── main ──────────────────────────────────────────────────────────────────────

def convert(input_dir: Path, output_dir: Path, target_fps: int,
            act_mode: bool = False, do_trim: bool = TRIM_ENABLED) -> None:
    # ACT mode pins the dataset rate and forces idle trimming so the exported
    # dataset matches the policy's training/deployment frequency exactly.
    if act_mode:
        target_fps = ACT_FPS
        do_trim = True
    trim_cfg = TrimConfig(TRIM_VEL_NORM_THRESH, TRIM_GRIPPER_RATE_THRESH,
                          TRIM_KEEP_BEFORE_S, TRIM_KEEP_AFTER_S)

    ep_json_paths = _find_episodes(input_dir)
    if not ep_json_paths:
        print(f"No episode_*.json files found in {input_dir}. Nothing to convert.")
        sys.exit(0)

    print(f"Found {len(ep_json_paths)} episode(s) in {input_dir}"
          + (f"  [ACT mode: {target_fps} Hz, trim ON]" if act_mode else
             f"  [fps={target_fps}, trim={'ON' if do_trim else 'OFF'}]"))

    # ── Pass 1: instructions + camera availability (intersection across episodes) ──
    instructions: list[str] = []
    cam_sets: list[set[str]] = []
    depth_cams: set[str] = set()
    cam_dims: dict[str, tuple[int, int]] = {}   # name -> (h, w) from intrinsics
    cam_depth_scale: dict[str, float] = {}

    for jp in ep_json_paths:
        try:
            with open(jp) as f:
                m = json.load(f)
        except Exception:
            continue
        instr = m.get("language_instruction", "").strip()
        if instr not in instructions:
            instructions.append(instr)
        cams = _episode_cameras(m, jp)
        cam_sets.append({c["name"] for c in cams})
        for c in cams:
            if c["depth"] is not None:
                depth_cams.add(c["name"])
            if c.get("depth_scale") is not None:
                cam_depth_scale[c["name"]] = c["depth_scale"]
            ci = c.get("intrinsics") or {}
            if "height" in ci and "width" in ci:
                cam_dims[c["name"]] = (int(ci["height"]), int(ci["width"]))

    if not instructions:
        instructions = [""]
    instructions.sort()
    task_map = {instr: idx for idx, instr in enumerate(instructions)}

    # Cameras present in EVERY episode → consistent video keys for the dataset.
    common_cams = set.intersection(*cam_sets) if cam_sets and all(cam_sets) else set()
    declared_cams = _order_cameras(common_cams)
    reference_cam = declared_cams[0] if declared_cams else None
    depth_declared = [c for c in declared_cams if c in depth_cams]

    if declared_cams:
        print(f"  Cameras: {declared_cams}  (reference={reference_cam})"
              + (f"  depth={depth_declared}" if depth_declared else ""))
    else:
        print("  No camera present in all episodes — exporting without video.")

    # ── output directory layout ───────────────────────────────────────────────
    (output_dir / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "data"  / "chunk-000").mkdir(parents=True, exist_ok=True)

    all_data_rows:     dict[str, list] = {}
    episode_meta_rows: list[dict]      = []
    total_episodes  = 0
    global_frame    = 0

    # ── Pass 2: per-episode conversion ─────────────────────────────────────────
    for jp in ep_json_paths:
        ep_idx = total_episodes

        try:
            meta, df, cams = _load_episode(jp)
        except Exception as exc:
            print(f"  [SKIP] {jp.name}: {exc}")
            continue

        cams_by_name = {c["name"]: c for c in cams}
        if declared_cams and not all(c in cams_by_name for c in declared_cams):
            print(f"  [SKIP] {jp.name}: missing one of {declared_cams}")
            continue

        df = _fill_sparse(df)

        # Automatic idle trimming (before resampling). Indices are relative to the
        # original recorded episode and stored in the episode metadata.
        trim_start, trim_end = 0, len(df)
        if do_trim and len(df) >= 3:
            trim_start, trim_end = compute_active_range(df, trim_cfg)
        df = df.iloc[trim_start:trim_end].reset_index(drop=True)
        if len(df) == 0:
            print(f"  [SKIP] {jp.name}: empty after trim")
            continue

        # Reference timeline = reference-camera frame timestamps within the trimmed
        # window (so every video and every data row stays 1:1 after trimming).
        ref_ts = None
        if reference_cam:
            ref_full = np.load(cams_by_name[reference_cam]["ts"]).astype(np.float64)
            t_lo = float(df["timestamp"].iloc[0])
            t_hi = float(df["timestamp"].iloc[-1])
            ref_ts = ref_full[(ref_full >= t_lo) & (ref_full <= t_hi)]
            if ref_ts.size == 0:
                ref_ts = None
        df, ref_timeline = _resample(df, ref_ts, target_fps)

        if len(df) == 0:
            print(f"  [SKIP] {jp.name}: empty after resampling")
            continue

        n        = len(df)
        t0       = float(df["timestamp"].iloc[0])
        instr    = meta.get("language_instruction", "").strip()
        task_idx = task_map.get(instr, 0)

        rows = _build_data_rows(df, ep_idx, task_idx, t0, global_frame)
        for col, vals in rows.items():
            all_data_rows.setdefault(col, []).extend(vals)

        # Resample each declared camera (and its depth) onto the reference timeline.
        for cam_name in declared_cams:
            cam = cams_by_name[cam_name]
            cam_ts = np.load(cam["ts"]).astype(np.float64)
            idxs   = _nearest_indices(np.sort(cam_ts), ref_timeline)

            vkey    = _video_key(cam_name)
            out_mp4 = output_dir / "videos" / vkey / "chunk-000" / f"file-{ep_idx:03d}.mp4"
            written, w, h = _reencode_video(cam["video"], idxs, out_mp4, target_fps)
            cam_dims.setdefault(cam_name, (h, w))
            if written != n:
                print(f"    [WARN] {cam_name}: wrote {written} frames, expected {n}")

            if cam_name in depth_declared and cam["depth"] is not None:
                with np.load(cam["depth"]) as dz:
                    depth = dz["depth"]                      # (N_src, H, W) uint16
                idxs_d = np.clip(idxs, 0, len(depth) - 1)
                aligned = depth[idxs_d].astype(np.uint16)    # (n, H, W)
                out_npy = output_dir / "depth" / vkey / "chunk-000" / f"file-{ep_idx:03d}.npy"
                out_npy.parent.mkdir(parents=True, exist_ok=True)
                np.save(out_npy, aligned)

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
            "trim_start_index":             int(trim_start),   # vs original recording
            "trim_end_index":               int(trim_end),
        }
        for cam_name in declared_cams:
            vkey = _video_key(cam_name)
            ep_meta[f"videos/{vkey}/chunk_index"]    = 0
            ep_meta[f"videos/{vkey}/file_index"]     = ep_idx
            ep_meta[f"videos/{vkey}/from_timestamp"] = 0.0
        episode_meta_rows.append(ep_meta)

        global_frame   += n
        total_episodes += 1
        cams_str = ",".join(declared_cams) if declared_cams else "none"
        print(f"  [{total_episodes}/{len(ep_json_paths)}] episode_{ep_idx:06d}  "
              f"frames={n}  cams={cams_str}  task={task_idx}")

    if total_episodes == 0:
        print("No valid episodes to write.")
        sys.exit(0)

    # ── write data parquet ────────────────────────────────────────────────────
    data_table = pa.table({
        "observation.state":     pa.array(all_data_rows["observation.state"],     type=pa.list_(pa.float32(), 6)),
        "action":                pa.array(all_data_rows["action"],                type=pa.list_(pa.float32(), 7)),
        "observation.eef_pose":        pa.array(all_data_rows["observation.eef_pose"],        type=pa.list_(pa.float32(), 6)),
        "observation.eef_position":    pa.array(all_data_rows["observation.eef_position"],    type=pa.list_(pa.float32(), 3)),
        "observation.eef_orientation": pa.array(all_data_rows["observation.eef_orientation"], type=pa.list_(pa.float32(), 3)),
        "observation.joint_vel": pa.array(all_data_rows["observation.joint_vel"], type=pa.list_(pa.float32(), 6)),
        "timestamp":             pa.array(all_data_rows["timestamp"],     type=pa.float32()),
        "index":                 pa.array(all_data_rows["index"],         type=pa.int64()),
        "episode_index":         pa.array(all_data_rows["episode_index"], type=pa.int64()),
        "frame_index":           pa.array(all_data_rows["frame_index"],   type=pa.int64()),
        "task_index":            pa.array(all_data_rows["task_index"],    type=pa.int64()),
        "next.done":             pa.array(all_data_rows["next.done"],     type=pa.bool_()),
    })
    pq.write_table(data_table, output_dir / "data" / "chunk-000" / "file-000.parquet",
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
        ("trim_start_index",          pa.int64()),
        ("trim_end_index",            pa.int64()),
    ]
    for cam_name in declared_cams:
        vkey = _video_key(cam_name)
        ep_schema_fields += [
            (f"videos/{vkey}/chunk_index",    pa.int64()),
            (f"videos/{vkey}/file_index",     pa.int64()),
            (f"videos/{vkey}/from_timestamp", pa.float64()),
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

    pq.write_table(pa.table(ep_arrays),
                   output_dir / "meta" / "episodes" / "chunk-000" / "file-000.parquet",
                   compression="snappy")

    # ── write tasks parquet ───────────────────────────────────────────────────
    tasks_df = pd.DataFrame({"task_index": list(task_map.values())},
                            index=pd.Index(list(task_map.keys()), name="task"))
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
        "observation.eef_position": {
            "dtype": "float32", "shape": [3], "names": ["x_mm", "y_mm", "z_mm"],
        },
        "observation.eef_orientation": {
            "dtype": "float32", "shape": [3], "names": ["rx_deg", "ry_deg", "rz_deg"],
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
    for cam_name in declared_cams:
        h, w = cam_dims.get(cam_name, (480, 640))
        features[_video_key(cam_name)] = {
            "dtype": "video", "shape": [h, w, 3],
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
    # Sidecar depth (kept out of `features` so stock LeRobot still loads).
    if depth_declared:
        info["depth_maps"] = {
            _video_key(cam_name): {
                "path":        "depth/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.npy",
                "dtype":       "uint16",
                "shape":       list(cam_dims.get(cam_name, (480, 640))),
                "depth_scale": cam_depth_scale.get(cam_name),
                "units":       "metres = pixel_value * depth_scale (aligned to color)",
            }
            for cam_name in depth_declared
        }

    # ACT metadata: surfaced so training/deployment read consistent values.
    # recommended_policy_frequency ≈ dataset_hz / 2 (safe open-loop ACT query rate).
    if act_mode:
        info["act"] = {
            "dataset_frequency":           target_fps,
            "recommended_chunk_size":      ACT_CHUNK_SIZE,
            "recommended_policy_frequency": ACT_POLICY_FREQUENCY,
        }

    with open(output_dir / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    if act_mode:
        cfg_path = act_config.write_act_config(output_dir, info, ACT_CHUNK_SIZE, ACT_POLICY_FREQUENCY)
        print(f"  ACT metadata + {cfg_path.name} written")

    print(f"\nDone. {total_episodes} episodes → {output_dir}")
    print(f"  total_frames={global_frame}  fps={target_fps}  tasks={len(instructions)}  "
          f"cameras={declared_cams}" + (f"  depth={depth_declared}" if depth_declared else ""))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert SO-101→FR5 episodes to HuggingFace LeRobot v3.0 format."
    )
    parser.add_argument("--input",  default="./episodes",        help="Input episodes directory")
    parser.add_argument("--output", default="./lerobot_dataset", help="Output dataset directory")
    parser.add_argument("--fps",    default=30, type=int,        help="Target FPS (default 30)")
    parser.add_argument("--act",    action="store_true",
                        help=f"ACT export: pin {ACT_FPS} Hz, force idle trimming, write ACT "
                             "metadata + act_config.yaml")
    parser.add_argument("--no-trim", action="store_true", help="Disable automatic idle trimming")
    args = parser.parse_args()
    convert(Path(args.input), Path(args.output), args.fps,
            act_mode=args.act, do_trim=(TRIM_ENABLED and not args.no_trim))


if __name__ == "__main__":
    main()
