# Dual-Camera Capture, Smoothing, Fault Recovery & Episode Recording

This document describes the changes added on the `feature/smooth-trajectory` branch to:

1. Add a **second camera** (Intel RealSense **D435i**, fixed *eye-to-hand*) alongside the wrist-mounted **D405** (*eye-in-hand*), recorded into every episode and exported to LeRobot.
2. Use **RealSense hardware (global-time) timestamps** for tighter cross-stream alignment.
3. Smooth the teleoperation with a **One-Euro input filter** on the SO-101 joints and a higher ServoJ trajectory filter.
4. Harden **FR5 fault recovery** so a latched servo fault can be cleared mid-session.
5. Save each episode in its **own sequentially-numbered folder**.

All changes were verified live on hardware (camera capture, timestamps) and on synthetic data (the LeRobot converter, the per-episode folder layout, back-compat).

---

## 1. Cameras

| Role | Camera | Serial | Streams | LeRobot key |
|---|---|---|---|---|
| **Eye-in-hand** (wrist) | D405 | `409122273756` | color | `observation.images.wrist_cam` |
| **Eye-to-hand** (fixed) | D435i | `420122071835` | color **+ aligned depth** | `observation.images.scene_cam` |

### Hardware requirement: both on USB 3.0
RealSense depth + RGB needs USB 3.0 (SuperSpeed, 5000 Mbps). Verify per device:

```bash
for d in /sys/bus/usb/devices/*; do
  [ "$(cat $d/idVendor 2>/dev/null)" = "8086" ] && \
    echo "$(cat $d/product): $(cat $d/speed) Mbps"
done
# want 5000 (or 10000), NOT 480 (= USB 2.0)
```

A camera reporting **480 Mbps** is on a USB 2.0 port/cable and will not stream full depth+RGB. Move it to a SuperSpeed port (blue / "SS" marked) **and** use a USB 3.x cable. Reseating in the *same* 2.0 port does nothing — the port itself must be SuperSpeed.

### Selection by serial
Each camera is opened by **serial number** (`config.py: CAMERAS`), so the wrist and scene devices are never confused. (The previous single-camera code opened "the first device" with no filter, which could grab the wrong camera.)

### Hardware timestamps
With `CAMERA_USE_HW_TIMESTAMP = True`, each sensor enables **global time** so `frame.get_timestamp()` is reported on the host epoch — comparable to the `time.time()` used by the joint log, but without thread-scheduling/USB jitter. Measured on hardware: frame-interval jitter dropped to ~0.01–0.8 ms (vs several ms with arrival timestamps). Falls back to `time.time()` automatically if a device only exposes its raw hardware clock.

> Note: the D405 and D435i are **not** hardware-genlocked (the D405 has no multi-cam sync port). The two free-running 30 fps streams have an inherent ~half-frame (~16 ms) phase offset; hardware timestamps measure it accurately so nearest-frame alignment in the converter is precise.

---

## 2. Smoothing

### One-Euro filter on the SO-101 leader (`one_euro.py`)
Adaptive low-pass on the leader joints at the 125 Hz control rate: filters hard when held still (kills tremor/encoder jitter), barely at all on fast moves (no lag). Verified: **67% jitter reduction at rest**, ~8 ms to track a fast step, 0.22° error tracking a 0.5 Hz sine.

Config (`config.py`):
```python
SO101_FILTER_ENABLED    = True
SO101_FILTER_MIN_CUTOFF = 1.0   # Hz — lower = smoother at rest, more lag
SO101_FILTER_BETA       = 0.7   # higher = less lag on fast moves
SO101_FILTER_DCUTOFF    = 1.0
```
The filter is reset on re-home (`H`) to drop stale state across the discontinuity.

### ServoJ trajectory filter
`FR5_FILTER_T` raised `0.04 → 0.06` s for smoother follower motion (slightly more lag).

> **Not** used: SLERP. SLERP interpolates *orientations* (quaternions) and only applies to Cartesian/task-space control. This pipeline is joint-space ServoJ, where per-joint linear interpolation is correct and SLERP would distort the motion.

---

## 3. Joint mapping amplitudes

`JOINT_AMP` was lowered progressively after a too-large follower swing tripped a servo fault:

```
[1.50, 2.00, 2.00, 3.00, 1.50]   # original (3.0x wrist_flex → ~75° swing → ServoJ error 14)
[1.25, 1.50, 1.50, 2.00, 1.25]
[1.00, 1.25, 1.25, 1.50, 1.00]   # current — near-1:1, gentler follower
#  pan   lift   elbow  wflex  wroll
```

---

## 4. FR5 fault recovery

A ServoJ fault (e.g. joint-limit / over-speed → **error 14**) latches the controller into a not-ready state where `ServoMoveStart` returns **error 99**. `ResetAllError` alone does not clear it.

`FR5Controller.recover()` (`fr5.py`) mirrors the full `connect()` init sequence:

```
ServoMoveEnd → StopMove → ResetAllError → Mode(0) → RobotEnable(1) → 0.5 s settle → ServoMoveStart
```

The teleop loop calls `recover()` after 20 consecutive errors, then resyncs the rate limiter to the robot's actual position (so the first post-recovery command is a small delta, not a jump) and clears the error counter.

To clear a fault manually before a run, just restart `teleop.py` (its `connect()` runs the same sequence), or:
```bash
python3 -c "from fr5 import FR5Controller; r=FR5Controller(); r.connect(); r.disconnect()"
```

---

## 5. Per-episode folder layout

Each recording is saved in its own sequentially-numbered folder (index = highest existing `episode_NNN` + 1):

```
episodes/
└── episode_000/
    ├── data.csv            # 125 Hz joint / EEF / gripper data
    ├── meta.json           # metadata + per-camera intrinsics + depth_scale
    ├── wrist_cam.mp4        wrist_cam_ts.npy
    ├── scene_cam.mp4        scene_cam_ts.npy
    └── scene_cam_depth.npz  # aligned uint16 depth stack (N, H, W)
```

`meta.json` includes `episode_index`, `episode_id` (ms timestamp), `language_instruction`, and a `cameras` block per camera.

---

## 6. Recording workflow

```bash
cd ~/so101-fr5-teleop

# 0. (once) verify connectivity
python3 check_network.py
python3 check_hardware.py

# 1. set the task label (becomes language_instruction)
echo "pick up the block and place it in the bin" > episode_instruction.txt

# 2. run teleop — in an interactive shell so Space (e-stop) works
python3 teleop.py
```

In the session:
1. Wait for `Ready.` and **both** cameras' `using hardware (global-time) timestamps`, plus `[FILTER] One-Euro ...`.
2. `R` → start recording (`[REC] Recording started ...`).
3. Move the SO-101 to perform the task.
4. `R` → stop and save (`[LOGGER] Episode NNN saved → episodes/episode_NNN/`).
5. `Ctrl-C` → graceful exit.

**Controls:** `Space` = e-stop, `R` = record toggle, `H` = re-home, `Ctrl-C` = quit.

---

## 7. LeRobot export

```bash
python3 convert_to_lerobot.py                                   # ./episodes → ./lerobot_dataset
python3 convert_to_lerobot.py --input ./episodes --output ./lerobot_dataset --fps 30
```

Output (LeRobot v3.0):
```
lerobot_dataset/
├── meta/{info.json, tasks.parquet, episodes/chunk-000/file-000.parquet}
├── data/chunk-000/file-000.parquet
├── videos/observation.images.wrist_cam/chunk-000/file-{ep:03d}.mp4
├── videos/observation.images.scene_cam/chunk-000/file-{ep:03d}.mp4
└── depth/observation.images.scene_cam/chunk-000/file-{ep:03d}.npy
```

**Alignment:** the converter picks the **wrist_cam** timestamps as the reference timeline, aligns the 125 Hz joint data to it (`merge_asof`, nearest), and resamples scene_cam color + depth onto the same timeline by nearest timestamp — so every camera and every data row has exactly one frame per index (a LeRobot requirement). Output dataset rate ≈ 30 Hz.

**Depth:** stored losslessly as per-episode `.npy` under `depth/...` and described in `meta/info.json["depth_maps"]` (with `depth_scale` in metres/unit). It is kept **out** of the strict LeRobot `features` dict so the dataset loads on stock LeRobot; wire depth into training from the sidecar files. (16-bit depth cannot be encoded in `mp4v`, hence the sidecar approach.)

**Back-compat:** the converter also reads the older flat layout (`episode_*.json` + `episode_*_<name>.mp4`) and single-camera `*_camera.mp4` episodes.

---

## 8. Control-loop frequencies

| Stage | Rate |
|---|---|
| ServoJ control loop (leader read → map → servo_j → CSV row) | 125 Hz |
| FR5 state readback qualifying cycle | 62.5 Hz |
| ...per property (actual joints / EEF / vel, rotated) | ~20.8 Hz each |
| Cameras (color; scene also depth) | 30 fps |
| Heartbeat print | 1 Hz |
| Exported LeRobot dataset | ~30 Hz (downsampled to wrist_cam frames) |

---

## 9. Troubleshooting

**`scene_cam Could not start (... errno=16 Device or resource busy)`**
Another process holds the camera — usually `realsense-viewer` or a crashed/stuck `teleop.py`. Find and kill the holder:
```bash
fuser /dev/video*            # lists PIDs holding the video nodes
ps -p <PID> -o pid,cmd
kill -9 <PID>
```
A `Ctrl-C` during the FR5 `connect()` step can abort teleop mid-init and leave the cameras held — kill the stuck PID, then the devices free up.

**`ServoJ failed with error 14` then repeated `error 99`**
A joint over-ranged / over-sped. With the current code the loop auto-calls `recover()`; if it persists, lower `JOINT_AMP`, move the leader more gently, or clear the fault by restarting teleop.

**One camera at 480 Mbps**
USB 2.0 — move to a SuperSpeed port and use a USB 3.x cable (see §1).

---

## 10. Files changed / added

| File | Change |
|---|---|
| `camera.py` | `D405Camera` → general `RealSenseCamera(serial, name, enable_depth)`; serial selection; aligned depth; global-time hardware timestamps |
| `config.py` | `D405_SERIAL` / `D435I_SERIAL`, `CAMERAS` list, `CAMERA_USE_HW_TIMESTAMP`, One-Euro params, `FR5_FILTER_T` 0.04→0.06, `JOINT_AMP` reduced |
| `one_euro.py` | **new** — One-Euro filter (`OneEuroFilter`, `JointOneEuro`) |
| `fr5.py` | **new** `recover()` — full clear+enable+restart-servo sequence |
| `teleop.py` | start/register both cameras; apply One-Euro filter; call `recover()` on sustained errors |
| `logger.py` | record multiple named cameras + depth; per-episode folder with sequential index; simplified filenames |
| `convert_to_lerobot.py` | two video keys + scene depth; reference-timeline resampling; per-episode folder discovery; back-compat |
