# SO-101 → FR5 Teleoperation

Real-time teleoperation bridge that streams joint positions from a **SO-101 leader arm** to a **Fairino FR5 cobot** at 125 Hz. The SO-101 (5-DOF, Feetech STS3215 servos) acts as a handheld controller; the FR5 (6-DOF industrial cobot) mirrors its motion with per-joint rate limiting, singularity protection, and gripper passthrough. Episodes are recorded as CSV + JSON + MP4 bundles ready for imitation-learning training.

---

## Hardware

| Component | Details |
|---|---|
| SO-101 leader arm | 5 joints + gripper, Feetech STS3215 servos, USB serial (RS-485) |
| Fairino FR5 cobot | 6-DOF industrial robot, Ethernet (default `192.168.58.2`) |
| DH AG-160-95 gripper | Parallel gripper, flange-mounted, controlled via Fairino SDK |
| Intel RealSense D405 | Wrist-mounted RGB camera, 640×480 @ 30 fps, factory intrinsics |

---

## Dependencies

```bash
pip install scservo-sdk fairino pynput numpy pandas pyserial pyrealsense2 opencv-python
```

> The `fairino` package is Fairino's official Python SDK. Install it from their distribution or PyPI for your controller firmware version.

---

## Setup

### 1. Connect hardware

- Plug the SO-101 into USB — it appears as `/dev/ttyACM0` (Linux) or `COMx` (Windows)
- Connect the FR5 controller via Ethernet and confirm you're on the same subnet (`192.168.58.x`)
- Mount the D405 on the FR5 wrist and plug into a USB 3.0 port directly on the motherboard (no hub)

### 2. Verify connectivity

```bash
python check_network.py     # ping + TCP check to FR5
python check_hardware.py    # SO-101 serial + FR5 joint read
```

### 3. Identify motor IDs (first-time only)

If you're unsure which servo ID maps to which joint:

```bash
# Scan all motors on the bus at once
python scan_motor_ids.py

# Or probe one motor at a time (plug in, identify, unplug, repeat)
python probe_one_motor.py
```

Update `SO101_MOTORS` in `config.py` with the result.

### 4. Edit `config.py`

```python
SO101_PORT = "/dev/ttyACM0"   # your USB port
FR5_IP     = "192.168.58.2"   # your FR5 controller IP
```

See [Configuration Reference](#configuration-reference) for all parameters.

---

## Running

```bash
python teleop.py
```

The system captures home poses on startup — both arms must be in a safe, reachable configuration before launching. The first command is always Δ=0, so there is no startup lurch regardless of where either arm is parked.

### Controls

| Key | Action |
|---|---|
| `Space` | Emergency stop — kills ServoJ and exits |
| `R` | Toggle episode recording on/off |
| `H` | Re-home — resets the reference pose to current positions |
| `Ctrl-C` | Graceful exit |

### Heartbeat output

Every second the loop prints a heartbeat line:

```
[HB] cyc=125  sing=CLEAR  SO101 moved  12.3°  →  FR5 cmd moved  18.5°  errs=0
```

- `SO101 moved 0°` with `FR5 cmd moved 0°` means everything is frozen (home pose held)
- `SO101 moved N°` with `FR5 cmd moved 0°` means the leader reads are working but the mapper or ServoJ is broken
- `errs=N` counts transient read/write errors — a few per session is normal; rising fast is not

---

## Joint Mapping

SO-101 has 5 arm joints; FR5 has 6. The forearm roll (J5) has no SO-101 counterpart and is frozen at its home value.

| SO-101 joint | FR5 joint | Scale | Amp |
|---|---|---|---|
| shoulder_pan | J1 | −1.0 | 1.5× |
| shoulder_lift | J2 | +1.0 | 2.0× |
| elbow_flex | J3 | +1.0 | 2.0× |
| wrist_flex | J4 | +1.0 | 3.0× |
| *(none)* | J5 — frozen | — | — |
| wrist_roll | J6 | +1.0 | 1.5× |

Mapping is **delta-based**: only the change from SO-101's home pose is applied to FR5's home pose. Direction (`JOINT_SCALE`) and amplitude (`JOINT_AMP`) per joint are configurable in `config.py`.

> **Key implementation note:** The mapper returns plain Python `float` values. `xmlrpc.client` (which backs the Fairino SDK) cannot serialize `numpy.float64` — every ServoJ call would raise a silent `TypeError`. Both `mapper.py` and `fr5.py` explicitly coerce values at the XML-RPC boundary with `float()`.

---

## Safety

### Rate limiting

Each FR5 joint has an independent maximum movement per control cycle (`MAX_DELTA_PER_JOINT`), preventing sudden jumps from noisy reads or large amplitude changes.

### Singularity detection

`singularity.py` monitors J3 (elbow) and J4 (wrist) proximity to singular configurations:

| Zone | Behavior |
|---|---|
| **WARN** | Speed linearly scaled down toward zero |
| **DANGER** | Motion blocked — robot holds current position |

Press `H` to re-home after recovering from a singularity.

### Joint limits

Hard limits from `FR5_JOINT_LIMITS` (from `GetJointSoftLimitDeg()` with a 5° margin) are clamped before the rate limiter — the robot never receives an out-of-range command.

### Gripper

`MoveGripper` cannot be called while `ServoMoveStart` is active. The gripper thread signals the main loop, which pauses ServoJ (~200 ms), sends the gripper command, then resumes. Gripper state changes use hysteresis thresholds to prevent rapid toggling.

---

## Data Recording

See **[docs/data_recording_guide.md](docs/data_recording_guide.md)** for the full step-by-step workflow.

### Quick reference

1. Write the task description to `episode_instruction.txt` (one line of plain text)
2. Run `python teleop.py` and position the arms
3. Press `R` to start recording, perform the task, press `R` again to stop
4. Files are saved to `./episodes/`

### Output files per episode

| File | Contents |
|---|---|
| `episode_{id}.csv` | 125 Hz joint data — all columns below |
| `episode_{id}.json` | Metadata: instruction, duration, camera intrinsics |
| `episode_{id}_camera.mp4` | RGB video at 30 fps (if D405 attached) |
| `episode_{id}_camera_ts.npy` | Per-frame wall-clock timestamps (float64) |

### CSV columns

```
timestamp
so101_shoulder_pan | so101_shoulder_lift | so101_elbow_flex | so101_wrist_flex | so101_wrist_roll
fr5_cmd_j1  .. fr5_cmd_j6       ← commanded joint positions (deg)
fr5_actual_j1 .. fr5_actual_j6  ← measured joint positions (deg)
fr5_eef_x_mm  fr5_eef_y_mm  fr5_eef_z_mm         ← TCP position
fr5_eef_rx_deg  fr5_eef_ry_deg  fr5_eef_rz_deg    ← TCP Euler angles
gripper_norm                     ← SO-101 gripper [0.0=closed, 1.0=open]
fr5_vel_j1 .. fr5_vel_j6        ← actual joint velocities (deg/s)
```

Actual FR5 state is read every 2 ServoJ cycles (62.5 Hz) and cached — every CSV row has complete data.

---

## Camera

The D405 runs a **color-only** pipeline (depth is not recorded) at 640×480 @ 30 fps. Factory-calibrated intrinsics are saved in each episode's JSON:

```json
"camera_intrinsics": {
  "fx": 615.3, "fy": 615.2, "cx": 320.1, "cy": 240.0,
  "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
  "width": 640, "height": 480,
  "distortion_model": "inverse_brown_conrady"
}
```

> The D405 uses **inverse Brown-Conrady** distortion. Standard `cv2.undistort()` applies forward Brown-Conrady — use the model name to select the correct undistortion path at training time.

---

## Diagnostics

If something is not working, run the targeted diagnostics before changing any code:

```bash
# Confirm SO-101 reads are updating when you move the arm
python so101_read_test.py

# Confirm the FR5 can accept and execute joint commands
python fr5_motion_test.py
```

- **SO-101 reads frozen** → torque disable failed (check USB port, servo IDs in config)
- **FR5 doesn't move in fr5_motion_test** → Ethernet, wrong IP, CNDE not running
- **SO-101 reads fine, FR5 moves in isolation, but teleop doesn't work** → mapper or ServoJ issue; check heartbeat for `SO101 moved N° → FR5 cmd moved 0°`

---

## Configuration Reference

All parameters live in `config.py`. See also **[docs/tuning_guide.md](docs/tuning_guide.md)** for how to tune for smoother / faster motion.

### Hardware

| Parameter | Default | Description |
|---|---|---|
| `SO101_PORT` | `/dev/ttyACM0` | USB serial port for SO-101 |
| `SO101_BAUDRATE` | 1,000,000 | Fixed for Feetech STS3215 |
| `FR5_IP` | `192.168.58.2` | Fairino FR5 controller IP |

### Control loop

| Parameter | Default | Description |
|---|---|---|
| `LOOP_HZ` | 125 | ServoJ frequency (must be 60–1000 Hz) |
| `LOOP_PERIOD` | 1/125 s | Derived from LOOP_HZ |

### Joint mapping

| Parameter | Default | Description |
|---|---|---|
| `JOINT_SCALE` | `[-1, 1, 1, 1, 1]` | Per-joint direction flip (+1 or −1) |
| `JOINT_AMP` | `[1.5, 2.0, 2.0, 3.0, 1.5]` | Per-joint amplitude multiplier |

### Motion quality

| Parameter | Default | Description |
|---|---|---|
| `MAX_DELTA_PER_JOINT` | `[0.16, 0.12, 0.12, 0.30, 0.08, 0.20]` | Max degrees/cycle per joint |
| `MAX_DELTA_DEG_PER_CYCLE` | 0.08 | Global fallback rate limit |
| `FR5_SERVO_VEL` | 15 | ServoJ velocity % (0–100) |
| `FR5_FILTER_T` | 0.04 | ServoJ trajectory filter in seconds |

### Safety

| Parameter | Default | Description |
|---|---|---|
| `FR5_JOINT_LIMITS` | (see config) | Min/max degrees per joint, 5° inside hardware limit |

### Gripper

| Parameter | Default | Description |
|---|---|---|
| `SO101_GRIPPER_OPEN_THRESHOLD` | 0.65 | Normalised position → open |
| `SO101_GRIPPER_CLOSE_THRESHOLD` | 0.35 | Normalised position → close |
| `GRIPPER_VEL_PCT` | 50 | Gripper speed (0–100) |
| `GRIPPER_FORCE_PCT` | 50 | Grip force (0–100) |

### Camera

| Parameter | Default | Description |
|---|---|---|
| `CAMERA_WIDTH` | 640 | Frame width in pixels |
| `CAMERA_HEIGHT` | 480 | Frame height in pixels |
| `CAMERA_FPS` | 30 | Capture frame rate |

### Logging

| Parameter | Default | Description |
|---|---|---|
| `LOG_DIR` | `./episodes` | Episode output directory |
| `INSTRUCTION_FILE` | `./episode_instruction.txt` | Task description file |
| `LOG_STATE_DOWNSAMPLE` | 2 | Read actual FR5 state every N cycles |

---

## Project Structure

```
teleop.py              — main teleoperation loop (125 Hz ServoJ)
so101.py               — SO-101 serial reader (STS3215 servos via RS-485)
fr5.py                 — FR5 controller (Fairino SDK, ServoJ mode)
mapper.py              — delta joint mapping SO-101 → FR5
gripper.py             — DH AG-160-95 gripper controller (shares FR5 connection)
singularity.py         — singularity detection and speed scaling
logger.py              — episode recording: CSV + JSON + MP4 + timestamps
camera.py              — Intel RealSense D405 capture (color-only background thread)
config.py              — all configuration parameters

check_hardware.py      — hardware connectivity diagnostic
check_network.py       — FR5 network diagnostic
scan_motor_ids.py      — scan servo bus for all motor IDs
probe_one_motor.py     — identify motors one at a time
so101_read_test.py     — isolated SO-101 leader read diagnostic
fr5_motion_test.py     — isolated FR5 ServoJ motion diagnostic

docs/
  mapper_explained.md      — full walkthrough of delta-mapping logic
  fr5_sdk_guide.md         — Fairino SDK internals: XML-RPC, ServoJ, RPC lock
  data_recording_guide.md  — step-by-step episode recording workflow
  tuning_guide.md          — how to tune parameters for smoother motion
  known_issues.md          — bug log with root causes and fixes
```
