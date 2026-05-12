# SO-101 → FR5 Teleoperation

Real-time teleoperation bridge that streams joint positions from a **SO-101 leader arm** to a **Fairino FR5 cobot** at 125 Hz. The SO-101 (5-DOF, Feetech STS3215 servos) acts as a handheld controller; the FR5 (6-DOF industrial cobot) mirrors its motion with per-joint rate limiting, singularity protection, and gripper passthrough.

---

## Hardware

| Component | Details |
|---|---|
| SO-101 leader arm | 5 joints + gripper, Feetech STS3215 servos, USB serial |
| Fairino FR5 cobot | 6-DOF industrial robot, Ethernet (default `192.168.58.2`) |
| DH AG-160-95 gripper | Parallel gripper, flange-mounted, controlled via Fairino SDK |

---

## Dependencies

```bash
pip install scservo-sdk fairino pynput numpy pandas pyserial
```

> The `fairino` package is Fairino's official Python SDK. Install it from their distribution or PyPI if available for your controller firmware version.

---

## Setup

### 1. Connect hardware

- Plug the SO-101 into USB — it will appear as `/dev/ttyACM0` (Linux) or `COMx` (Windows)
- Connect the FR5 controller via Ethernet and confirm you're on the same subnet (`192.168.58.x`)

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

Tune `JOINT_SCALE`, `JOINT_AMP`, joint limits, and gripper thresholds as needed for your physical setup.

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

---

## Joint Mapping

SO-101 has 5 arm joints; FR5 has 6. The forearm roll (J5) has no SO-101 counterpart and is frozen at its home value.

| SO-101 | FR5 |
|---|---|
| shoulder_pan | J1 |
| shoulder_lift | J2 |
| elbow_flex | J3 |
| wrist_flex | J4 |
| *(none)* | J5 — frozen |
| wrist_roll | J6 |

Mapping is **delta-based**: only the change from SO-101's home pose is applied to FR5's home pose. Direction and amplitude per joint are configurable via `JOINT_SCALE` and `JOINT_AMP` in `config.py`.

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

Hard limits from `FR5_JOINT_LIMITS` (pulled from `GetJointSoftLimitDeg()` with a 5° margin) are clamped before the rate limiter, so the robot never receives an out-of-range command.

### Gripper

`MoveGripper` cannot be called while `ServoMoveStart` is active. The gripper thread signals the main loop, which pauses ServoJ (~200 ms), sends the gripper command, then resumes. Gripper state changes use hysteresis thresholds to prevent rapid toggling.

---

## Data Logging

Press `R` to start/stop recording. Episodes are saved to `./episodes/` as CSV files:

```
timestamp | so101_shoulder_pan | so101_shoulder_lift | ... | fr5_j1 | fr5_j2 | ... | fr5_j6
```

An in-progress recording is auto-saved on Ctrl-C or emergency stop.

---

## Configuration Reference

All parameters live in `config.py`:

| Parameter | Description |
|---|---|
| `LOOP_HZ` | Control loop frequency (default 125 Hz) |
| `JOINT_SCALE` | Per-joint direction flip (+1 or −1) |
| `JOINT_AMP` | Per-joint amplitude multiplier (gear ratio SO-101 → FR5) |
| `MAX_DELTA_PER_JOINT` | Per-joint rate limit in degrees/cycle |
| `FR5_SERVO_VEL` | ServoJ velocity % — start low, tune up |
| `FR5_FILTER_T` | ServoJ trajectory filter (seconds) — smooths commands |
| `FR5_JOINT_LIMITS` | Min/max degrees per joint |
| `SO101_GRIPPER_RANGE` | Raw degree range of the SO-101 gripper motor |
| `SO101_GRIPPER_OPEN_THRESHOLD` | Normalised position above which gripper opens |
| `SO101_GRIPPER_CLOSE_THRESHOLD` | Normalised position below which gripper closes |
| `LOG_DIR` | Directory for episode CSV files |

---

## Project Structure

```
teleop.py          — main teleoperation loop
so101.py           — SO-101 serial reader (STS3215 servos)
fr5.py             — FR5 controller (Fairino SDK, ServoJ mode)
mapper.py          — delta joint mapping SO-101 → FR5
gripper.py         — DH AG-160-95 gripper controller
singularity.py     — singularity detection and speed scaling
logger.py          — episode recording to CSV
config.py          — all configuration parameters
check_hardware.py  — hardware connectivity diagnostic
check_network.py   — FR5 network diagnostic
scan_motor_ids.py  — scan servo bus for all motor IDs
probe_one_motor.py — identify motors one at a time
```
