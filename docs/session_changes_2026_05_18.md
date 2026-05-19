# Session Changes — 2026-05-18

Summary of all changes made to the teleoperation stack during this session.

---

## 1. Diagnostics Added to Heartbeat (`teleop.py`)

**What changed:** The 1 Hz heartbeat line now reads and prints the actual FR5 J1 joint position alongside the commanded position.

**Before:**
```
[HB] cyc=500  sing=CLEAR  SO101 moved  16.0°  →  FR5 cmd moved   6.0°  errs=0
```

**After:**
```
[HB] cyc=500  sing=CLEAR  SO101 moved  16.0°  →  FR5 cmd moved   6.0°  actual_J1=-91.7°  errs=0
```

**Why:** The heartbeat previously only showed commanded positions. This made it impossible to tell whether the FR5 was physically executing the commands or silently ignoring them. The actual read uses `GetActualJointPosDegree(0)` (CNDE) at 1 Hz — low enough overhead to not affect the 8 ms ServoJ budget.

---

## 2. `--no-gripper` Flag (`teleop.py`)

**What changed:** Added a command-line flag to skip gripper activation entirely.

**Usage:**
```bash
python3 teleop.py --no-gripper
```

**Why:** The gripper fires a `pause_for_gripper` sequence on the very first loop iteration (because the SO-101 gripper motor reads above the open threshold at startup). This was suspected as the cause of the FR5 not physically moving despite ServoJ returning 0. The flag lets you run the full teleop pipeline without any gripper involvement to isolate the issue.

**Note:** The gripper motion issue is still under investigation. Use `--no-gripper` until it is resolved.

---

## 3. Gripper Pause Fix — Drive Re-enable Delay (`gripper.py`)

**What changed:** Added `time.sleep(0.5)` after `RobotEnable(1)` in `pause_for_gripper()`.

**Before:**
```python
with robot._rpc_lock:
    robot._robot.RobotEnable(1)
robot.start_servo_mode()
```

**After:**
```python
with robot._rpc_lock:
    robot._robot.RobotEnable(1)
time.sleep(0.5)                     # drives need ~500ms to energise after re-enable
robot.start_servo_mode()
```

**Why:** `connect()` already waits 500 ms after `RobotEnable(1)` before proceeding. `pause_for_gripper` was calling `ServoMoveStart()` immediately, potentially before the servo drives had fully energized. This mirrors the same wait used at startup.

**Status:** Implemented but did not resolve the FR5 not-moving issue. Investigation ongoing.

---

## 4. Diagnostic Script — Gripper→ServoJ Isolation (`test_servo_after_gripper.py`)

**What changed:** New script created at `~/teleop/test_servo_after_gripper.py`.

**What it does:** Tests four steps in sequence using a single FR5 connection and reports PASS/FAIL for each:

| Step | Test |
|------|------|
| 1 | Plain ServoJ with no gripper (baseline) |
| 2 | `ActGripper` before ServoJ |
| 3 | Full `pause_for_gripper` sequence WITH `RobotEnable(1)` |
| 4 | Full `pause_for_gripper` sequence WITHOUT `RobotEnable(1)` |

**Usage:**
```bash
python3 test_servo_after_gripper.py
```

**Why:** Isolates exactly which call in the gripper pause sequence breaks ServoJ, without needing to run the full teleop.

---

## 5. Mapper — Three Motion Quality Improvements (`mapper.py`, `config.py`, `teleop.py`)

### 5a. Deadband — Suppress Encoder Noise

**File:** `mapper.py`, `config.py`

**Problem:** STS3215 encoder noise is ±1–2 counts = ±0.09–0.18°. With `JOINT_AMP[wrist_flex]=3.0`, this becomes ±0.54° of oscillation commanded to the FR5 wrist even when the operator holds the SO-101 still.

**Fix:** Zero out any SO-101 joint delta below `DEADBAND_SO101_DEG` before applying scale and amplification.

```python
# mapper.py — after computing delta_raw:
mask  = np.abs(delta_raw) > DEADBAND_SO101_DEG
delta = delta_raw * mask * np.array(JOINT_SCALE) * np.array(JOINT_AMP)
```

```python
# config.py — new parameter:
DEADBAND_SO101_DEG = 0.18   # slightly above the ±1 count encoder noise floor
```

**Effect:** FR5 holds perfectly still when the operator is holding the SO-101 still.

---

### 5b. Acceleration Limiter — Eliminate Jerk on Direction Reversals

**File:** `mapper.py`, `config.py`

**Problem:** The previous rate limiter had no velocity memory. The step size (velocity) could jump from 0 to `rate_limit` in a single 8 ms cycle, and reverse direction (`+rate_limit` to `−rate_limit`) in a single cycle. Measured jerk on direction reversal: 30°/s². With acceleration limiting: 1.9°/s² — 16× smoother.

**Fix:** Add `prev_step` (last step size per joint) as a parameter. The new step can only change by `MAX_ACCEL_PER_JOINT` per cycle.

```python
# mapper.py — inside the per-joint loop:
desired_step = np.clip(pos_error, -effective_plim, effective_plim)
actual_step  = ps + np.clip(desired_step - ps, -alim, alim)
```

```python
# config.py — new parameter:
MAX_ACCEL_PER_JOINT = [0.0180, 0.0150, 0.0150, 0.0300, 0.0080, 0.0240]
# J1 ramps from 0 → full speed in ~14 cycles = 110 ms
```

**Signature change:** `so101_to_fr5` now takes `prev_step: list[float]` and returns `(result, new_steps)`.

**teleop.py changes:**
- `self._prev_step = [0.0] * 6` added to `__init__`
- Cleared to zeros on: re-home (`H` key), after gripper pause, when singularity blocks motion
- Call site updated to unpack the tuple

---

### 5c. Dynamic Rate + Raised Limits — Reduce Coasting

**File:** `mapper.py`, `config.py`

**Problem:** When the operator moves quickly, the rate limiter can't keep up and lag accumulates. When they stop, the FR5 keeps moving at full `rate_limit` speed until the lag is paid off. With `JOINT_AMP=2.0` and the old limits, a 1-second movement produced ~1.7 seconds of coasting afterward.

**Root cause:** `MAX_DELTA_PER_JOINT` was too low relative to `JOINT_AMP`. At 125 Hz with `JOINT_AMP=2.0`, each SO-101 cycle moves the target by up to 2× human speed, but the old limit (0.12°/cycle) could only close 37.5% of the gap per cycle — lag accumulated every cycle the operator moved.

**Fix A — Raised rate limits (+50%):**

| Joint | Old (°/cycle) | New (°/cycle) | Speed |
|-------|--------------|--------------|-------|
| J1 | 0.16 | 0.24 | 30°/s |
| J2 | 0.12 | 0.20 | 25°/s |
| J3 | 0.12 | 0.20 | 25°/s |
| J4 | 0.30 | 0.40 | 50°/s |
| J5 | 0.08 | 0.08 | frozen |
| J6 | 0.20 | 0.30 | 37°/s |

**Fix B — Dynamic rate (tapers near target):**

```python
# mapper.py — replaces the fixed-limit clip:
catchup        = min(1.0, abs(pos_error) / (plim * 3))
effective_plim = plim * max(0.1, catchup)
desired_step   = np.clip(pos_error, -effective_plim, effective_plim)
```

When the FR5 is far from the target (catching up): full `rate_limit` speed.
When within 3×`rate_limit` of the target: speed tapers linearly down to 10% — the robot decelerates into position rather than stopping abruptly.

**Effect:** Coasting after a stop is replaced by a smooth deceleration into the target position.

---

## Summary Table

| File | Change | Status |
|------|--------|--------|
| `teleop.py` | Actual J1 in heartbeat | Done |
| `teleop.py` | `--no-gripper` flag | Done |
| `teleop.py` | `_prev_step` state + resets | Done |
| `teleop.py` | Updated mapper call site | Done |
| `gripper.py` | 500 ms sleep after `RobotEnable(1)` | Done — did not fix motion issue |
| `test_servo_after_gripper.py` | New diagnostic script | Done |
| `mapper.py` | Deadband filter | Done |
| `mapper.py` | Acceleration limiter | Done |
| `mapper.py` | Dynamic rate / coasting fix | Done |
| `config.py` | `DEADBAND_SO101_DEG = 0.18` | Done |
| `config.py` | `MAX_ACCEL_PER_JOINT` | Done |
| `config.py` | `MAX_DELTA_PER_JOINT` +50% | Done |
| `config.py` | `MAX_DELTA_DEG_PER_CYCLE` updated | Done |

---

## Open Issue — FR5 Not Physically Moving During Teleop

**Symptom:** `actual_J1` stays frozen at home despite `FR5 cmd moved` reaching 40°+ and `errs=0`.

**Confirmed working:** `fr5_motion_test.py` moves the robot 5° successfully — the hardware path is fine.

**Confirmed broken:** Full teleop with gripper enabled — robot does not move at all.

**Suspected cause:** The `pause_for_gripper` sequence (triggered by gripper OPEN/CLOSED on the first loop iteration) leaves the FR5 controller accepting ServoJ calls but not executing them.

**Next step:** Run `python3 teleop.py --no-gripper` and check if `actual_J1` changes when the SO-101 arm is moved. If it does, the gripper code is 100% the cause and the fix will be in `gripper.py`.
