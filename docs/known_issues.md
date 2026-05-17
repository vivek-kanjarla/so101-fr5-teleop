# Known Issues & Fixes

A running log of bugs found, their root causes, and the fixes applied.

---

## 1. `Robot.RPC.is_connect` / `is_conect` AttributeError

**File:** `fr5.py`

**Symptom:**
```
AttributeError: type object 'RPC' has no attribute 'is_connect'. Did you mean: 'is_conect'?
```
or the inverse (`is_conect` not found, suggests `is_connect`), depending on order of operations.

**Root cause:**
The Fairino SDK sets `Robot.RPC.is_conect` (one 'n') dynamically on the class — it doesn't exist until after the first successful CNDE handshake. If `connect()` is called before CNDE has ever run in the same process, the attribute is absent and any access throws `AttributeError`. Python's fuzzy-match suggestion is misleading — it points to the spelling that existed at suggestion time, not a guaranteed valid attribute.

**Fix:**
```python
try:
    if not Robot.RPC.is_conect:
        Robot.RPC.is_conect = True
except AttributeError:
    pass  # not yet set by SDK; safe to ignore
```

---

## 2. SO-101 Leader Arm Stiff / Positions Not Updating

**File:** `so101.py`

**Symptom:**
Moving the SO-101 arm physically has no effect — joints resist movement and encoder readings stay frozen. The teleop loop sees zero delta and sends no motion commands to the FR5.

**Root cause:**
Feetech STS3215 servos boot with torque **enabled** (position-control mode). They actively fight any external force to hold their last commanded position. `SO101Reader.open()` never disabled torque, so the arm was always stiff regardless of the teleop state.

**Fix:**
Write `0` to `ADDR_TORQUE_ENABLE` (register 40) for all motors immediately after opening the port:
```python
for mid in self._motor_ids + [SO101_GRIPPER_ID]:
    self._packet.write1ByteTxRx(self._port, mid, ADDR_TORQUE_ENABLE, 0)
```
After this, the arm moves freely and encoder positions track naturally.

---

## 3. SO-101 Encoder Wraparound (Large Position Values)

**File:** `so101.py`

**Symptom:**
Occasional readings like `2880°`, `2891°` for a joint that should be near `0°`. These appear when a motor is moved past its zero crossing in one direction.

**Root cause:**
`read2ByteTxRx` returns an unsigned 16-bit integer. When the STS3215 encoder wraps past zero going negative, the raw value becomes something like `32768` (0x8000), which converts to `32768/4096 × 360 ≈ 2880°`. The fix is to interpret the raw value as signed:
```python
if raw > 32767:
    raw -= 65536
```
**Status:** Not yet fixed. Workaround: keep joints away from the mechanical zero crossing during teleoperation.

---

## 4. FR5 Appears Not Moving (Rate Limiter Too Conservative)

**Config:** `config.py`

**Symptom:**
The FR5 moves but very slowly — the operator may think it's not responding at all.

**Root cause:**
`MAX_DELTA_PER_JOINT = [0.08, 0.06, 0.06, 0.20, 0.04, 0.10]` at 125 Hz gives max speeds of 10°/s, 7.5°/s, 7.5°/s, 25°/s, 5°/s, 12.5°/s. For a shoulder joint with typical workspace demands of 60–90°, reaching a target takes 6–12 seconds. Operators perceive this as "no response".

**Fix:**
Increase per-joint limits. Suggested starting values:
```python
MAX_DELTA_PER_JOINT = [0.16, 0.12, 0.12, 0.30, 0.08, 0.20]
#                      J1    J2    J3    J4    J5   J6
#                      20/s  15/s  15/s  37/s  10/s 25°/s
```
Also consider raising `FR5_SERVO_VEL` from `2` to `5` for more responsive motion.

---

## 5. D405 Camera USB State Corruption

**File:** `camera.py`, `camera_view.py`

**Symptom:**
After killing a Python process that held an active RealSense pipeline (e.g., `Ctrl+C` mid-stream, `pkill`), the next `pipeline.start()` raises:
```
RuntimeError: xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```
or the camera disappears entirely from `lsusb`.

**Root cause:**
The RealSense D405 uses a proprietary UVC control protocol. If the pipeline is not cleanly stopped via `pipeline.stop()`, the kernel's UVC driver leaves the USB device in a streaming state. The camera may need to re-enumerate (physical replug) to recover.

**Fixes applied:**
- `camera.py`: `D405Camera.stop()` always calls `pipeline.stop()` via `try/except`.
- `teleop.py`: `_camera_cleanup()` context manager guarantees `camera.stop()` even if the robot connection fails before the main loop starts.

**Recovery:**
If the camera is stuck, unplug and replug the USB-C cable. Use a USB 3.0 port directly on the motherboard (not through a hub) and a data-capable cable.

---

## 6. D405 Dual-Stream Pipeline Instability

**File:** `camera.py`

**Symptom:**
Enabling both color and depth streams simultaneously causes `wait_for_frames()` to time out consistently even with 2000 ms timeout and 15-frame warmup, resulting in 0 frames captured.

**Root cause:**
Unknown — single-stream (color-only) works reliably at the same resolution and FPS on the same USB port. Possibly related to USB bandwidth negotiation or CNDE/UVC driver contention when both streams are open in the same pipeline. `camera_view.py` (which also uses dual stream) was unaffected, suggesting the issue may be specific to the interplay of rapid pipeline start/stop cycles within a session.

**Fix:**
Reverted to color-only pipeline. Depth calibration (intrinsics, extrinsics, scale, stereo baseline) was removed from the saved metadata since depth frames are not recorded. Color intrinsics including distortion model name (`inverse_brown_conrady`) are still saved.

---

## 7. D405 Color Distortion Model Not Saved

**File:** `camera.py`

**Symptom:**
The saved `camera_intrinsics` in episode JSON contained `dist_coeffs` but not the distortion model name. Using these coefficients with `cv2.undistort()` directly would silently apply the wrong Brown-Conrady formula.

**Root cause:**
The D405 color stream uses `inverse_brown_conrady` (coefficients applied in the inverse direction). OpenCV's `cv2.undistort()` assumes standard Brown-Conrady. Without knowing the model name, callers cannot apply the correct undistortion.

**Fix:**
Added `"distortion_model": str(ci.model).split(".")[-1]` to the saved intrinsics dict. Value will be `"inverse_brown_conrady"` for D405 color.
