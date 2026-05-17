# Meta Quest 3 VR Teleoperation — Setup Guide

Real-time end-effector control of the FR5 using the Meta Quest 3 right controller.
The controller's 6-DOF pose (position + orientation delta) maps to FR5 TCP motion
via inverse kinematics. The trigger analog drives the gripper.

---

## Hardware required

| Item | Notes |
|---|---|
| Meta Quest 3 | Developer mode enabled (required for sideloading) |
| USB-C cable | For ADB / sideloading — cable must support data |
| 5 GHz WiFi router | Dedicated or low-traffic network; 5 GHz is required for latency |
| PC running teleop_vr.py | Same subnet as Quest |

---

## Step 1 — Enable Developer Mode on Quest 3

1. Open the **Meta Quest mobile app** on your phone.
2. Go to **Menu → Devices → [your Quest 3] → Developer Mode → On**.
3. Put on the headset and accept the prompt to confirm.

---

## Step 2 — Install ADB (Android Debug Bridge)

macOS:
```bash
brew install android-platform-tools
```

Ubuntu/Debian:
```bash
sudo apt install adb
```

Windows: Download [Android SDK Platform Tools](https://developer.android.com/tools/releases/platform-tools).

Verify:
```bash
adb version
```

---

## Step 3 — Sideload the Oculus Reader APK

The [Oculus Reader APK](https://github.com/rail-berkeley/oculus_reader) streams
controller pose over a TCP socket.

```bash
# Clone the repo to get the APK
git clone https://github.com/rail-berkeley/oculus_reader
cd oculus_reader

# Connect Quest 3 via USB-C, accept the "Allow USB debugging" prompt in headset
adb devices          # should list your Quest 3

# Install the APK
adb install reader.apk
```

---

## Step 4 — Forward the port

The APK runs a TCP server on the Quest. Forward it to your PC:

```bash
adb forward tcp:5555 tcp:5555
```

> Re-run this command every time you reconnect the Quest via USB or restart ADB.
> You can disconnect the USB cable after forwarding — the tunnel persists briefly,
> but for reliable operation keep the cable connected or switch to UDP mode (see below).

---

## Step 5 — Launch the Oculus Reader app on Quest 3

In the headset, open the **App Library → Unknown Sources → OculusReader**.
The app shows a small overlay and starts streaming immediately.
There is no visible UI — it runs in the background.

---

## Step 6 — Verify the stream (optional)

```python
# Quick sanity check — run from the project directory
python - <<'EOF'
from quest3 import Quest3Reader
with Quest3Reader() as q:
    for _ in range(20):
        import time; time.sleep(0.1)
        s = q.get_state()
        print(f"valid={s.valid}  pos={s.pos}  trig={s.trigger:.2f}")
EOF
```

Move the right controller while watching the output. `pos` should change.

---

## Step 7 — Run VR teleoperation

```bash
python teleop_vr.py
```

On startup the system captures **home poses** — hold the controller still and keep
the FR5 in a safe, open configuration before launching.

### Controls

| Input | Action |
|---|---|
| Move controller | EEF translates (scaled by `VR_POSITION_SCALE`) |
| Rotate controller | EEF rotates (scaled by `VR_ROTATION_SCALE`) |
| Trigger ≥ 0.7 | Gripper close |
| Trigger ≤ 0.2 | Gripper open |
| `H` key | Re-home — reset reference to current poses |
| `R` key | Toggle episode recording |
| `Space` | Emergency stop |
| `Ctrl-C` | Graceful exit |

### Heartbeat output

```
[HB] cyc=125  sing=CLEAR  Quest moved  12.3mm  →  FR5 cmd moved  18.5°  trig=0.00  errs=0
```

- `Quest moved 0 mm` → controller stream frozen or invalid
- `Quest moved N mm` but `FR5 cmd moved 0°` → IK failing or singularity blocked motion
- `errs=N` rising fast → communication issue

---

## Coordinate frame calibration

The default `VR_FRAME_ROTATION` matrix in `config.py` maps:

| Quest 3 axis | FR5 TCP axis |
|---|---|
| X (right)    | Y (right) |
| Y (up)       | Z (up) |
| Z (back)     | −X (forward inverted) |

If EEF motion axes are wrong (e.g., moving forward moves EEF sideways), adjust
`VR_FRAME_ROTATION` in `config.py`. It is a 3×3 matrix where each row is one FR5
axis expressed in Quest coordinates. Use only ±1 and 0 entries (permutation + sign).

---

## Tuning

| Parameter | Default | Effect |
|---|---|---|
| `VR_POSITION_SCALE` | 300 | mm of EEF motion per metre of controller motion. Increase for larger workspace coverage. |
| `VR_ROTATION_SCALE` | 0.8 | EEF rotation gain. 1.0 = 1:1 with controller rotation. |
| `VR_MAX_DELTA_POS_MM` | 8.0 | Hard clamp per cycle — prevents lurches from tracking jumps. |
| `VR_MAX_DELTA_ROT_DEG` | 3.0 | Hard clamp per cycle on rotation. |
| `VR_GRIPPER_CLOSE_THRESHOLD` | 0.7 | Trigger level to close gripper. |
| `VR_GRIPPER_OPEN_THRESHOLD` | 0.2 | Trigger level to open gripper. |

Start with `VR_POSITION_SCALE = 200` (conservative) and increase once motion feels stable.

---

## Alternative: UDP mode

If you prefer to use a custom Quest app or Hand Tracking Streamer instead of Oculus
Reader, set `QUEST3_MODE = "udp"` in `config.py`.

Your Quest app must send UDP packets to this machine on `QUEST3_UDP_PORT` (default 5005)
using the binary format documented in `quest3.py:_UDPTransport`:

```
Offset  Type      Field
0       uint32    magic = 0x51455354
4       float64   timestamp (seconds)
12      float32   pos_x (metres)
16      float32   pos_y
20      float32   pos_z
24      float32   quat_x
28      float32   quat_y
32      float32   quat_z
36      float32   quat_w
40      float32   trigger [0, 1]
44      float32   grip [0, 1]
48      uint32    buttons bitmask (bit 0=A/X, 1=B/Y, 2=menu, 3=thumbstick)
```

Total: 52 bytes, little-endian.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `TimeoutError: No data received within 5 s` | ADB forward not set up or app not running | Re-run `adb forward tcp:5555 tcp:5555`, relaunch app |
| `Quest moved 0 mm` in heartbeat | Quest tracking lost or controller sleeping | Wake controller, check battery |
| IK errors / `GetInverseKin failed` | Target EEF pose unreachable | Reduce `VR_POSITION_SCALE`; press `H` to re-home |
| Axes inverted | Coordinate frame mismatch | Adjust `VR_FRAME_ROTATION` in config.py |
| Gripper not responding | Trigger thresholds wrong | Adjust `VR_GRIPPER_*_THRESHOLD` |
| High latency / jerky motion | WiFi congestion or 2.4 GHz band | Switch Quest to 5 GHz band, reduce background traffic |
