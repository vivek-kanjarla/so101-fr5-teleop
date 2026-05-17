"""
config.py — central configuration for SO-101 → FR5 teleoperation.
Edit these values to match your physical setup before running teleop.py.
"""

# ── SO-101 (leader arm) ───────────────────────────────────────────────────────
SO101_PORT     = "/dev/ttyACM0"   # Run: ls /dev/ttyACM* after plugging in USB
SO101_BAUDRATE = 1_000_000        # Fixed for Feetech STS3215

# Motor IDs on the SO-101 bus (1=shoulder_pan ... 5=wrist_roll, 6=gripper)
SO101_MOTORS = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
}
SO101_GRIPPER_ID = 6   # read separately — not part of arm mapping

# ── FR5 (follower cobot) ──────────────────────────────────────────────────────
FR5_IP = "192.168.58.2"           # Default Fairino controller IP

# ── Control loop ──────────────────────────────────────────────────────────────
LOOP_HZ     = 125                 # ServoJ must run between 60–1000 Hz
LOOP_PERIOD = 1.0 / LOOP_HZ      # ~0.008 s between calls

# ── Joint mapping: SO-101 (5 joints) → FR5 (6 joints) ────────────────────────
#
# FR5 J4 (forearm roll) has NO SO-101 counterpart — it is frozen at 0°.
#
# Mapping:
#   SO-101 shoulder_pan  → FR5 J1
#   SO-101 shoulder_lift → FR5 J2
#   SO-101 elbow_flex    → FR5 J3
#   SO-101 wrist_flex    → FR5 J4
#   (nothing)            → FR5 J5  ← FROZEN
#   SO-101 wrist_roll    → FR5 J6
#
FR5_J4_FROZEN_DEG = 0.0

# Scale: +1.0 = same direction, -1.0 = reversed
# Index: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
JOINT_SCALE = [-1.0, 1.0, 1.0, 1.0, 1.0]

# Amplification: gear ratio between SO-101 and FR5.
# 2.0 means 1° of SO-101 movement → 2° of FR5 movement.
# Increase to cover FR5 workspace regions the SO-101 can't physically reach.
# Index: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
JOINT_AMP = [1.5, 2.0, 2.0, 3.0, 1.5]

# ── Safety ────────────────────────────────────────────────────────────────────
# Global fallback — used when per-joint limit not specified
MAX_DELTA_DEG_PER_CYCLE = 0.08    # 0.08 × 125Hz = 10°/s

# Per-joint rate limits (degrees/cycle) for [J1, J2, J3, J4, J5, J6]
# Wrist joints (J4, J6) can move faster — less inertia, less risk
MAX_DELTA_PER_JOINT = [0.16, 0.12, 0.12, 0.30, 0.08, 0.20]
#                       J1     J2     J3     J4    J5    J6
#                       20°/s  15°/s  15°/s  37/s  10/s  25°/s

FR5_SERVO_VEL           = 15      # ServoJ velocity % — start low, tune up
FR5_FILTER_T            = 0.04    # ServoJ trajectory filter (seconds) — smooths between commands

# From GetJointSoftLimitDeg() on this controller, with 5° margin inside each limit.
FR5_JOINT_LIMITS = [
    (-170, 170),   # J1  hardware ±175°
    (-260,  80),   # J2  hardware (-265, 85)
    (-155, 155),   # J3  hardware ±160° — robot parks at 94°, old 55° cap was wrong
    (-260,  80),   # J4  hardware (-265, 85) — frozen in mapper
    (-170, 170),   # J5  hardware ±175°
    (-170, 170),   # J6  hardware ±175°
]

# ── DH AG-160-95 gripper (flange-mounted, via Fairino SDK) ───────────────────
GRIPPER_INDEX     = 1      # gripper number configured on the FR5 controller
GRIPPER_TYPE      = 0      # 0=parallel gripper (AG-160-95), 1=rotating

GRIPPER_OPEN_PCT  = 100    # position % sent when opening  (0–100)
GRIPPER_CLOSE_PCT = 0      # position % sent when closing
GRIPPER_VEL_PCT   = 50     # movement speed    (0–100)
GRIPPER_FORCE_PCT = 50     # grip force        (0–100)
GRIPPER_MAXTIME_MS = 5000  # max travel time before timeout (ms)

# SO-101 gripper motor position normalised to [0,1]:
#   above OPEN_THRESHOLD  → send open
#   below CLOSE_THRESHOLD → send close
#   between               → hold (hysteresis)
# Calibration: range_min=2028, range_max=3236  → degrees = raw/4096*360
SO101_GRIPPER_RANGE        = (2028/4096*360, 3236/4096*360)  # (178.0°, 284.3°)
SO101_GRIPPER_OPEN_THRESHOLD  = 0.65   # norm ≥ 0.65 → open
SO101_GRIPPER_CLOSE_THRESHOLD = 0.35   # norm ≤ 0.35 → close

# ── Meta Quest 3 VR controller ───────────────────────────────────────────────
#
# Two transport modes:
#   "oculus_reader" — Oculus Reader APK (rail-berkeley/oculus_reader) via ADB
#                     Run: adb forward tcp:5555 tcp:5555
#   "udp"           — lightweight UDP packet streamer (custom app / HTS-compatible)
#
QUEST3_MODE       = "oculus_reader"  # "oculus_reader" | "udp"
QUEST3_ADB_PORT   = 5555             # TCP port after adb forward (oculus_reader mode)
QUEST3_UDP_PORT   = 5005             # UDP port this PC listens on (udp mode)
QUEST3_ACTIVE_HAND = "right"         # which controller drives the arm: "right" | "left"

# Position scale: 1 meter of controller motion → this many mm of EEF motion.
# Start conservative (300) and increase once the motion feels right.
VR_POSITION_SCALE = 300.0    # mm / m

# Rotation scale: multiplier on controller rotation → EEF rotation (deg).
# 1.0 = 1:1 mapping. Reduce if wrist snaps feel aggressive.
VR_ROTATION_SCALE = 0.8

# Per-cycle safety clamps applied BEFORE IK
VR_MAX_DELTA_POS_MM  = 8.0    # max EEF translation per cycle (mm)
VR_MAX_DELTA_ROT_DEG = 3.0    # max EEF rotation per cycle (deg, per axis)

# Coordinate frame rotation: Quest 3 OpenXR convention → FR5 TCP frame.
# Each entry is (axis, angle_deg) applied in order — default maps
# Quest (X=right, Y=up, Z=back) → FR5 TCP (X=fwd, Y=left, Z=up).
# Tune this if EEF motion axes feel wrong on your setup.
# Format: list of [source_axis_index, target_axis_index, sign]
# Stored as a 3×3 matrix (row = FR5 axis, col = Quest axis):
#   FR5_X ← -Quest_Z  (Quest backward  → FR5 forward inverted)
#   FR5_Y ←  Quest_X  (Quest right     → FR5 right)
#   FR5_Z ←  Quest_Y  (Quest up        → FR5 up)
VR_FRAME_ROTATION = [
    [ 0.0, 0.0, -1.0],   # FR5 X
    [ 1.0, 0.0,  0.0],   # FR5 Y
    [ 0.0, 1.0,  0.0],   # FR5 Z
]

# Gripper control via Quest trigger (analog, 0=released → 1=fully pulled)
VR_GRIPPER_OPEN_THRESHOLD  = 0.2   # trigger below this → open
VR_GRIPPER_CLOSE_THRESHOLD = 0.7   # trigger above this → close

# ── D405 RealSense wrist camera ───────────────────────────────────────────────
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30    # 30 / 60 / 90 supported by D405; 30 is standard for training data

# ── Data logging ──────────────────────────────────────────────────────────────
LOG_DIR = "./episodes"

# Write the task description here before pressing R to start recording.
# One line, plain text. Stored in the episode metadata JSON alongside the CSV.
# Example: "pick up the red block and place it in the bin"
INSTRUCTION_FILE = "./episode_instruction.txt"

# State reads are staggered: each qualifying cycle reads ONE of {joint positions,
# EEF pose, joint velocities} in rotation. LOG_STATE_DOWNSAMPLE controls how
# often a qualifying cycle occurs — e.g., N=2 means one read every 2 cycles,
# cycling through all three properties every 6 cycles (~21 Hz per property).
# This caps per-cycle overhead at one RPC call (~3 ms) and keeps the 8 ms
# ServoJ budget intact. Every CSV row still gets complete data via caching.
LOG_STATE_DOWNSAMPLE = 2
