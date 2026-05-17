"""
config.py — configuration for Meta Quest 3 → FR5 VR teleoperation.
Edit these values to match your physical setup before running teleop_vr.py.
"""

# ── FR5 (follower cobot) ──────────────────────────────────────────────────────
FR5_IP = "192.168.58.2"           # Default Fairino controller IP

# ── Control loop ──────────────────────────────────────────────────────────────
LOOP_HZ     = 125                 # ServoJ must run between 60–1000 Hz
LOOP_PERIOD = 1.0 / LOOP_HZ      # ~0.008 s between calls

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
    (-155, 155),   # J3  hardware ±160°
    (-260,  80),   # J4  hardware (-265, 85)
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

# Quest 3 trigger analog mapped to gripper state via hysteresis:
#   trigger ≥ CLOSE_THRESHOLD → close
#   trigger ≤ OPEN_THRESHOLD  → open
#   between                   → hold current state
GRIPPER_OPEN_THRESHOLD  = 0.2   # trigger below this → open
GRIPPER_CLOSE_THRESHOLD = 0.7   # trigger above this → close

# ── Meta Quest 3 VR controller ───────────────────────────────────────────────
#
# Two transport modes:
#   "oculus_reader" — Oculus Reader APK (rail-berkeley/oculus_reader) via ADB
#                     Run: adb forward tcp:5555 tcp:5555
#   "udp"           — lightweight UDP packet streamer (custom app / HTS-compatible)
#
QUEST3_MODE        = "oculus_reader"  # "oculus_reader" | "vuer" | "udp"
QUEST3_ADB_PORT    = 5555             # TCP port after adb forward (oculus_reader mode)
QUEST3_UDP_PORT    = 5005             # UDP port this PC listens on (udp mode)
QUEST3_ACTIVE_HAND = "right"          # which controller drives the arm: "right" | "left"

# Vuer transport (QUEST3_MODE = "vuer") — Quest 3 browser → HTTPS WebXR server
# Generate cert once: openssl req -x509 -newkey rsa:4096 -nodes \
#   -out ssl/cert.pem -keyout ssl/key.pem -days 365 -subj "/CN=quest-teleop"
QUEST3_VUER_PORT   = 8012             # HTTPS port the Quest browser connects to
QUEST3_VUER_CERT   = "./ssl/cert.pem" # path to SSL certificate
QUEST3_VUER_KEY    = "./ssl/key.pem"  # path to SSL private key

# Position scale: 1 metre of controller motion → this many mm of EEF motion.
# Start conservative (300) and increase once the motion feels right.
VR_POSITION_SCALE = 300.0    # mm / m

# Rotation scale: multiplier on controller rotation → EEF rotation (deg).
# 1.0 = 1:1 mapping. Reduce if wrist snaps feel aggressive.
VR_ROTATION_SCALE = 0.8

# Per-cycle safety clamps applied BEFORE IK
VR_MAX_DELTA_POS_MM  = 8.0    # max EEF translation per cycle (mm)
VR_MAX_DELTA_ROT_DEG = 3.0    # max EEF rotation per cycle (deg, per axis)

# Coordinate frame rotation: Quest 3 OpenXR convention → FR5 TCP frame.
# Stored as a 3×3 matrix (row = FR5 axis, col = Quest axis):
#   FR5_X ← -Quest_Z  (Quest backward  → FR5 forward inverted)
#   FR5_Y ←  Quest_X  (Quest right     → FR5 right)
#   FR5_Z ←  Quest_Y  (Quest up        → FR5 up)
# Tune this if EEF motion axes feel wrong on your setup — use ±1 and 0 only.
VR_FRAME_ROTATION = [
    [ 0.0, 0.0, -1.0],   # FR5 X
    [ 1.0, 0.0,  0.0],   # FR5 Y
    [ 0.0, 1.0,  0.0],   # FR5 Z
]

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
