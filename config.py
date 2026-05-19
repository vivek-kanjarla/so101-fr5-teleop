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
LOOP_HZ     = 125
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
JOINT_SCALE = [-1.000, 1.000, 1.000, 1.000, 1.000]

# Amplification: gear ratio between SO-101 and FR5.
# 2.0 means 1° of SO-101 movement → 2° of FR5 movement.
# Increase to cover FR5 workspace regions the SO-101 can't physically reach.
# Index: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
JOINT_AMP = [1.50, 2.00, 2.00, 3.00, 1.50]

# ── Safety ────────────────────────────────────────────────────────────────────
# Global fallback — used when per-joint limit not specified
MAX_DELTA_DEG_PER_CYCLE = 0.1200

# Per-joint rate limits (degrees/cycle) for [J1, J2, J3, J4, J5, J6]
# Raised 50% vs previous defaults so the FR5 can keep up with human motion
# and coasting after stops is minimised.
MAX_DELTA_PER_JOINT = [0.2400, 0.2000, 0.2000, 0.4000, 0.0800, 0.3000]
#                       J1     J2     J3     J4    J5    J6
#                       30°/s  25°/s  25°/s  50/s  10/s  37°/s

# Per-joint acceleration limits (degrees/cycle²) for [J1, J2, J3, J4, J5, J6]
# Velocity ramps from 0 → rate_limit in ceil(rate/accel) cycles.
# Example: J1 0.24/0.018 = ~14 cycles = 110ms ramp — 16× less jerk than instant step.
MAX_ACCEL_PER_JOINT = [0.0180, 0.0150, 0.0150, 0.0300, 0.0080, 0.0240]
#                       J1     J2     J3     J4    J5    J6

# Encoder noise floor for the STS3215 servo (±1–2 counts = ±0.09–0.18°).
# SO-101 deltas smaller than this (before amplification) are treated as zero.
# Prevents the FR5 from micro-oscillating when the operator holds still.
DEADBAND_SO101_DEG = 0.18

FR5_SERVO_VEL           = 15
FR5_FILTER_T            = 0.04

# From GetJointSoftLimitDeg() on this controller, with 5° margin inside each limit.
FR5_JOINT_LIMITS = [
    (-170, 170),   # J1  hardware ±175°
    (-260, 80),   # J2  hardware (-265, 85)
    (-155, 155),   # J3  hardware ±160°
    (-260, 80),   # J4  hardware (-265, 85) — frozen in mapper
    (-170, 170),   # J5  hardware ±175°
    (-170, 170),   # J6  hardware ±175°
]

# ── DH AG-160-95 gripper (flange-mounted, via Fairino SDK) ───────────────────
GRIPPER_INDEX     = 1      # gripper number configured on the FR5 controller
GRIPPER_TYPE      = 0      # 0=parallel gripper (AG-160-95), 1=rotating

GRIPPER_OPEN_PCT  = 100
GRIPPER_CLOSE_PCT = 0
GRIPPER_VEL_PCT   = 50
GRIPPER_FORCE_PCT = 50
GRIPPER_MAXTIME_MS = 5000

# SO-101 gripper motor position normalised to [0,1]:
#   above OPEN_THRESHOLD  → send open
#   below CLOSE_THRESHOLD → send close
#   between               → hold (hysteresis)
# Calibration: range_min=2028, range_max=3236  → degrees = raw/4096*360
SO101_GRIPPER_RANGE        = (2028/4096*360, 3236/4096*360)  # (178.0°, 284.3°)
SO101_GRIPPER_OPEN_THRESHOLD  = 0.65
SO101_GRIPPER_CLOSE_THRESHOLD = 0.35

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
