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
# Lowered progressively from [1.50, 2.00, 2.00, 3.00, 1.50] → [1.25,1.50,1.50,2.00,1.25]:
# the old 3.0× on wrist_flex turned a ~38° leader move into a ~75° follower swing
# that overran a joint limit and tripped ServoJ error 14. Reduced further toward a
# near-1:1 mapping for gentler, more controllable follower motion.
# Index: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
JOINT_AMP = [1.00, 1.25, 1.25, 1.50, 1.00]

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
FR5_FILTER_T            = 0.06   # ServoJ trajectory filter (s). Raised from 0.04
                                 # for smoother follower motion (slightly more lag).

# One-Euro filter on the SO-101 leader joints — adaptive low-pass that removes
# hand tremor / encoder jitter when holding still while staying responsive on
# fast moves. Applied per joint at the control rate. See one_euro.py.
SO101_FILTER_ENABLED    = True
SO101_FILTER_MIN_CUTOFF = 1.0    # Hz — lower = smoother at rest, more lag
SO101_FILTER_BETA       = 0.7    # higher = less lag on fast moves
SO101_FILTER_DCUTOFF    = 1.0    # Hz — derivative cutoff (leave at 1.0)

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

# ── RealSense cameras ─────────────────────────────────────────────────────────
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30    # 30 / 60 / 90 supported; 30 is standard for training data

# Use the RealSense per-frame hardware timestamp (global-time domain) instead of
# the host arrival time (time.time()). Global time maps the device clock onto the
# host epoch, so values stay comparable to the joint log while removing thread-
# scheduling / USB-arrival jitter — giving tighter cross-camera alignment.
# Falls back to time.time() automatically if a device only reports its raw
# hardware clock (which is not on the host epoch).
CAMERA_USE_HW_TIMESTAMP = True

# Per-device serials so the wrist (eye-in-hand) and scene (eye-to-hand) cameras
# are selected explicitly and never swapped. Find them with:
#   rs-enumerate-devices | grep -A1 Name
D405_SERIAL  = "409122273756"   # wrist-mounted D405  → eye-in-hand
D435I_SERIAL = "420122071835"   # fixed external D435i → eye-to-hand

# Camera spec list consumed by teleop.py / logger.py. Each entry becomes one
# LeRobot video key: observation.images.<name>. Order is not significant.
#   enable_depth=True records aligned depth (D435i scene view) alongside color.
CAMERAS = [
    {"name": "wrist_cam", "serial": D405_SERIAL,  "enable_depth": False},  # D405  eye-in-hand
    {"name": "scene_cam", "serial": D435I_SERIAL, "enable_depth": True},   # D435i eye-to-hand
]

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

# ── Clutch (hold-to-engage teleoperation) ─────────────────────────────────────
# When enabled, the FR5 only follows the SO-101 while the clutch key is HELD.
# Releasing it freezes the follower so the operator can reposition the leader
# without moving the robot; pressing it again re-syncs (re-homes) both arms so
# there is no jump. Disabled by default → existing always-on teleop is unchanged.
CLUTCH_ENABLED = False
CLUTCH_KEY     = "ctrl_r"   # ctrl_r | ctrl_l | ctrl | alt_r | shift_r | <single char>

# ── Velocity limiter (smooth saturation before ServoJ) ────────────────────────
# Final per-joint velocity + acceleration guard applied to the ServoJ command.
# Uses tanh soft-saturation (never an abrupt clip), so noisy / aggressive targets
# decelerate smoothly instead of stepping — this is what keeps recorded actions
# clean for ACT (high-jerk samples hurt action-chunk learning).
VEL_LIMITER_ENABLED = True
VEL_LIMIT_DEG_S  = [40.0, 35.0, 35.0, 60.0, 15.0, 45.0]      # per joint J1..J6
ACC_LIMIT_DEG_S2 = [400.0, 350.0, 350.0, 600.0, 150.0, 450.0]  # per joint J1..J6

# ── Automatic episode trimming (applied at export) ────────────────────────────
# Removes dead air before/after the demonstration. Activity = joint-velocity norm
# OR gripper motion above threshold; we keep a margin of context on each side.
# ACT trains better without long idle segments (they bias the policy toward
# "do nothing"), but a small margin preserves approach/retreat dynamics.
TRIM_ENABLED            = True
TRIM_VEL_NORM_THRESH    = 8.0    # deg/s — joint-velocity-norm activity threshold
TRIM_GRIPPER_RATE_THRESH = 0.05  # normalized gripper units/s — grasp activity
TRIM_KEEP_BEFORE_S      = 2.0    # seconds of context kept before first motion
TRIM_KEEP_AFTER_S       = 2.0    # seconds of context kept after final motion

# ── Episode quality scoring ───────────────────────────────────────────────────
# Weighted 0..100 score combining smoothness, completion duration, and motion
# efficiency. Used to rank/filter demonstrations so ACT trains on the best data.
# Success-centric scoring: ACT learns task success, not minimum-jerk paths.
# A successful demo with a few pauses / re-grasps must outrank a smooth failure,
# so success dominates and smoothness is only a small (diagnostic-weight) term.
# Cartesian efficiency is removed from the scalar score (kept as a diagnostic) —
# pick-and-place is inherently multi-waypoint, so straightness is a poor proxy.
# Weights are normalised by their sum, so they need not total 1.0.
QUALITY_W_SUCCESS    = 0.40   # successful=1.0 / partial=0.5 / failed=0.0
QUALITY_W_DURATION   = 0.25
QUALITY_W_GRASP      = 0.15
QUALITY_W_PAUSE      = 0.10
QUALITY_W_SMOOTHNESS = 0.10   # diagnostic-weight only

QUALITY_TARGET_DURATION_S = 35.0    # 3-block pick-and-place (task-specific, configurable)
QUALITY_EXPECTED_GRASPS   = 3       # pick+place 3 blocks → 3 grasp (close) events

QUALITY_JERK_REF          = 12000.0 # deg/s^3 — commanded-jerk scale for smoothness
QUALITY_PAUSE_VEL_THRESH  = 5.0     # deg/s — below this counts toward a pause
QUALITY_PAUSE_MIN_S       = 0.3     # min duration to count as a distinct pause
QUALITY_PAUSE_FREE        = 3       # pauses allowed free (alignment corrections)
QUALITY_PAUSE_REF         = 12      # pauses beyond free that drive pause_score → 0

# ── ACT export targets ────────────────────────────────────────────────────────
# Surfaced into the dataset metadata and act_config.yaml so training/deployment
# read consistent values. recommended_policy_frequency ≈ dataset_hz / 2 is a safe
# ACT query rate (open-loop chunk replays ~half the chunk before re-querying).
ACT_FPS              = 30
ACT_CHUNK_SIZE       = 50
ACT_POLICY_FREQUENCY = 15
