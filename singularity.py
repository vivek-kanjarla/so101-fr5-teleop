"""
singularity.py — detects proximity to singular configurations on the FR5.

Checks two singularities that can be hit during teleoperation:

  Wrist singularity  — J5 ≈ 0°  (J4 and J6 axes align, infinite IK solutions)
  Elbow singularity  — J3 ≈ 0°  (arm near full extension/fold)

Returns a severity level and a velocity scale factor (0.0–1.0) that the
teleop loop multiplies against MAX_DELTA_DEG_PER_CYCLE before sending ServoJ.
"""

from enum import Enum

# ── Thresholds (degrees) ──────────────────────────────────────────────────────
WRIST_WARN_DEG   = 15.0   # |J5| below this → WARN
WRIST_DANGER_DEG =  5.0   # |J5| below this → DANGER

ELBOW_WARN_DEG   =  8.0   # |J3| below this → WARN
ELBOW_DANGER_DEG =  3.0   # |J3| below this → DANGER


class Level(Enum):
    CLEAR  = "CLEAR"
    WARN   = "WARN"
    DANGER = "DANGER"


def _scale(value: float, warn: float, danger: float) -> float:
    """Linear scale 1.0 → 0.1 between warn and danger thresholds."""
    if value >= warn:
        return 1.0
    if value <= danger:
        return 0.0
    return 0.1 + 0.9 * (value - danger) / (warn - danger)


def check(joints_deg: list[float]) -> tuple[Level, float, str]:
    """
    Evaluate singularity proximity for given joint positions.

    Returns:
        level       — worst Level across all checks
        scale       — multiply MAX_DELTA_DEG_PER_CYCLE by this (0.0 = full stop)
        message     — human-readable description of the active constraint
    """
    j3  = abs(joints_deg[2])   # elbow_flex
    j4  = abs(joints_deg[3])   # wrist_flex (now active — J5 is frozen)

    wrist_scale = _scale(j4, WRIST_WARN_DEG, WRIST_DANGER_DEG)
    elbow_scale = _scale(j3, ELBOW_WARN_DEG, ELBOW_DANGER_DEG)

    scale = min(wrist_scale, elbow_scale)

    parts = []
    if j4 <= WRIST_DANGER_DEG:
        parts.append(f"WRIST SINGULARITY (J4={joints_deg[3]:.1f}°) — motion blocked")
        level = Level.DANGER
    elif j4 <= WRIST_WARN_DEG:
        parts.append(f"wrist near singular (J4={joints_deg[3]:.1f}°, limit={WRIST_WARN_DEG}°)")
        level = Level.WARN
    elif j3 <= ELBOW_DANGER_DEG:
        parts.append(f"ELBOW SINGULARITY (J3={joints_deg[2]:.1f}°) — motion blocked")
        level = Level.DANGER
    elif j3 <= ELBOW_WARN_DEG:
        parts.append(f"elbow near singular (J3={joints_deg[2]:.1f}°, limit={ELBOW_WARN_DEG}°)")
        level = Level.WARN
    else:
        level = Level.CLEAR

    message = " | ".join(parts) if parts else ""
    return level, scale, message
