"""
mapper.py — maps SO-101 joint positions to FR5 joint commands.

Uses delta-based mapping: only the *change* from SO-101's home pose is applied
to the FR5's home pose. This means the first command is always Δ=0 regardless
of where either arm is parked, eliminating startup drift.
"""

import numpy as np
from config import JOINT_SCALE, JOINT_AMP, MAX_DELTA_DEG_PER_CYCLE, MAX_DELTA_PER_JOINT, FR5_JOINT_LIMITS

ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def so101_to_fr5(
    so101_deg: dict[str, float],
    so101_home: dict[str, float],
    fr5_home: list[float],
    prev_fr5_deg: list[float],
    delta_limit: float | None = None,
) -> list[float]:
    """
    Convert SO-101 joint dict → FR5 6-joint command list.

    Steps:
      1. Compute SO-101 delta from its home pose.
      2. Apply JOINT_SCALE (direction flip where needed).
      3. Add scaled delta to FR5 home pose.
      4. Hold J4 at its home value (no SO-101 counterpart).
      5. Rate-limit each joint to MAX_DELTA_DEG_PER_CYCLE.
    """
    so101_now  = np.array([so101_deg[k]  for k in ORDER])
    so101_base = np.array([so101_home[k] for k in ORDER])
    delta      = (so101_now - so101_base) * np.array(JOINT_SCALE) * np.array(JOINT_AMP)

    target = [
        fr5_home[0] + delta[0],  # J1 ← shoulder_pan
        fr5_home[1] + delta[1],  # J2 ← shoulder_lift
        fr5_home[2] + delta[2],  # J3 ← elbow_flex
        fr5_home[3] + delta[3],  # J4 ← wrist_flex
        prev_fr5_deg[4],         # J5 — frozen (no SO-101 counterpart)
        fr5_home[5] + delta[4],  # J6 ← wrist_roll
    ]

    # Clamp target to joint limits BEFORE rate-limiting so the rate limiter
    # handles any limit-imposed delta gracefully instead of passing a sudden jump.
    target_clamped = [
        max(lo, min(hi, t))
        for t, (lo, hi) in zip(target, FR5_JOINT_LIMITS)
    ]

    # Per-joint limits — scale down by singularity factor if provided
    sing_scale = (delta_limit / MAX_DELTA_DEG_PER_CYCLE) if delta_limit is not None else 1.0
    limits = [lim * sing_scale for lim in MAX_DELTA_PER_JOINT]

    result = []
    for t, p, lim in zip(target_clamped, prev_fr5_deg, limits):
        d = np.clip(t - p, -lim, lim)
        result.append(p + d)

    return result
