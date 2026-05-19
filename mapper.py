"""
mapper.py — maps SO-101 joint positions to FR5 joint commands.

Uses delta-based mapping: only the *change* from SO-101's home pose is applied
to the FR5's home pose. This means the first command is always Δ=0 regardless
of where either arm is parked, eliminating startup drift.

Rate-limiting pipeline per joint:
  1. Deadband  — zero out deltas below encoder noise floor (stops micro-oscillation).
  2. Target    — compute FR5 target = fr5_home + amplified_SO101_delta.
  3. Joint limit clamp — keep target inside hardware limits.
  4. Dynamic position limit — taper rate as FR5 closes in on target (reduces coasting).
  5. Acceleration limit — step size can only change by MAX_ACCEL_PER_JOINT per cycle
                          (eliminates velocity step / jerk on direction reversals).
"""

import numpy as np
from config import (
    JOINT_SCALE, JOINT_AMP,
    MAX_DELTA_DEG_PER_CYCLE, MAX_DELTA_PER_JOINT, MAX_ACCEL_PER_JOINT,
    FR5_JOINT_LIMITS, DEADBAND_SO101_DEG,
)

ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def so101_to_fr5(
    so101_deg:  dict[str, float],
    so101_home: dict[str, float],
    fr5_home:   list[float],
    prev_fr5_deg: list[float],
    prev_step:    list[float],        # velocity memory: last step size per joint
    delta_limit:  float | None = None,
) -> tuple[list[float], list[float]]:
    """
    Convert SO-101 joint dict → (FR5 6-joint command list, new_step list).

    Returns a tuple so the caller can feed new_step back in next cycle.
    Reset prev_step to [0.0]*6 on re-home or after a gripper pause to avoid
    carrying stale velocity through a discontinuity.
    """
    so101_now  = np.array([so101_deg[k]  for k in ORDER])
    so101_base = np.array([so101_home[k] for k in ORDER])
    delta_raw  = so101_now - so101_base

    # 1. Deadband — suppress encoder noise below the noise floor (pre-amplification).
    mask  = np.abs(delta_raw) > DEADBAND_SO101_DEG
    delta = delta_raw * mask * np.array(JOINT_SCALE) * np.array(JOINT_AMP)

    target = [
        fr5_home[0] + delta[0],  # J1 ← shoulder_pan
        fr5_home[1] + delta[1],  # J2 ← shoulder_lift
        fr5_home[2] + delta[2],  # J3 ← elbow_flex
        fr5_home[3] + delta[3],  # J4 ← wrist_flex
        prev_fr5_deg[4],         # J5 — frozen (no SO-101 counterpart)
        fr5_home[5] + delta[4],  # J6 ← wrist_roll
    ]

    # 2. Joint limit clamp — before rate-limiting so the limiter handles the
    #    imposed delta gracefully rather than passing a sudden jump.
    target_clamped = [
        max(lo, min(hi, t))
        for t, (lo, hi) in zip(target, FR5_JOINT_LIMITS)
    ]

    # Scale all limits down uniformly near a singularity.
    sing_scale = (delta_limit / MAX_DELTA_DEG_PER_CYCLE) if delta_limit is not None else 1.0
    pos_limits  = [lim * sing_scale for lim in MAX_DELTA_PER_JOINT]
    acc_limits  = [lim * sing_scale for lim in MAX_ACCEL_PER_JOINT]

    result    = []
    new_steps = []
    for t, p, ps, plim, alim in zip(target_clamped, prev_fr5_deg, prev_step, pos_limits, acc_limits):
        pos_error = t - p

        # 3. Dynamic position limit — taper the allowed step as error shrinks so
        #    the FR5 decelerates smoothly into the target rather than coasting at
        #    full rate_limit velocity until it arrives.
        #    Ramps from 10% to 100% of plim over a 3×plim error window.
        catchup = min(1.0, abs(pos_error) / (plim * 3))
        effective_plim = plim * max(0.1, catchup)

        desired_step = np.clip(pos_error, -effective_plim, effective_plim)

        # 4. Acceleration limit — step size itself can only change by alim/cycle,
        #    eliminating the instantaneous velocity jump that causes jerk.
        actual_step = float(ps + np.clip(desired_step - ps, -alim, alim))

        # float() is required: numpy.float64 cannot be marshalled by xmlrpc.client.
        result.append(float(p + actual_step))
        new_steps.append(actual_step)

    return result, new_steps
