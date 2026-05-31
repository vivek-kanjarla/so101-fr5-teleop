"""
velocity_limiter.py — smooth per-joint velocity + acceleration limiter.

Applied to the FR5 ServoJ command as the *last* stage before it is sent. Unlike
a hard clip (np.clip), this uses tanh soft-saturation: the command approaches the
limit asymptotically and decelerates smoothly rather than stepping. That matters
for ACT — abrupt velocity steps inject high-jerk transients into the recorded
`action` stream, which a chunked policy then tries (and fails) to reproduce.

The mapper already rate/accel-limits in joint-delta space; this is an independent,
physically-meaningful (deg/s, deg/s^2) final guard with smooth behaviour. With the
default limits set a little above the mapper envelope it only engages on outliers.

Stateful: call reset(pos) on any discontinuity (startup, re-home, clutch re-engage,
fault recovery) so the limiter does not fight a deliberate position jump.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def _soft_clip(x: float, limit: float) -> float:
    """tanh soft-saturation toward ±limit. Near-identity for |x| << limit."""
    if limit <= 0.0:
        return x
    return limit * math.tanh(x / limit)


@dataclass
class VelocityLimiter:
    vel_limit: list[float]          # per-joint max |velocity| (deg/s)
    acc_limit: list[float]          # per-joint max |acceleration| (deg/s^2)
    dt: float                       # control period (s)
    enabled: bool = True
    _prev_pos: list[float] | None = field(default=None, repr=False)
    _prev_vel: list[float] | None = field(default=None, repr=False)

    def reset(self, pos: list[float]) -> None:
        """Seed state at a known position with zero velocity (no startup jump)."""
        self._prev_pos = [float(p) for p in pos]
        self._prev_vel = [0.0] * len(pos)

    def __call__(self, target: list[float]) -> list[float]:
        if not self.enabled:
            return [float(t) for t in target]
        if self._prev_pos is None or self._prev_vel is None:
            self.reset(target)
            return [float(t) for t in target]

        out: list[float] = []
        for i, t in enumerate(target):
            p_prev = self._prev_pos[i]
            v_prev = self._prev_vel[i]
            vmax   = self.vel_limit[i] if i < len(self.vel_limit) else 0.0
            amax   = self.acc_limit[i] if i < len(self.acc_limit) else 0.0

            v_des = (t - p_prev) / self.dt
            v_lim = _soft_clip(v_des, vmax)
            # Acceleration guard: limit the change in velocity per step (a*dt),
            # again with smooth saturation rather than a hard clamp.
            dv    = _soft_clip(v_lim - v_prev, amax * self.dt)
            v_new = v_prev + dv
            p_new = p_prev + v_new * self.dt

            out.append(float(p_new))
            self._prev_pos[i] = p_new
            self._prev_vel[i] = v_new
        return out
