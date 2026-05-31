"""
one_euro.py — One-Euro filter for smoothing the SO-101 leader joints.

The One-Euro filter (Casiez et al., 2012) is an adaptive low-pass filter: it
filters hard when the signal moves slowly (killing tremor / encoder jitter while
the operator holds still) and barely at all when the signal moves fast (so quick
intentional moves are not lagged). Two knobs:

  min_cutoff (Hz) — baseline smoothing. Lower → smoother at rest, more lag.
  beta            — speed coupling. Higher → less lag on fast moves.

Applied per joint at the 125 Hz control rate, using the loop's monotonic time
as the sample timestamp so variable cycle timing is handled correctly.
"""

import math


class OneEuroFilter:
    """Scalar One-Euro filter."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta       = float(beta)
        self.d_cutoff   = float(d_cutoff)
        self._x_prev  = None
        self._dx_prev = 0.0
        self._t_prev  = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        """Clear state so the next sample passes through unfiltered (no startup lag)."""
        self._x_prev  = None
        self._dx_prev = 0.0
        self._t_prev  = None

    def __call__(self, x: float, t: float) -> float:
        if self._x_prev is None or self._t_prev is None:
            self._x_prev = x
            self._t_prev = t
            self._dx_prev = 0.0
            return x

        dt = t - self._t_prev
        if dt <= 0:
            dt = 1e-3   # guard against zero/negative dt

        dx     = (x - self._x_prev) / dt
        a_d    = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a      = self._alpha(cutoff, dt)
        x_hat  = a * x + (1.0 - a) * self._x_prev

        self._x_prev  = x_hat
        self._dx_prev = dx_hat
        self._t_prev  = t
        return x_hat


class JointOneEuro:
    """One-Euro filter bank over a set of named joints (operates on a dict)."""

    def __init__(self, keys, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        self._filters = {k: OneEuroFilter(min_cutoff, beta, d_cutoff) for k in keys}

    def reset(self) -> None:
        for f in self._filters.values():
            f.reset()

    def __call__(self, values: dict, t: float) -> dict:
        """Filter every known key; pass through any keys not in the bank."""
        return {
            k: (self._filters[k](v, t) if k in self._filters else v)
            for k, v in values.items()
        }
