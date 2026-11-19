"""A slowdown-only clock for the tight turns in smooth pursuit."""

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.ndimage import gaussian_filter1d


def pursuit_turn_clock(
    times_s: np.ndarray, tcp_m: np.ndarray, corner_speed_fraction: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return new waypoint times, dt_new/dt_old, and its old-time derivative.

    Estimate how fast the direction of the LED is changing, not its distance
    to a waypoint. Blend in extra time around tighter turns. PCHIP preserves
    the scale bounds with a continuous derivative; integrating that scale
    guarantees we never shorten an interval or insert a dwell.
    """
    if (
        times_s.ndim != 1
        or len(times_s) < 3
        or tcp_m.shape != (len(times_s), 3)
        or not np.isfinite(times_s).all()
        or not np.isfinite(tcp_m).all()
        or times_s[0] < 0
        or np.any(np.diff(times_s) <= 0)
        or not 0.7 <= corner_speed_fraction < 1.0
    ):
        raise ValueError("Invalid trajectory samples or corner speed fraction")

    span = times_s[-1] - times_s[0]
    grid = np.linspace(
        times_s[0], times_s[-1], max(3, int(np.ceil(span / 0.05)) + 1)
    )
    dt = grid[1] - grid[0]
    xyz = np.column_stack([np.interp(grid, times_s, axis) for axis in tcp_m.T])
    # Suppress small IK and retiming variations before measuring turn rate.
    # No change is made to any commanded positions.
    xyz = gaussian_filter1d(xyz, 0.22 / dt, axis=0, mode="nearest")
    velocity = np.gradient(xyz, dt, axis=0)
    acceleration = np.gradient(velocity, dt, axis=0)
    speed2 = np.sum(velocity**2, axis=1)
    turn_rate = np.linalg.norm(np.cross(velocity, acceleration), axis=1) / (
        speed2 + 0.012**2
    )
    strength = np.clip((turn_rate - 0.70) / (2.20 - 0.70), 0.0, 1.0)
    strength = strength**3 * (10.0 - 15.0 * strength + 6.0 * strength**2)
    strength = gaussian_filter1d(strength, 0.45 / dt, mode="nearest")
    extra = (1.0 / corner_speed_fraction - 1.0) * strength
    clock_rate = PchipInterpolator(grid, 1.0 + extra)
    elapsed = clock_rate.antiderivative()
    return (
        times_s[0] + elapsed(times_s) - elapsed(times_s[0]),
        clock_rate(times_s),
        clock_rate.derivative()(times_s),
    )
