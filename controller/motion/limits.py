"""Clamping helpers shared by the motion layer."""

from __future__ import annotations


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into ``[low, high]``."""
    if low > high:
        raise ValueError(f"clamp bounds are inverted: low={low} high={high}")
    return max(low, min(high, value))
