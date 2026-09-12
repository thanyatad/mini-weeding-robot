"""Skid-steer mixing: (v, omega) -> left and right wheel speeds.

This formula is implemented twice in places that cannot share code — here, and
in C++ in firmware/esp32/.  Both sides are held to the same table,
``config/drive_mixing_vectors.csv``, which is the single source of truth.

Deliberately free of any framework import (no numpy, no config loader): the
C++ mirror has none of that either, and the two must stay line-for-line
comparable.

Sign convention (hardware/mechanical/coordinate-frames.md): ``v > 0`` forward,
``omega > 0`` turns left (CCW, z up), so the right wheel runs faster.

The formula is *nominal*.  Real skid-steer slips, so the omega the rover
achieves is not the omega commanded — the row follower closes the loop on
image, not on this.  It is also why gains do not transfer from sim to hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class WheelSpeeds:
    """Commanded surface speed of each side, mm/s.  Not measured — no encoders."""

    left_mm_s: float
    right_mm_s: float


def mix(
    v_mm_s: float,
    omega_deg_s: float,
    *,
    track_width_mm: float,
    wheel_v_max_mm_s: float,
) -> WheelSpeeds:
    """Mix a body velocity into wheel speeds, scaling both sides if either saturates.

    When a wheel would exceed ``wheel_v_max_mm_s``, both sides are multiplied by
    the same factor.  Clipping only the fast side would change the ratio between
    the wheels — and so the turn the rover actually drives — with nothing in the
    system able to notice.  Scaling keeps the ratio and gives up speed instead,
    which the row follower can see and correct for.
    """
    if wheel_v_max_mm_s <= 0:
        raise ValueError(f"wheel_v_max_mm_s must be positive, got {wheel_v_max_mm_s}")

    omega_rad_s = math.radians(omega_deg_s)
    half_track_mm = track_width_mm / 2.0

    v_left = v_mm_s - omega_rad_s * half_track_mm
    v_right = v_mm_s + omega_rad_s * half_track_mm

    peak = max(abs(v_left), abs(v_right))
    if peak > wheel_v_max_mm_s:
        scale = wheel_v_max_mm_s / peak
        v_left *= scale
        v_right *= scale

    return WheelSpeeds(left_mm_s=v_left, right_mm_s=v_right)
