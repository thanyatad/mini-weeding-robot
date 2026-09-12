"""The furrow centreline the harness follows, in the bed frame.

The bed frame exists in documentation and in simulation, never at runtime:
there is no ``bed -> rover`` transform on the real rover because there is no
localisation.  The harness is allowed to know it precisely because it is the
harness — it plays the part of the world, not the part of the robot.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True)
class Row:
    """Furrow centreline as ``y = centre_offset_mm(x)``, both in mm, bed frame."""

    centre_offset: Callable[[float], float]

    def centre_offset_mm(self, x_mm: float) -> float:
        return float(self.centre_offset(x_mm))


def straight_row(offset_mm: float = 0.0) -> Row:
    """A furrow running straight along bed X.

    Note what a straight row cannot prove: a follower with gain 0 tracks it
    perfectly.  That is exactly why simulation.yaml forbids
    ``row_curvature_mm: 0`` in ordinary testing.  Here the convergence case
    starts off-centre and skewed, so gain 0 fails it.
    """
    return Row(centre_offset=lambda _x: offset_mm)
