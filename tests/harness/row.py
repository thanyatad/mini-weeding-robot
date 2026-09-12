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
    """Furrow centreline as ``y = centre_offset_mm(x)``, both in mm, bed frame.

    ``length_mm`` is where the furrow stops existing.  ``None`` means it runs
    forever, which is what a convergence test wants and what an ending cannot
    be got out of: past the end there is simply nothing to sample, and that is
    the whole mechanism behind the invalid estimates the row-loss watchdog
    counts.  The bed in the scenario files is 2000 mm.
    """

    centre_offset: Callable[[float], float]
    length_mm: float | None = None

    def centre_offset_mm(self, x_mm: float) -> float:
        return float(self.centre_offset(x_mm))

    def furthest_x_mm(self, limit_mm: float) -> float:
        """``limit_mm``, or the end of the furrow if that comes first."""
        if self.length_mm is None:
            return limit_mm
        return min(limit_mm, self.length_mm)


def straight_row(offset_mm: float = 0.0, length_mm: float | None = None) -> Row:
    """A furrow running straight along bed X, ending at ``length_mm`` if given.

    Note what a straight row cannot prove: a follower with gain 0 tracks it
    perfectly.  That is exactly why simulation.yaml forbids
    ``row_curvature_mm: 0`` in ordinary testing.  Here the convergence case
    starts off-centre and skewed, so gain 0 fails it.
    """
    return Row(centre_offset=lambda _x: offset_mm, length_mm=length_mm)
