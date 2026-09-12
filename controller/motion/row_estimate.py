"""The contract between perception and the controller.

Deliberately carries no millimetres and no degrees.  The front camera is not
calibrated — on purpose, because row following needs only an angle and an
offset within the image — so everything measurable from it is a pixel ratio.
Declaring those as mm or deg would claim a unit nobody can back up.

The consequence is that the row follower's gains have units of
``(deg/s) / unitless`` and must be tuned rather than derived from geometry.
Saying so outright is better than hiding it under a fake unit.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RowEstimate:
    """One estimate of where the furrow is, in image space.

    ``lateral_err``  furrow offset from the image centre, + = furrow is to the right
    ``heading_err``  tilt of the furrow in the image,     + = furrow tilts right
    ``confidence``   valley prominence, [0, 1]
    ``green_fraction`` share of pixels passing the ExG threshold, [0, 1]

    ``green_fraction`` is the only thing separating "end of the row" from "lost
    the row": both arrive as invalid estimates, and the interpretation differs
    entirely (stop normally vs. fault).
    """

    valid: bool
    lateral_err: float
    heading_err: float
    confidence: float
    green_fraction: float

    def __post_init__(self) -> None:
        for name, low, high in (
            ("lateral_err", -1.0, 1.0),
            ("heading_err", -1.0, 1.0),
            ("confidence", 0.0, 1.0),
            ("green_fraction", 0.0, 1.0),
        ):
            value = getattr(self, name)
            if not low <= value <= high:
                raise ValueError(f"{name} must be within [{low}, {high}], got {value}")
