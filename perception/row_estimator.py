"""Front camera to :class:`RowEstimate` — the valley histogram of §row_estimator.

What the rover follows is the furrow, which is the gap between two bands of
green.  Sum the mask down a strip of the image and the two crop rows are two
hills with a valley between them::

    column-wise green sum over one strip:

      ###                       ###        <- left / right crop row
      ###.                     .###
      #####.        v         .#####
      -----------------------------------  u
                  valley = middle of the furrow

Two strips, two jobs::

    valley_near = find_valley(mask[bottom_third])   # under the wheels
    valley_far  = find_valley(mask[middle_third])   # lookahead

    lateral_err = (valley_near - u_centre) / (width / 2)
    heading_err = (valley_far - valley_near) / (width / 2)
    confidence  = valley prominence, the weaker of the two strips
    valid       = confidence > conf_min on *both* strips

Differencing the two strips is what makes ``heading_err`` free: the tilt of the
furrow falls out of two measurements at two distances, with no camera geometry
anywhere in it.

Why not a line fit or a Hough transform
---------------------------------------
================  ==================================  =======================
                  valley histogram                    line fitting
================  ==================================  =======================
plant missing     survives — the column sum still     the blob it needs is
mid-row           sees the band                       gone; the fit skews
curved furrow     survives — two distances, measured  has to fit a curve
                  separately, no straight line
cost              a sum and an argmax; 10 Hz on a Pi  Hough is far dearer
debugging         plot the histogram, the miss is     blobs plus several
                  visible                             parameters
================  ==================================  =======================

The first row is an acceptance criterion of V1, not a preference.

``valley_prominence`` as confidence is not a number tuned until frames pass:
it is how far the valley sits below the lower of its two shoulders.  When the
two bands do not stand clear of the gap between them, the furrow genuinely is
not visible, and the estimate genuinely should be invalid.

No millimetres and no degrees anywhere
--------------------------------------
The front camera is uncalibrated on purpose, so every number here is a pixel
ratio.  ``RowEstimate``'s docstring carries the consequence: the follower's
gains have units of ``(deg/s) / unitless`` and must be tuned.

Otsu is never re-implemented here.  ``exg.green_mask`` owns the floor beneath
it, and the floor is the only thing stopping Otsu from splitting soil noise
into two classes and reporting vegetation on an empty frame — which would keep
``green_fraction`` high and the end-of-row heuristic silent forever.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from controller.motion import RowEstimate
from perception.exg import green_mask


@dataclass(frozen=True)
class Valley:
    """Where the furrow is in one strip, and how sure that is.

    ``u``            column index, fractional — the middle of the gap
    ``prominence``   [0, 1], how far below the lower shoulder the valley sits
    """

    u: float
    prominence: float


def find_valley(histogram: np.ndarray) -> Valley:
    """The furrow in one column-wise green sum.

    A column is a candidate if there is green on *both* sides of it, and its
    prominence is how far it sits below the lower of the two::

        prominence(u) = (min(max left of u, max right of u) - hist[u])
                        / min(max left of u, max right of u)

    Requiring both shoulders is what disqualifies the open ground outside the
    outer crop row.  Those margins are deeper valleys than the furrow — they
    are empty all the way to the frame edge — and a rover that steered at them
    would leave the bed.  They have no shoulder on one side, so they score 0.

    The winner is the *widest* run of columns tied at the best prominence, not
    the first one: a one-column dropout inside a crop row is as deep as the
    furrow and has shoulders on both sides, and width is what tells the furrow
    from the dropout.  Remaining ties go to the run nearest the image centre —
    with more than one furrow in view, the one the rover is in is the one it
    is pointed at.

    The valley is reported at the middle of that run, which is what makes a
    tilted furrow read straight: over a strip, the columns that are empty for
    every row are the intersection of the sweeping gap, and its middle is the
    furrow's position at the middle row of the strip.
    """
    hist = np.asarray(histogram, dtype=np.float64).ravel()
    columns = hist.size
    if columns < 3:
        return Valley(u=max(columns - 1, 0) / 2.0, prominence=0.0)

    # Tallest column strictly to the left of each column, and to the right.
    left = np.empty(columns)
    left[0] = 0.0
    left[1:] = np.maximum.accumulate(hist)[:-1]

    right = np.empty(columns)
    right[-1] = 0.0
    right[:-1] = np.maximum.accumulate(hist[::-1])[::-1][1:]

    shoulder = np.minimum(left, right)
    prominence = np.zeros(columns)
    np.divide(shoulder - hist, shoulder, out=prominence, where=shoulder > 0.0)
    np.clip(prominence, 0.0, 1.0, out=prominence)

    best = float(prominence.max())
    if best <= 0.0:
        # No column has green on both sides of it: whatever is in this strip,
        # it is not a furrow between two crop rows.
        return Valley(u=(columns - 1) / 2.0, prominence=0.0)

    starts, ends = _runs(prominence >= best - 1e-9)
    centres = (starts + ends - 1) / 2.0
    widest = np.lexsort((np.abs(centres - (columns - 1) / 2.0), starts - ends))

    return Valley(u=float(centres[widest[0]]), prominence=best)


def _runs(flags: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Half-open ``[start, end)`` bounds of each contiguous true run."""
    padded = np.concatenate(([False], flags, [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return edges[0::2], edges[1::2]


class RowEstimator:
    """Turns one front-camera frame into one :class:`RowEstimate`.

    Takes an array rather than a ``cameras.Frame``: it needs pixels and has no
    use for a clock, and a field in the signature it must not read is an
    invitation.  The staleness check that ``camera_timeout_ms`` describes is
    ``Event.CAMERA_TIMEOUT``, which ``row_run.FAULT_EVENTS`` files under faults
    detected outside the control loop.
    """

    #: Fraction of the frame height, measured from the bottom, that each strip
    #: covers.  Thirds, as the spec draws them: the bottom third is where the
    #: wheels are about to be, the one above it is the lookahead.
    NEAR_STRIP = (2 / 3, 1.0)
    FAR_STRIP = (1 / 3, 2 / 3)

    def __init__(self, exg_floor: float, conf_min: float) -> None:
        self.exg_floor = exg_floor
        self.conf_min = conf_min

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> RowEstimator:
        from controller.config import get

        return cls(
            exg_floor=get(config, "perception.exg.exg_floor"),
            conf_min=get(config, "perception.row_estimator.conf_min"),
        )

    def estimate(self, rgb: np.ndarray) -> RowEstimate:
        """One frame in, one estimate out.  Never raises on a bad-looking frame.

        An unreadable frame is an *invalid* estimate and not an exception: §5.5
        rides out ``row_loss_frames`` of those on purpose, and an exception here
        would turn a gap mid-row into a crash in the control loop.
        """
        mask = green_mask(rgb, self.exg_floor)
        height, width = mask.shape

        near = find_valley(self._strip(mask, self.NEAR_STRIP).sum(axis=0))
        far = find_valley(self._strip(mask, self.FAR_STRIP).sum(axis=0))

        half_width = width / 2.0
        centre = (width - 1) / 2.0

        return RowEstimate(
            # Both strips, not the better of the two.  Passing on the near
            # strip alone is a rover trusting a lookahead it cannot see.
            valid=near.prominence > self.conf_min and far.prominence > self.conf_min,
            lateral_err=_unit((near.u - centre) / half_width),
            heading_err=_unit((far.u - near.u) / half_width),
            confidence=min(near.prominence, far.prominence),
            green_fraction=float(mask.mean()),
        )

    @staticmethod
    def _strip(mask: np.ndarray, bounds: tuple[float, float]) -> np.ndarray:
        """The rows of ``mask`` between two fractions of the frame height.

        Rows, and never columns.  ``front_downscale`` is ``[320, 240]``, which
        is (width, height), while the frame is a numpy array of shape
        ``(240, 320)``.  Getting that backwards is silent: the estimate still
        comes out, measured across the wrong axis.
        """
        height = mask.shape[0]
        low, high = bounds
        return mask[int(height * low) : int(height * high)]


def _unit(value: float) -> float:
    """Clamp to the [-1, 1] ``RowEstimate`` enforces.

    ``heading_err`` is a difference between two column indices over a half
    width, so it can reach +/-2 when the two strips find furrows at opposite
    edges.  That is already far past anything a follower can act on, and
    letting it through would make ``RowEstimate`` raise inside the control
    loop rather than report an implausible frame.
    """
    return float(np.clip(value, -1.0, 1.0))
