"""ExG segmentation: excess green, a per-frame Otsu threshold, and an absolute
floor underneath both.

Shared by the two pipelines that otherwise have nothing in common -- the row
estimator on the front camera, the weed detector on the down camera.

    exg = 2G - R - B
    if exg.max() < exg_floor:  ->  empty mask
    mask = exg > otsu(exg)

Why ExG and not a colour threshold: ExG measures green *relative* to red and
blue, so it survives the light and the soil colour changing.  Wet soil and dry
soil are very different colours; a threshold on raw channels has to be retuned
every time the bed is watered.

Why Otsu per frame and not a constant: same reason, one level up.  Otsu reads
the threshold off the frame's own histogram, so nobody has to set it.

Why the floor: Otsu always returns a threshold.  Handed a frame with no green
in it at all, it splits the noise into two classes and returns a mask that
looks like vegetation -- and the end-of-row heuristic, which reads
green_fraction, never fires.  ``exg_floor`` is the one value in this pipeline
that has to be tuned by hand, and config/README.md says so.
"""

from __future__ import annotations

import numpy as np


def exg_index(rgb: np.ndarray) -> np.ndarray:
    """Excess green, ``2G - R - B``, one float per pixel.

    Computed in float on purpose: the index is negative across most of a soil
    frame, and in uint8 that wraps to a large positive number, which quietly
    makes every threshold downstream meaningless.
    """
    image = np.asarray(rgb, dtype=np.float64)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError(f"expected an (h, w, 3) RGB frame, got shape {image.shape}")

    red, green, blue = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    return 2.0 * green - red - blue


def otsu_threshold(values: np.ndarray, bins: int = 256) -> float:
    """Otsu's threshold over ``values``: the split maximising between-class variance.

    Returns the minimum when every value is identical -- there is no split to
    find, and the caller's floor is what decides that case anyway.
    """
    flat = np.asarray(values, dtype=np.float64).ravel()
    low, high = float(flat.min()), float(flat.max())
    if high <= low:
        return low

    counts, edges = np.histogram(flat, bins=bins, range=(low, high))
    centres = (edges[:-1] + edges[1:]) / 2.0

    weight_below = np.cumsum(counts)
    weight_above = weight_below[-1] - weight_below

    sum_below = np.cumsum(counts * centres)
    sum_above = sum_below[-1] - sum_below

    # Splits that leave one side empty tell us nothing; drop them.
    usable = (weight_below > 0) & (weight_above > 0)
    if not usable.any():
        return low

    mean_below = np.divide(sum_below, weight_below, out=np.zeros_like(sum_below), where=usable)
    mean_above = np.divide(sum_above, weight_above, out=np.zeros_like(sum_above), where=usable)

    between = weight_below * weight_above * (mean_below - mean_above) ** 2
    between[~usable] = -np.inf

    return float(centres[int(np.argmax(between))])


def green_mask(rgb: np.ndarray, exg_floor: float) -> np.ndarray:
    """Boolean mask of vegetation, or an all-false mask when the frame has none.

    ``exg_floor`` is compared against the frame's *peak* excess green, not its
    mean.  A single small plant in a frame of soil barely moves the mean, and
    losing it is losing the detector exactly at the row end, where one
    remaining crop is the whole question.
    """
    exg = exg_index(rgb)

    if exg.max() < exg_floor:
        # No real green in the frame.  Otsu is not consulted: asked to split
        # soil noise it would answer, and the answer would look like plants.
        return np.zeros(exg.shape, dtype=bool)

    if exg.max() == exg.min():
        # Uniform frame.  There is no split for Otsu to find, and the floor has
        # already established the frame is green, so all of it is.  The mirror
        # of the case above, and just as silent if left to Otsu: it would
        # return the single value and `>` would mask nothing.
        return np.ones(exg.shape, dtype=bool)

    return exg > otsu_threshold(exg)
