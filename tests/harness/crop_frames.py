"""Synthetic front-camera frames: soil, crop rows, and the furrow between them.

Here rather than beside one test because three suites need them now -- the ExG
floor, the row estimator, and the closed loop driven from pixels -- which is
the same reason ``Bench`` is here.

Everything is generated rather than rendered.  That is the whole point of Stage
1: the row estimator can be held to its acceptance criteria with no GPU, no
Isaac and no camera, on frames whose furrow position is known exactly because
it was chosen.  What it cannot settle is any number that depends on what real
foliage looks like -- ``conf_min`` and ``green_fraction_row_end`` both need
rendered crops, which is V1/V2 work.
"""

from __future__ import annotations

import numpy as np

#: ``perception.row_estimator.front_downscale`` is ``[320, 240]``, which is
#: (width, height).  The numpy frame is the other way round.
SHAPE = (240, 320)

SOIL = np.array([160.0, 120.0, 90.0])  # ExG = 2*120 - 160 - 90 = -10
LEAF = np.array([70.0, 150.0, 60.0])  # ExG = 300 - 70 - 60 = +170

HALF_SPACING = 80  # furrow centre to crop-row centre, px
BAND_HALF = 30  # half-width of one crop row, px


def bare_soil(seed: int = 0, shape: tuple[int, int] = SHAPE) -> np.ndarray:
    """A frame of soil and nothing else.

    Soil is genuinely ExG-negative -- dry earth is red-dominant -- and its
    texture is mostly shading, which scales all three channels together and so
    very nearly cancels in 2G - R - B.  That is the physical reason ExG works
    at all, and it is what makes the floor a usable guard rather than a number
    that has to be lucky.
    """
    rng = np.random.default_rng(seed)

    shading = rng.uniform(0.55, 1.30, size=(*shape, 1))
    chroma_noise = rng.normal(0.0, 1.5, size=(*shape, 3))

    image = SOIL * shading + chroma_noise
    return np.clip(image, 0, 255).astype(np.uint8)


def soil_with_crops(seed: int = 0, shape: tuple[int, int] = SHAPE) -> np.ndarray:
    """Bare soil with two green bands on it, the furrow running between them."""
    image = bare_soil(seed, shape).astype(np.float64)

    width = shape[1]
    for centre in (width // 4, 3 * width // 4):
        image[:, centre - 30 : centre + 30] = LEAF

    return np.clip(image, 0, 255).astype(np.uint8)


def crop_rows(
    near_u: float | None = None,
    far_u: float | None = None,
    top_u: float | None = None,
    seed: int = 0,
    shape: tuple[int, int] = SHAPE,
    missing_left_rows: tuple[int, int] | None = None,
    right_band_only: bool = False,
) -> np.ndarray:
    """Two crop rows on soil, with a furrow of stated position between them.

    ``near_u`` is where the furrow crosses the middle row of the bottom strip
    and ``far_u`` where it crosses the middle row of the lookahead strip, so a
    test can state the two numbers the estimator is supposed to recover.

    ``top_u`` bends the furrow: with it, the centre line is the parabola
    through all three stated points instead of the straight line through two.
    Left out, the furrow is straight, which is what every test that does not
    care about curvature wants.

    ``missing_left_rows`` wipes the left crop row over a stretch of image rows:
    a plant missing mid-row, which is the case that separates a histogram from
    a line fit.
    """
    height, width = shape
    if near_u is None:
        near_u = (width - 1) / 2
    if far_u is None:
        far_u = near_u

    soil = bare_soil(seed, shape).astype(np.float64)
    image = soil.copy()

    v_near, v_far = 5 * height / 6, height / 2
    if top_u is None:
        # Arithmetic, not np.polyfit: the band edges of a centred furrow land on
        # exactly x.5, where a least-squares fit's last bit of float noise
        # decides which way round() goes and moves the whole furrow a pixel.
        slope = (far_u - near_u) / (v_far - v_near)

        def centre_at(v: float) -> float:
            return near_u + slope * (v - v_near)
    else:
        centre_at = np.poly1d(np.polyfit([v_near, v_far, 0.0], [near_u, far_u, top_u], 2))

    sides = (1,) if right_band_only else (-1, 1)
    for v in range(height):
        centre = float(centre_at(v))
        for side in sides:
            band = centre + side * HALF_SPACING
            low = max(int(round(band - BAND_HALF)), 0)
            high = min(int(round(band + BAND_HALF)), width)
            if high > low:
                image[v, low:high] = LEAF

    if missing_left_rows is not None:
        first, last = missing_left_rows
        image[first:last, : width // 2] = soil[first:last, : width // 2]

    return np.clip(image, 0, 255).astype(np.uint8)


def receding_row(
    step: float,
    seed: int = 0,
    shape: tuple[int, int] = SHAPE,
) -> list[np.ndarray]:
    """A row running out, one frame per ``step`` of the frame height driven.

    The crops survive only in the bottom ``q`` of the frame and ``q`` falls by
    ``step`` each frame, so ``step`` is how fast the rover is closing on the
    end of the row, measured in the only unit an uncalibrated camera has:
    fractions of its own frame.
    """
    height = shape[0]
    frames, q = [], 1.0

    while q > -1e-9:
        image = crop_rows(seed=seed, shape=shape)
        bare_to = int(round(height * (1 - q)))
        image[:bare_to] = bare_soil(seed, shape)[:bare_to]
        frames.append(image)
        q -= step

    return frames
