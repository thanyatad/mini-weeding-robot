"""ExG segmentation, and the floor that keeps Otsu from inventing plants.

The dead spot this guards: Otsu always finds a threshold.  Given a frame with
no green in it at all, it splits the noise into two classes and hands back a
mask that looks like vegetation.  Downstream that reads as "still plenty of
green", and the end-of-row heuristic never fires -- the rover drives off the
end of the bed.

So the important test here is the one that feeds bare soil and demands an empty
mask, together with the one proving Otsu alone would not have given one.
"""

import numpy as np
import pytest

from controller.config import get, load_config
from perception.exg import exg_index, green_mask, otsu_threshold

EXG_FLOOR = 12
SHAPE = (240, 320)  # perception.row_estimator.front_downscale


def bare_soil(seed: int = 0, shape: tuple[int, int] = SHAPE) -> np.ndarray:
    """A frame of soil and nothing else.

    Soil is genuinely ExG-negative -- dry earth is red-dominant -- and its
    texture is mostly shading, which scales all three channels together and so
    very nearly cancels in 2G - R - B.  That is the physical reason ExG works
    at all, and it is what makes the floor a usable guard rather than a number
    that has to be lucky.
    """
    rng = np.random.default_rng(seed)
    base = np.array([160.0, 120.0, 90.0])  # ExG = 2*120 - 160 - 90 = -10

    shading = rng.uniform(0.55, 1.30, size=(*shape, 1))
    chroma_noise = rng.normal(0.0, 1.5, size=(*shape, 3))

    image = base * shading + chroma_noise
    return np.clip(image, 0, 255).astype(np.uint8)


def soil_with_crops(seed: int = 0, shape: tuple[int, int] = SHAPE) -> np.ndarray:
    """Bare soil with two green bands on it, the furrow running between them."""
    image = bare_soil(seed, shape).astype(np.float64)
    leaf = np.array([70.0, 150.0, 60.0])  # ExG = 300 - 70 - 60 = +170

    width = shape[1]
    for centre in (width // 4, 3 * width // 4):
        image[:, centre - 30 : centre + 30] = leaf

    return np.clip(image, 0, 255).astype(np.uint8)


class TestExgIndex:
    def test_it_is_two_g_minus_r_minus_b(self):
        pixel = np.array([[[10, 50, 30]]], dtype=np.uint8)
        assert exg_index(pixel)[0, 0] == pytest.approx(2 * 50 - 10 - 30)

    def test_it_does_not_wrap_around_on_uint8(self):
        """2G - R - B is negative over most of a soil frame.  Computed in uint8
        that becomes a large positive number and every threshold after it is
        meaningless."""
        pixel = np.array([[[255, 0, 255]]], dtype=np.uint8)
        assert exg_index(pixel)[0, 0] == pytest.approx(-510.0)

    def test_it_is_indifferent_to_brightness(self):
        """Half the light, half the index -- no sign change.  This is why a
        per-frame threshold works across shade and sun without retuning."""
        bright = np.array([[[160, 120, 90]]], dtype=np.uint8)
        dim = np.array([[[80, 60, 45]]], dtype=np.uint8)
        assert exg_index(bright)[0, 0] == pytest.approx(2 * exg_index(dim)[0, 0])

    def test_it_returns_one_value_per_pixel(self):
        assert exg_index(bare_soil()).shape == SHAPE


class TestAbsoluteFloor:
    def test_bare_soil_yields_an_empty_mask(self):
        """The acceptance criterion."""
        assert not green_mask(bare_soil(), exg_floor=EXG_FLOOR).any()

    @pytest.mark.parametrize("seed", range(8))
    def test_bare_soil_yields_an_empty_mask_whatever_the_noise(self, seed):
        assert not green_mask(bare_soil(seed), exg_floor=EXG_FLOOR).any()

    def test_otsu_on_its_own_would_have_invented_plants(self):
        """Without the floor there is nothing to stop this: Otsu splits the soil
        noise into two classes and reports roughly half the frame as green."""
        exg = exg_index(bare_soil())
        would_be_mask = exg > otsu_threshold(exg)

        assert would_be_mask.any()
        assert would_be_mask.mean() > 0.05

    def test_the_mask_stays_empty_rather_than_nearly_empty(self):
        """A handful of stray pixels is still a green_fraction above zero, and
        the end-of-row heuristic reads green_fraction."""
        assert green_mask(bare_soil(), exg_floor=EXG_FLOOR).sum() == 0

    def test_a_floor_of_zero_gives_the_dead_spot_back(self):
        """Proof the floor is what is doing the work, not the test image."""
        assert green_mask(bare_soil(), exg_floor=0).any()

    def test_the_floor_is_tested_against_the_peak_not_the_average(self):
        """One small plant in a frame of soil barely moves the mean.  It has to
        survive, or the detector goes blind exactly at the row end where a last
        remaining crop still matters."""
        image = bare_soil()
        image[10:14, 10:14] = [70, 150, 60]
        assert green_mask(image, exg_floor=EXG_FLOOR).any()


class TestGreenMask:
    def test_crops_are_found(self):
        mask = green_mask(soil_with_crops(), exg_floor=EXG_FLOOR)
        assert mask.any()
        assert 0.2 < mask.mean() < 0.6

    def test_the_furrow_between_the_bands_stays_dark(self):
        """What the row estimator looks for is the valley, so the gap between
        the crop bands must not be filled in."""
        mask = green_mask(soil_with_crops(), exg_floor=EXG_FLOOR)
        centre = mask[:, 150:170]
        assert not centre.any()

    def test_the_threshold_follows_the_frame_rather_than_a_constant(self):
        """Wet soil is a different colour from dry soil.  A per-frame threshold
        is what makes watering stop being a retuning event."""
        dry = soil_with_crops()
        wet = (soil_with_crops().astype(np.float64) * 0.45).astype(np.uint8)

        dry_mask = green_mask(dry, exg_floor=EXG_FLOOR)
        wet_mask = green_mask(wet, exg_floor=EXG_FLOOR)
        assert wet_mask.mean() == pytest.approx(dry_mask.mean(), abs=0.05)

    def test_the_mask_is_boolean_and_frame_shaped(self):
        mask = green_mask(soil_with_crops(), exg_floor=EXG_FLOOR)
        assert mask.dtype == bool
        assert mask.shape == SHAPE

    def test_the_input_frame_is_not_modified(self):
        image = soil_with_crops()
        before = image.copy()
        green_mask(image, exg_floor=EXG_FLOOR)
        assert np.array_equal(image, before)

    def test_a_frame_that_is_all_leaf_is_all_mask(self):
        image = np.full((*SHAPE, 3), [70, 150, 60], dtype=np.uint8)
        assert green_mask(image, exg_floor=EXG_FLOOR).all()


class TestOtsuThreshold:
    def test_it_separates_two_clear_populations(self):
        values = np.concatenate([np.full(500, -10.0), np.full(500, 170.0)])
        assert -10.0 < otsu_threshold(values) < 170.0

    def test_it_lands_between_the_modes_of_a_real_frame(self):
        exg = exg_index(soil_with_crops())
        threshold = otsu_threshold(exg)
        assert exg.min() < threshold < exg.max()

    def test_a_single_valued_frame_does_not_crash(self):
        assert otsu_threshold(np.zeros(100)) == pytest.approx(0.0)


class TestConfiguredFloor:
    def test_the_shipped_floor_is_the_one_under_test(self):
        assert get(load_config(), "perception.exg.exg_floor") == EXG_FLOOR

    def test_the_shipped_floor_clears_bare_soil_and_still_finds_crops(self):
        floor = get(load_config(), "perception.exg.exg_floor")
        assert not green_mask(bare_soil(), exg_floor=floor).any()
        assert green_mask(soil_with_crops(), exg_floor=floor).any()
