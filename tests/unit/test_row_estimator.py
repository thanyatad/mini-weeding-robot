"""The valley histogram, and the five acceptance criteria of V1-ISAAC Stage 1.

What the rover follows is the *furrow* -- the gap between two bands of green --
so the estimator sums the mask column-wise over a strip and looks for the
valley between the two shoulders.  Two strips, two jobs: the bottom third sits
under the wheels and gives the lateral offset, the middle third is the
lookahead and, by difference, the heading.

The reason it is a histogram and not a fitted line is the crop gap: when plants
are missing along a stretch of row, the blob a line fit needs is gone, while
the column sum over the whole strip still sees the band.  That case is an
acceptance criterion, and :class:`TestCropGapMidRow` is it.

Nothing here is in millimetres or degrees.  The front camera is not calibrated,
on purpose, so everything measurable from it is a pixel ratio.
"""

import numpy as np
import pytest

from controller.config import get, load_config
from controller.motion import RowEstimate
from perception.row_estimator import RowEstimator, find_valley
from tests.harness import LEAF, SHAPE, bare_soil, crop_rows

EXG_FLOOR = 12
CONF_MIN = 0.25


def an_estimator(conf_min: float = CONF_MIN) -> RowEstimator:
    return RowEstimator(exg_floor=EXG_FLOOR, conf_min=conf_min)


class TestFindValley:
    def test_it_lands_in_the_middle_of_the_gap(self):
        histogram = np.array([0, 8, 8, 0, 0, 0, 0, 8, 8, 0], dtype=float)
        assert find_valley(histogram).u == pytest.approx(4.5)

    def test_an_empty_gap_is_fully_prominent(self):
        histogram = np.array([0, 8, 8, 0, 0, 0, 0, 8, 8, 0], dtype=float)
        assert find_valley(histogram).prominence == pytest.approx(1.0)

    def test_prominence_falls_as_the_furrow_fills_in(self):
        """Prominence is the depth of the valley against the lower of its two
        shoulders, which is what makes it mean something physical: green in the
        furrow is exactly the thing that should cost confidence."""
        def furrow_filled_to(level: float) -> float:
            histogram = np.array([0, 8, 8, level, level, 8, 8, 0], dtype=float)
            return find_valley(histogram).prominence

        assert furrow_filled_to(0) > furrow_filled_to(2) > furrow_filled_to(6)

    def test_prominence_is_the_fraction_of_the_shoulder_the_valley_is_below(self):
        histogram = np.array([0, 8, 8, 2, 2, 8, 8, 0], dtype=float)
        assert find_valley(histogram).prominence == pytest.approx(0.75)

    def test_the_shorter_shoulder_is_the_one_that_counts(self):
        """A tall band on one side cannot vouch for a furrow the other side
        barely borders -- that is a rover believing an edge it half sees."""
        histogram = np.array([0, 20, 20, 2, 2, 4, 4, 0], dtype=float)
        assert find_valley(histogram).prominence == pytest.approx(0.5)

    def test_a_gap_with_green_on_only_one_side_is_not_a_furrow(self):
        """The empty margins outside the outer crop row are deep valleys too.
        What disqualifies them is having no shoulder on one side."""
        histogram = np.array([0, 0, 0, 0, 8, 8, 8, 8], dtype=float)
        assert find_valley(histogram).prominence == pytest.approx(0.0)

    def test_an_empty_histogram_has_no_valley(self):
        assert find_valley(np.zeros(10)).prominence == pytest.approx(0.0)

    def test_the_widest_gap_wins_over_a_nick_inside_a_band(self):
        """A one-column dropout inside a crop row is as deep as the furrow and
        has shoulders on both sides.  Width is what tells them apart."""
        histogram = np.array([8, 8, 0, 8, 8, 0, 0, 0, 0, 0, 8, 8], dtype=float)
        assert find_valley(histogram).u == pytest.approx(7.0)

    def test_the_valley_is_reported_as_a_column_index(self):
        histogram = np.array([0, 8, 0, 0, 8, 0], dtype=float)
        assert 0 <= find_valley(histogram).u <= len(histogram) - 1


class TestStraightFurrow:
    def test_a_centred_furrow_reads_as_no_lateral_error(self):
        est = an_estimator().estimate(crop_rows())
        assert est.lateral_err == pytest.approx(0.0, abs=0.02)

    def test_a_centred_furrow_reads_as_no_heading_error(self):
        est = an_estimator().estimate(crop_rows())
        assert est.heading_err == pytest.approx(0.0, abs=0.02)

    def test_a_centred_furrow_is_valid(self):
        assert an_estimator().estimate(crop_rows()).valid

    def test_it_returns_a_row_estimate(self):
        assert isinstance(an_estimator().estimate(crop_rows()), RowEstimate)


class TestLateralSign:
    def test_a_furrow_to_the_right_reads_positive(self):
        """The sign convention `RowEstimate` documents: + = the furrow is to
        the right of where the rover is pointed."""
        est = an_estimator().estimate(crop_rows(near_u=200))
        assert est.lateral_err > 0

    def test_a_furrow_to_the_left_reads_negative(self):
        est = an_estimator().estimate(crop_rows(near_u=120))
        assert est.lateral_err < 0

    def test_the_magnitude_is_the_offset_as_a_fraction_of_a_half_width(self):
        width = SHAPE[1]
        offset = 40
        est = an_estimator().estimate(crop_rows(near_u=(width - 1) / 2 + offset))
        assert est.lateral_err == pytest.approx(offset / (width / 2), abs=0.02)

    def test_a_bigger_offset_reads_bigger(self):
        gentle = an_estimator().estimate(crop_rows(near_u=180))
        hard = an_estimator().estimate(crop_rows(near_u=210))
        assert hard.lateral_err > gentle.lateral_err


class TestHeadingSign:
    def test_a_furrow_leaning_right_reads_positive(self):
        """Leaning right means the lookahead strip finds the furrow further
        right than the strip under the wheels does."""
        est = an_estimator().estimate(crop_rows(near_u=160, far_u=200))
        assert est.heading_err > 0

    def test_a_furrow_leaning_left_reads_negative(self):
        est = an_estimator().estimate(crop_rows(near_u=160, far_u=120))
        assert est.heading_err < 0

    def test_heading_is_independent_of_lateral_offset(self):
        """The two strips are differenced, so a furrow that is parallel but
        off to one side is a lateral error and not a heading error.  A follower
        that saw heading here would steer to a heading it is already on."""
        est = an_estimator().estimate(crop_rows(near_u=200, far_u=200))
        assert est.heading_err == pytest.approx(0.0, abs=0.02)
        assert est.lateral_err > 0.2


class TestCurvedFurrow:
    """The second row of README's table: a curve needs no curve to be fitted.

    `top_u` bends the furrow into the parabola through all three stated points
    instead of the line through two.
    """

    CENTRE = (SHAPE[1] - 1) / 2

    @pytest.mark.parametrize(
        "far_u, top_u",
        [
            (CENTRE + 10, CENTRE + 50),  # bending right
            (CENTRE - 10, CENTRE - 50),  # bending left
            (CENTRE + 20, CENTRE + 90),  # hard right
            (CENTRE + 25, CENTRE - 15),  # an S-bend, worst case for one fit
        ],
    )
    def test_a_curved_furrow_is_still_valid(self, far_u, top_u):
        assert an_estimator().estimate(crop_rows(far_u=far_u, top_u=top_u)).valid

    def test_curvature_costs_no_confidence(self):
        """Prominence is measured within a strip, and a curve is still a clean
        gap inside one.  A method that fitted the furrow over the whole frame
        would have to lose confidence here, and would then be least sure
        exactly where the rover most needs steering."""
        curved = crop_rows(far_u=self.CENTRE + 20, top_u=self.CENTRE + 90)
        assert an_estimator().estimate(curved).confidence == pytest.approx(1.0)

    def test_a_curve_bending_right_reads_positive(self):
        est = an_estimator().estimate(
            crop_rows(far_u=self.CENTRE + 10, top_u=self.CENTRE + 50)
        )
        assert est.heading_err > 0

    def test_a_curve_bending_left_reads_negative(self):
        est = an_estimator().estimate(
            crop_rows(far_u=self.CENTRE - 10, top_u=self.CENTRE - 50)
        )
        assert est.heading_err < 0

    def test_a_curve_reads_the_same_as_the_line_through_its_two_strips(self):
        """The claim itself: `heading_err` is a difference between two strips,
        so what the furrow does *above* the lookahead cannot reach it.  These
        two frames share near and far and diverge by 25 px at the top of the
        image, and the estimator cannot tell them apart -- which is the whole
        of why no curve has to be fitted and no straight line has to hold.
        """
        curved = crop_rows(far_u=self.CENTRE + 10, top_u=self.CENTRE + 50)
        straight = crop_rows(far_u=self.CENTRE + 10)

        assert an_estimator().estimate(curved).heading_err == pytest.approx(
            an_estimator().estimate(straight).heading_err, abs=0.01
        )

    def test_a_sharper_curve_reads_as_more_heading(self):
        gentle = an_estimator().estimate(
            crop_rows(far_u=self.CENTRE + 10, top_u=self.CENTRE + 50)
        )
        hard = an_estimator().estimate(
            crop_rows(far_u=self.CENTRE + 20, top_u=self.CENTRE + 90)
        )
        assert hard.heading_err > gentle.heading_err


class TestBareSoil:
    def test_bare_soil_is_not_valid(self):
        assert not an_estimator().estimate(bare_soil()).valid

    def test_bare_soil_reports_almost_no_green(self):
        assert an_estimator().estimate(bare_soil()).green_fraction == pytest.approx(0.0)

    def test_bare_soil_has_no_confidence(self):
        assert an_estimator().estimate(bare_soil()).confidence == pytest.approx(0.0)

    @pytest.mark.parametrize("seed", range(4))
    def test_bare_soil_is_not_valid_whatever_the_noise(self, seed):
        assert not an_estimator().estimate(bare_soil(seed)).valid

    def test_an_invalid_estimate_still_reports_errors_in_range(self):
        """`RowEstimate.__post_init__` rejects anything outside [-1, 1], so an
        estimator that leaves a junk number in an invalid estimate raises
        instead of reporting an invalid frame."""
        est = an_estimator().estimate(bare_soil())
        assert -1.0 <= est.lateral_err <= 1.0
        assert -1.0 <= est.heading_err <= 1.0


class TestCropGapMidRow:
    """The case that is the whole reason for a histogram (V1-ISAAC Stage 1)."""

    def test_a_missing_plant_mid_row_stays_valid(self):
        image = crop_rows(missing_left_rows=(170, 200))
        assert an_estimator().estimate(image).valid

    def test_a_missing_plant_mid_row_does_not_move_the_furrow(self):
        """The band is thinner where the plant is gone, but it is still a band,
        and the valley between the two is where it always was."""
        intact = an_estimator().estimate(crop_rows())
        gapped = an_estimator().estimate(crop_rows(missing_left_rows=(170, 200)))
        assert gapped.lateral_err == pytest.approx(intact.lateral_err, abs=0.02)

    def test_a_missing_plant_mid_row_still_reports_green(self):
        """Distinguishing this from the end of the row is `green_fraction`'s
        entire job, and the watchdog reads it."""
        est = an_estimator().estimate(crop_rows(missing_left_rows=(170, 200)))
        trip = get(load_config(), "perception.row_estimator.green_fraction_row_end")
        assert est.green_fraction > trip


class TestBothStripsMustAgree:
    def test_green_on_only_one_side_is_not_a_furrow(self):
        """One crop row and open ground beside it.  There is a deep valley in
        the histogram, but no shoulder on the far side of it."""
        assert not an_estimator().estimate(crop_rows(right_band_only=True)).valid

    def test_a_lookahead_that_cannot_be_seen_invalidates_the_frame(self):
        """The strip under the wheels is a clean furrow; the lookahead strip is
        bare.  Passing on the near strip alone would be a rover trusting a
        lookahead it has not got."""
        image = crop_rows()
        image[: SHAPE[0] * 2 // 3] = bare_soil()[: SHAPE[0] * 2 // 3]
        assert not an_estimator().estimate(image).valid

    def test_confidence_is_the_weaker_of_the_two_strips(self):
        """Weeds across the whole lookahead furrow, none under the wheels.  The
        near strip is still a clean valley; the estimate must not report that."""
        image = crop_rows()
        image[100:150, 110:210] = LEAF  # the lookahead furrow, part-way filled
        est = an_estimator().estimate(image)

        assert an_estimator().estimate(crop_rows()).confidence == pytest.approx(1.0)
        assert 0.0 < est.confidence < 1.0

    def test_a_partly_filled_furrow_is_only_partly_filled_in_confidence(self):
        """Green over 50 of the lookahead strip's 80 rows leaves the valley at
        3/8 of the shoulder — prominence tracks how much of the furrow is gone,
        rather than collapsing the moment anything lands in it."""
        image = crop_rows()
        image[100:150, 110:210] = LEAF
        assert an_estimator().estimate(image).confidence == pytest.approx(0.375, abs=0.02)


class TestGreenFraction:
    def test_it_is_the_share_of_pixels_the_mask_keeps(self):
        from perception.exg import green_mask

        image = crop_rows()
        expected = green_mask(image, exg_floor=EXG_FLOOR).mean()
        assert an_estimator().estimate(image).green_fraction == pytest.approx(expected)

    def test_the_end_of_a_row_reaches_the_trip_with_almost_nothing_to_spare(self):
        """A measurement, reported rather than tuned away (V1-ISAAC rule 7).

        Walk a row end in: crops surviving only in the bottom ``q`` of the
        frame.  The estimate goes invalid at ``q = 0.30``, the moment the
        lookahead strip runs out of crops -- and ``green_fraction`` is 0.113
        there, still *above* the 0.10 trip.  So on the first invalid frame the
        watchdog would read §5.4's table as ROW_LOST, a fault, for what is a
        normal end of a row.

        It has three more frames before it trips, and green_fraction keeps
        falling, so this is not proof the shipped value is wrong -- it is proof
        the margin is 0.013 wide and the verdict turns on how far the rover
        moves in those frames.  Settling it needs rendered frames with real
        foliage, which is `row_end.yaml` paired against `crop_gap_midrow.yaml`
        at V1/V2; `runaway.py` says the same thing from the other end.
        """
        trip = get(load_config(), "perception.row_estimator.green_fraction_row_end")
        height = SHAPE[0]

        for step in range(100, -1, -1):
            image = crop_rows()
            bare_to = int(round(height * (1 - step / 100)))
            image[:bare_to] = bare_soil()[:bare_to]
            est = an_estimator().estimate(image)
            if not est.valid:
                break

        assert trip < est.green_fraction < trip * 1.5

    def test_crops_and_bare_soil_fall_on_opposite_sides_of_the_trip(self):
        """The shipped `green_fraction_row_end` has to separate these two, or
        the end of a row and the loss of a row arrive as the same reading."""
        trip = get(load_config(), "perception.row_estimator.green_fraction_row_end")
        assert an_estimator().estimate(bare_soil()).green_fraction < trip
        assert an_estimator().estimate(crop_rows()).green_fraction > trip


class TestEstimatorFromConfig:
    def test_it_reads_the_shipped_floor_and_confidence(self):
        estimator = RowEstimator.from_config(load_config())
        config = load_config()
        assert estimator.exg_floor == get(config, "perception.exg.exg_floor")
        assert estimator.conf_min == get(config, "perception.row_estimator.conf_min")

    def test_the_shipped_values_accept_a_furrow_and_reject_bare_soil(self):
        estimator = RowEstimator.from_config(load_config())
        assert estimator.estimate(crop_rows()).valid
        assert not estimator.estimate(bare_soil()).valid

    def test_front_downscale_is_width_by_height_and_numpy_shape_is_not(self):
        """`front_downscale` is [320, 240] = (width, height) while the frames it
        describes are numpy arrays of shape (240, 320).  Transposing the two is
        silent: the estimator still returns numbers, and every one of them is
        measured across the wrong axis."""
        width, height = get(load_config(), "perception.row_estimator.front_downscale")
        assert (height, width) == SHAPE


class TestConfidenceThreshold:
    def test_validity_follows_conf_min(self):
        """Proof that `conf_min` is what decides, not the image: the same frame
        either side of a threshold that steps over its measured confidence."""
        image = crop_rows()
        measured = an_estimator().estimate(image).confidence

        assert an_estimator(conf_min=measured * 0.5).estimate(image).valid
        assert not an_estimator(conf_min=min(measured * 1.5, 1.0)).estimate(image).valid

    def test_confidence_stays_within_the_unit_range(self):
        for image in (bare_soil(), crop_rows(), crop_rows(near_u=200, far_u=120)):
            assert 0.0 <= an_estimator().estimate(image).confidence <= 1.0
