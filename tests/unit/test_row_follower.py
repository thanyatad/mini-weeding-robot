"""RowFollower: RowEstimate -> (v, omega).  Pure function, synthetic numbers,
no camera and no rover.

The sign convention is the load-bearing part.  coordinate-frames.md: +Y is
left, omega > 0 is CCW, and lateral_err > 0 means the furrow is to the right.
If the sign flips anywhere, the feedback becomes positive and the rover leaves
the furrow *faster* than an uncontrolled one would — not slower.
"""

import pytest

from controller.config import load_config
from controller.motion import RowEstimate, RowFollower

K_LAT = 45.0
K_HEAD = 25.0
V_MM_S = 100.0
OMEGA_MAX = 40.0


@pytest.fixture
def follower():
    return RowFollower(k_lat=K_LAT, k_head=K_HEAD, v_mm_s=V_MM_S, omega_max_deg_s=OMEGA_MAX)


def estimate(lateral_err=0.0, heading_err=0.0, **kwargs):
    return RowEstimate(
        valid=kwargs.pop("valid", True),
        lateral_err=lateral_err,
        heading_err=heading_err,
        confidence=kwargs.pop("confidence", 1.0),
        green_fraction=kwargs.pop("green_fraction", 0.4),
    )


class TestSignConvention:
    def test_row_right_of_centre_turns_right(self):
        follower = RowFollower(K_LAT, K_HEAD, V_MM_S, OMEGA_MAX)
        _, omega = follower.step(estimate(lateral_err=+0.5))
        assert omega < 0

    def test_row_left_of_centre_turns_left(self, follower):
        _, omega = follower.step(estimate(lateral_err=-0.5))
        assert omega > 0

    def test_row_tilting_right_turns_right(self, follower):
        _, omega = follower.step(estimate(heading_err=+0.5))
        assert omega < 0

    def test_centred_and_aligned_drives_straight(self, follower):
        v, omega = follower.step(estimate())
        assert omega == 0.0
        assert v == V_MM_S


class TestFormula:
    def test_matches_the_documented_formula(self, follower):
        _, omega = follower.step(estimate(lateral_err=0.2, heading_err=-0.1))
        assert omega == pytest.approx(-(K_LAT * 0.2 + K_HEAD * -0.1))

    def test_speed_is_constant_in_the_mvp(self, follower):
        for lateral_err in (-1.0, -0.3, 0.0, 0.3, 1.0):
            v, _ = follower.step(estimate(lateral_err=lateral_err))
            assert v == V_MM_S

    def test_heading_term_damps_a_lateral_correction(self):
        """k_head is the damping term: heading_err is already the rate of change
        of lateral_err in geometry, so no derivative of a noisy image signal is
        needed — and no integral, whose windup is not worth a standing offset
        inside a furrow."""
        follower = RowFollower(K_LAT, K_HEAD, V_MM_S, OMEGA_MAX)
        _, undamped = follower.step(estimate(lateral_err=0.4, heading_err=0.0))
        _, damped = follower.step(estimate(lateral_err=0.4, heading_err=-0.2))
        assert abs(damped) < abs(undamped)


class TestClamping:
    @pytest.mark.parametrize("lateral_err", (1.0, 0.95))  # 45 x 0.95 = 42.75 > 40
    def test_omega_is_clamped_to_the_configured_maximum(self, follower, lateral_err):
        _, omega = follower.step(estimate(lateral_err=lateral_err))
        assert omega == pytest.approx(-OMEGA_MAX)

    def test_omega_is_clamped_symmetrically(self, follower):
        _, omega = follower.step(estimate(lateral_err=-1.0))
        assert omega == pytest.approx(+OMEGA_MAX)


class TestRowEstimate:
    def test_it_carries_no_millimetres_and_no_degrees(self):
        """The front camera is not calibrated.  What it yields is a pixel ratio;
        declaring it in mm or deg would claim a unit nobody can back up — which
        is why the gains have units of (deg/s) / unitless and must be tuned."""
        fields = set(RowEstimate.__dataclass_fields__)
        assert fields == {"valid", "lateral_err", "heading_err", "confidence", "green_fraction"}
        assert not [f for f in fields if f.endswith(("_mm", "_deg", "_deg_s"))]

    def test_it_is_frozen(self):
        est = estimate()
        with pytest.raises(Exception):
            est.lateral_err = 0.5  # type: ignore[misc]

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"lateral_err": 1.5},
            {"lateral_err": -1.5},
            {"heading_err": 2.0},
            {"confidence": 1.2},
            {"green_fraction": -0.1},
        ],
    )
    def test_out_of_range_values_are_rejected(self, kwargs):
        with pytest.raises(ValueError):
            estimate(**kwargs)


class TestFromConfig:
    def test_gains_come_from_the_named_backend(self):
        follower = RowFollower.from_config(load_config(), backend="fake")
        assert (follower.k_lat, follower.k_head) == (45.0, 25.0)
        assert follower.v_mm_s == 100.0
        assert follower.omega_max_deg_s == 40.0

    def test_untuned_gains_are_refused_rather_than_borrowed_from_sim(self):
        """Slip on real soil is not slip in sim.  Letting the sim gains stand in
        on hardware is a rover that weaves into the crops with nobody able to
        say why."""
        with pytest.raises(ValueError, match="esp32"):
            RowFollower.from_config(load_config(), backend="esp32")
