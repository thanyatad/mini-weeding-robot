"""Skid-steer mixing against config/drive_mixing_vectors.csv.

The same table is the contract for firmware/esp32/test/test_mixing.cpp.  If one
side fails, the two implementations have drifted apart.  Never edit the table
to make a test pass — fix the implementation.
"""

import csv
import math

import pytest

from controller.config import CONFIG_DIR
from controller.rover.mixing import mix

# The header block of the CSV states the parameters the table was generated
# against.  They are repeated here so a change to either one fails loudly
# instead of silently invalidating every row.
TRACK_WIDTH_MM = 120.0
WHEEL_V_MAX_MM_S = 340.0
TOLERANCE_MM_S = 0.01

VECTORS_CSV = CONFIG_DIR / "drive_mixing_vectors.csv"


def _golden_vectors() -> list[tuple[float, float, float, float]]:
    lines = [
        line
        for line in VECTORS_CSV.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    return [
        (
            float(r["v_mm_s"]),
            float(r["omega_deg_s"]),
            float(r["v_left_mm_s"]),
            float(r["v_right_mm_s"]),
        )
        for r in csv.DictReader(lines)
    ]


GOLDEN = _golden_vectors()


def test_the_table_is_actually_read():
    assert len(GOLDEN) == 12


@pytest.mark.parametrize("v, omega, expect_left, expect_right", GOLDEN)
def test_golden_vector(v, omega, expect_left, expect_right):
    wheels = mix(v, omega, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)
    assert wheels.left_mm_s == pytest.approx(expect_left, abs=TOLERANCE_MM_S)
    assert wheels.right_mm_s == pytest.approx(expect_right, abs=TOLERANCE_MM_S)


def test_the_table_still_matches_the_config_it_was_generated_against():
    """If track_width or wheel_v_max moves, the whole table must be regenerated."""
    from controller.config import get, load_config

    config = load_config()
    assert get(config, "rover.track_width_mm") == TRACK_WIDTH_MM
    assert get(config, "rover.drive.wheel_v_max_mm_s") == WHEEL_V_MAX_MM_S


def test_positive_omega_turns_left():
    """coordinate-frames.md: omega > 0 is CCW with z up, so the right wheel runs
    faster.  A flipped sign here turns the row follower into positive feedback."""
    wheels = mix(100.0, 20.0, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)
    assert wheels.right_mm_s > wheels.left_mm_s


def test_saturation_scales_both_sides_instead_of_clipping_one():
    """The trap: clipping only the fast wheel silently changes the turn rate.

    Scaling both sides keeps v_right / v_left — and therefore the path the rover
    actually drives — even though it drives it slower.
    """
    v, omega = 320.0, 40.0
    unsaturated = mix(v, omega, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=1e9)
    saturated = mix(v, omega, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)

    assert saturated.right_mm_s == pytest.approx(WHEEL_V_MAX_MM_S, abs=TOLERANCE_MM_S)
    # A clip would have left the inner wheel at its unsaturated value.
    assert saturated.left_mm_s < unsaturated.left_mm_s
    # Both sides moved by exactly the same factor.
    assert saturated.left_mm_s / unsaturated.left_mm_s == pytest.approx(
        saturated.right_mm_s / unsaturated.right_mm_s
    )


def test_saturation_is_symmetric_in_reverse():
    forward = mix(320.0, 40.0, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)
    backward = mix(-320.0, -40.0, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)
    assert backward.left_mm_s == pytest.approx(-forward.left_mm_s)
    assert backward.right_mm_s == pytest.approx(-forward.right_mm_s)


def test_spin_in_place_needs_no_forward_speed():
    wheels = mix(0.0, 40.0, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=WHEEL_V_MAX_MM_S)
    expected = math.radians(40.0) * TRACK_WIDTH_MM / 2
    assert wheels.left_mm_s == pytest.approx(-expected)
    assert wheels.right_mm_s == pytest.approx(+expected)


def test_a_negative_wheel_limit_is_rejected():
    with pytest.raises(ValueError):
        mix(100.0, 0.0, track_width_mm=TRACK_WIDTH_MM, wheel_v_max_mm_s=-1.0)
