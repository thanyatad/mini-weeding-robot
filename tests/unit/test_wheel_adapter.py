"""The adapter boundary: (v, omega) in rover units -> wheel joint velocity in Isaac units.

hardware/mechanical/coordinate-frames.md#unit-convention names exactly two
places where units change and says both must have a unit test.  This is the
second one:

    cad/urdf              <-> config     mm <-> m,  deg <-> rad
    sim/isaac/adapters/   <-> Rover      mm <-> m,  deg/s <-> rad/s

The adapter must not contain the skid-steer formula.  It is already written
twice - controller/rover/mixing.py and firmware/esp32 - and both are pinned to
config/drive_mixing_vectors.csv.  A third copy drifts, so the tests below hold
the adapter to that same table rather than to numbers typed in here.
"""

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from controller.config import CONFIG_DIR, get, load_config
from sim.isaac.adapters.wheel_adapter import WheelAdapter

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "cad" / "urdf" / "weeding_rover.urdf"
VECTORS_CSV = CONFIG_DIR / "drive_mixing_vectors.csv"

TRACK_WIDTH_MM = 430.0
WHEEL_V_MAX_MM_S = 327.0
WHEEL_RADIUS_MM = 125.0

#: config/drive_mixing_vectors.csv carries three decimals, so a row is good to
#: 0.001 mm/s.  tests/unit/test_drive_mixing.py allows 0.01 mm/s against the same
#: table; the same allowance is used here, carried across the unit change.
TOLERANCE_MM_S = 0.01
TOLERANCE_RAD_S = TOLERANCE_MM_S / WHEEL_RADIUS_MM


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


def adapter() -> WheelAdapter:
    return WheelAdapter(
        track_width_mm=TRACK_WIDTH_MM,
        wheel_v_max_mm_s=WHEEL_V_MAX_MM_S,
        wheel_radius_mm=WHEEL_RADIUS_MM,
    )


# -- the conversion itself -------------------------------------------------


def test_driving_straight_turns_both_sides_at_the_same_rate():
    sides = adapter().wheel_velocities(100.0, 0.0)
    assert sides.left_rad_s == pytest.approx(100.0 / WHEEL_RADIUS_MM)
    assert sides.right_rad_s == pytest.approx(sides.left_rad_s)


@pytest.mark.parametrize("v, omega, expect_left, expect_right", GOLDEN)
def test_every_golden_vector_converts_to_rad_s_by_the_wheel_radius(
    v, omega, expect_left, expect_right
):
    """Holds the adapter to config/drive_mixing_vectors.csv, the same table the
    firmware is held to.  If the adapter grew its own copy of the formula, the
    saturating rows here are where it would part company."""
    sides = adapter().wheel_velocities(v, omega)
    assert sides.left_rad_s == pytest.approx(expect_left / WHEEL_RADIUS_MM, abs=TOLERANCE_RAD_S)
    assert sides.right_rad_s == pytest.approx(expect_right / WHEEL_RADIUS_MM, abs=TOLERANCE_RAD_S)


def test_a_positive_omega_runs_the_right_side_faster():
    """coordinate-frames.md: omega > 0 is CCW with Z up, which is a left turn.

    Check the sign against v_right > v_left, not against intuition.  Flipped,
    the row follower's feedback becomes positive and the rover leaves the
    furrow faster than it would with no steering at all.
    """
    sides = adapter().wheel_velocities(100.0, 40.0)
    assert sides.right_rad_s > sides.left_rad_s


def test_turning_on_the_spot_runs_the_sides_in_opposite_directions():
    sides = adapter().wheel_velocities(0.0, 40.0)
    assert sides.left_rad_s < 0.0 < sides.right_rad_s
    assert sides.left_rad_s == pytest.approx(-sides.right_rad_s)


def test_saturation_scales_both_sides_rather_than_clipping_one():
    """drive(320, 40) asks for more than the wheels have.

    Clipping the fast side alone would change the turn the rover actually
    drives, and nothing in the system could notice.  Scaling gives up speed
    instead, which the row follower sees and corrects for.
    """
    bias_mm_s = math.radians(40.0) * TRACK_WIDTH_MM / 2.0
    unclipped_left, unclipped_right = 320.0 - bias_mm_s, 320.0 + bias_mm_s
    scale = WHEEL_V_MAX_MM_S / unclipped_right

    sides = adapter().wheel_velocities(320.0, 40.0)

    assert sides.right_rad_s == pytest.approx(unclipped_right * scale / WHEEL_RADIUS_MM)
    assert sides.left_rad_s == pytest.approx(unclipped_left * scale / WHEEL_RADIUS_MM)
    # A clip would have left the slow side exactly where it was.
    assert sides.left_rad_s < unclipped_left / WHEEL_RADIUS_MM


def _urdf_velocity_limit() -> float:
    return float(
        next(
            j.find("limit").get("velocity")
            for j in ET.parse(URDF).getroot().findall("joint")
            if j.get("name") == "wheel_fl"
        )
    )


def test_the_urdf_velocity_limit_is_the_wheel_limit_carried_across_the_units():
    """2.616 rad/s is 327 mm/s / 125 mm, written to four decimals."""
    assert _urdf_velocity_limit() == pytest.approx(round(WHEEL_V_MAX_MM_S / WHEEL_RADIUS_MM, 4))


@pytest.mark.parametrize("v, omega, _left, _right", GOLDEN)
def test_no_vector_exceeds_the_velocity_limit_the_urdf_declares(v, omega, _left, _right):
    """A joint target above the URDF limit is refused by Isaac silently, and the
    omega achieved stops matching the omega commanded.

    The allowance is the URDF's own rounding: 327 / 125 is 2.616 exactly and the
    file may round it, so a fully saturated command may sit a small amount above
    the declared limit - the rounding allowance of 0.5e-4 rad/s covers any
    floating-point representation differences.
    """
    rounding = 0.5e-4
    sides = adapter().wheel_velocities(v, omega)
    assert abs(sides.left_rad_s) <= _urdf_velocity_limit() + rounding
    assert abs(sides.right_rad_s) <= _urdf_velocity_limit() + rounding


# -- the unit boundary -----------------------------------------------------


def test_the_adapter_radius_is_the_urdf_collision_cylinder_in_metres():
    """The one place mm and m meet on this side of the boundary.

    The visual mesh is 0.0345 m after vertex clustering; collision is 0.035 m
    and collision is what physics uses, so that is the radius the adapter must
    divide by.  A slip here does not crash - the rover just drives at the wrong
    scale, which reads as a tuning problem rather than an arithmetic one.
    """
    radius_m = float(
        next(
            ln.find("collision/geometry/cylinder").get("radius")
            for ln in ET.parse(URDF).getroot().findall("link")
            if ln.get("name") == "wheel_fl_link"
        )
    )
    assert WheelAdapter.from_config(load_config()).wheel_radius_mm == pytest.approx(
        radius_m * 1000.0
    )


def test_from_config_reads_the_geometry_rather_than_repeating_it():
    config = load_config()
    built = WheelAdapter.from_config(config)
    assert built.track_width_mm == get(config, "rover.track_width_mm")
    assert built.wheel_v_max_mm_s == get(config, "rover.drive.wheel_v_max_mm_s")
    assert built.wheel_radius_mm == get(config, "rover.wheel_diameter_mm") / 2.0


def test_degrees_per_second_become_radians_per_second():
    """omega_max 40 deg/s is 0.698 rad/s.  At v = 0 the side speed is
    omega_rad * track/2, so the conversion is visible in the answer."""
    sides = adapter().wheel_velocities(0.0, 40.0)
    expected_mm_s = math.radians(40.0) * TRACK_WIDTH_MM / 2.0
    assert sides.right_rad_s == pytest.approx(expected_mm_s / WHEEL_RADIUS_MM)
