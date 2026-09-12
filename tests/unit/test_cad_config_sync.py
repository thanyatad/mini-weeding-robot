"""CAD -> config sync, the test cad/parameters/README.md#sync-enforcement names.

Every value in config/ carrying a ``# derived: cad ...`` comment has its origin
in ``cad/parameters/parameters.csv``.  Nothing regenerates the YAML from the CSV
by choice — a Fusion-API generator costs more to maintain than this POC is
worth — so the marker is only a promise until something checks it.  This is that
something.

If a test here fails it means CAD moved and config has not caught up.  Fix the
config, never the CAD table, and never the other way around: the ownership line
in config/README.md#derived-values is that anything measurable with a caliper
belongs to CAD.
"""

import csv
import math
import re
from pathlib import Path

import pytest

from controller.config import get, load_config

REPO = Path(__file__).resolve().parents[2]
PARAMETERS_CSV = REPO / "cad" / "parameters" / "parameters.csv"
CONFIG_DIR = REPO / "config"
BOM_MD = REPO / "hardware" / "bom" / "poc-v2.md"

#: Every parameter cad/parameters/README.md#parameters lists.  Named here so a
#: parameter quietly dropped from the export is a failure and not a silently
#: skipped check.
EXPECTED_PARAMETERS = (
    # Chassis
    "body_length_mm",
    "chassis_plate_width_mm",
    "body_width_mm",
    "body_height_mm",
    "chassis_clearance_mm",
    # Drive
    "track_width_mm",
    "wheelbase_mm",
    "wheel_diameter_mm",
    "wheel_width_mm",
    "motor_mount_pitch_mm",
    # Camera
    "camera_front_height_mm",
    "camera_front_tilt_deg",
    "camera_front_offset_x_mm",
    "camera_down_height_mm",
    "camera_down_tilt_deg",
    "camera_down_offset_x_mm",
)

#: The markers that are a plain equality: the config value is the CAD value.
#: Keyed by the YAML leaf the marker sits on.
EQUALITIES: dict[str, tuple[str, str]] = {
    # yaml leaf: (dotted config path, cad parameter)
    "track_width_mm": ("rover.track_width_mm", "track_width_mm"),
    "wheelbase_mm": ("rover.wheelbase_mm", "wheelbase_mm"),
    "body_width_mm": ("rover.body_width_mm", "body_width_mm"),
    "wheel_diameter_mm": ("rover.wheel_diameter_mm", "wheel_diameter_mm"),
    "chassis_clearance_mm": ("rover.chassis_clearance_mm", "chassis_clearance_mm"),
}

#: Markers that are a *formula*, not an equality, and so have a test of their
#: own.  `wheel_v_max_mm_s: 340  # derived: cad wheel_diameter + 100 RPM` is the
#: reason this list has to exist: a parser that read every marker as "equals the
#: CAD parameter named here" would assert 340 == 65.
FORMULAS = ("wheel_v_max_mm_s",)

_MARKER = re.compile(
    r"^\s*(?P<key>[A-Za-z_][\w]*)\s*:\s*(?P<value>-?[\d.]+)\s*#\s*derived:\s*cad\s+(?P<source>\S+)"
)


def _cad_parameters() -> dict[str, float]:
    lines = [
        line
        for line in PARAMETERS_CSV.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    return {row["name"]: float(row["value"]) for row in csv.DictReader(lines)}


def _derived_markers() -> dict[str, str]:
    """Every ``# derived: cad`` marker in config/, as {yaml leaf: cad name cited}."""
    markers: dict[str, str] = {}
    for path in sorted(CONFIG_DIR.glob("*.yaml")):
        for line in path.read_text(encoding="utf-8").splitlines():
            found = _MARKER.match(line)
            if found is not None:
                markers[found.group("key")] = found.group("source")
    return markers


def test_the_parameter_table_is_actually_read():
    parameters = _cad_parameters()
    assert set(parameters) == set(EXPECTED_PARAMETERS), (
        "cad/parameters/parameters.csv no longer matches the parameter list in "
        "cad/parameters/README.md#parameters"
    )


def test_every_derived_marker_is_covered_by_a_check():
    """A marker nobody checks is a comment, which is what this file exists to stop."""
    markers = set(_derived_markers())
    checked = set(EQUALITIES) | set(FORMULAS)
    assert markers == checked, (
        f"config carries `# derived: cad` markers this test does not check: "
        f"{sorted(markers - checked)}; and checks markers that no longer exist: "
        f"{sorted(checked - markers)}"
    )


def test_every_marker_cites_a_real_cad_parameter():
    """The marker names its origin without the unit suffix (`cad track_width`).
    A marker pointing at a parameter that was renamed in Fusion is drift too."""
    parameters = _cad_parameters()
    unknown = {
        key: source
        for key, source in _derived_markers().items()
        if not any(name.startswith(source) for name in parameters)
    }
    assert unknown == {}, f"markers cite CAD parameters that do not exist: {unknown}"


@pytest.mark.parametrize("yaml_leaf", sorted(EQUALITIES))
def test_config_value_equals_the_cad_parameter(yaml_leaf):
    config_path, cad_name = EQUALITIES[yaml_leaf]
    cad_value = _cad_parameters()[cad_name]
    config_value = float(get(load_config(), config_path))
    assert cad_value == config_value, (
        f"CAD {cad_name} = {cad_value} but config {config_path} = {config_value} — "
        f"CAD moved and config/rover.yaml has not caught up"
    )


def test_body_width_is_the_widest_point_not_the_chassis_plate():
    """body_width is what hits the crop, and the wheels stand proud of the plate.

    This derivation is not decoration: it is what caught `track_width 140`
    during design, where the overall width came out at 166 mm and left
    invariant 5 a margin of 4 mm.  A test that only compared two numbers
    would have been perfectly green on that value.
    """
    parameters = _cad_parameters()
    expected = max(
        parameters["chassis_plate_width_mm"],
        parameters["track_width_mm"] + parameters["wheel_width_mm"],
    )
    assert parameters["body_width_mm"] == expected, (
        f"CAD body_width_mm = {parameters['body_width_mm']} but "
        f"max(chassis_plate_width {parameters['chassis_plate_width_mm']}, "
        f"track_width {parameters['track_width_mm']} + "
        f"wheel_width {parameters['wheel_width_mm']}) = {expected} — "
        f"body_width must be the overall width including the wheels"
    )


def test_the_bom_still_states_the_motor_speed():
    """wheel_v_max is derived from a number that lives in the BOM, not in CAD.
    If the BOM row is reworded, the derivation below is quietly testing nothing.
    """
    assert _motor_rpm() == 100.0


def _motor_rpm() -> float:
    found = re.search(
        r"DC gear motor 12 V\s*\*\*~(\d+) RPM\*\*", BOM_MD.read_text(encoding="utf-8")
    )
    assert found is not None, f"could not find the motor speed in {BOM_MD}"
    return float(found.group(1))


def test_wheel_v_max_follows_from_wheel_diameter_and_motor_rpm():
    """The one marker that is a formula: (rpm / 60) x pi x d, rounded.

    config/rover.yaml cites `cad wheel_diameter + motor 100 RPM`, and the RPM
    half of that comes from hardware/bom/poc-v2.md, not from CAD.
    """
    wheel_diameter_mm = _cad_parameters()["wheel_diameter_mm"]
    expected = (_motor_rpm() / 60.0) * math.pi * wheel_diameter_mm
    config_value = float(get(load_config(), "rover.drive.wheel_v_max_mm_s"))

    assert config_value == round(expected), (
        f"CAD wheel_diameter {wheel_diameter_mm} mm at {_motor_rpm():.0f} RPM gives "
        f"{expected:.1f} mm/s (rounds to {round(expected)}) but config "
        f"rover.drive.wheel_v_max_mm_s = {config_value}"
    )


def test_the_gain_coupling_is_not_covered_here():
    """cad/parameters/README.md#สิ่งที่-test-จับไม่ได้: camera_front_tilt_deg and
    camera_front_height_mm set the row follower's lookahead, and so its gain.
    There is no formula from geometry to gain — it has to be re-tuned at V1 and
    V4 by hand.  This test asserts only that the two parameters are in the
    table, so the warning has something to point at.  It deliberately does not
    pretend to check the coupling.
    """
    parameters = _cad_parameters()
    assert "camera_front_tilt_deg" in parameters
    assert "camera_front_height_mm" in parameters
