"""rover.usd is the committed override layer.  This is what keeps it honest.

cad/README.md#two-usd-layers is the most important rule in the simulation half
of this repo: ``rover_base.usd`` is regenerated from the URDF whenever CAD
changes and everything written into it is destroyed, so every value that must
survive - drive gains, damping, friction, contact offsets, solver settings -
belongs in ``rover.usd``.

Three things can go wrong and none of them raise:

1. A number is changed in config.yaml and not in rover.usd.  config.yaml is the
   readable file, so that is the one a person edits; PhysX reads the other one.
2. rover.usd stops layering over rover_base.usd, in which case the rover loads
   with no drive at all and simply never moves.
3. rover.usd starts restating geometry.  An override here wins over CAD
   silently, and tests/unit/test_urdf_matches_cad.py would keep passing while
   the simulated rover stopped matching the drawing.

No Isaac needed: rover.usd is ASCII usda and is read as text.
"""

import csv
import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
ROVER_DIR = REPO / "sim" / "isaac" / "robots" / "rover"
ROVER_USD = ROVER_DIR / "rover.usd"
CONFIG_YAML = ROVER_DIR / "config.yaml"
PARAMETERS_CSV = REPO / "cad" / "parameters" / "parameters.csv"

WHEELS = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")


def _wheel_radius_m() -> float:
    """Wheel radius in metres, read from the CAD table the same way
    tests/unit/test_urdf_matches_cad.py does -- so this stays tied to the
    current wheel instead of a number frozen at whatever it used to be."""
    lines = [
        line
        for line in PARAMETERS_CSV.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    cad = {row["name"]: float(row["value"]) for row in csv.DictReader(lines)}
    return cad["wheel_diameter_mm"] / 2 / 1000.0


@pytest.fixture(scope="module")
def usd() -> str:
    return ROVER_USD.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def physics() -> dict:
    return yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))


def values(usd: str, attribute: str) -> list[float]:
    """Every authored value of ``attribute``, in file order."""
    pattern = rf"^\s*\w+ {re.escape(attribute)} = (-?[\d.]+)\s*$"
    return [float(m) for m in re.findall(pattern, usd, re.MULTILINE)]


# -- the layer itself ------------------------------------------------------


def test_the_override_layers_over_the_generated_base(usd):
    """Without this sublayer the rover loads with no drive, no damping and no
    friction - and looks like it loaded correctly."""
    assert re.search(r"subLayers\s*=\s*\[\s*@\./rover_base\.usd@\s*\]", usd)


def test_the_generated_base_is_not_committed():
    """rover_base.usd is disposable.  If it ever gets tracked, someone will
    edit it, and their edit dies at the next import."""
    tracked = subprocess.run(
        ["git", "ls-files", "sim/isaac/robots/rover"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    assert "sim/isaac/robots/rover/rover_base.usd" not in tracked
    assert "sim/isaac/robots/rover/rover.usd" in tracked, "the override layer must be committed"
    assert not [path for path in tracked if path.startswith("sim/isaac/robots/rover/payloads")]


def test_the_override_restates_no_geometry(usd):
    """Mass, inertia, joint origins, joint axes and collision shapes belong to
    the URDF, which is held to cad/parameters/parameters.csv.  Overriding one
    here would beat CAD silently."""
    forbidden = (
        "physics:localPos",
        "physics:localRot",
        "physics:axis",
        "physics:mass",
        "physics:density",
        "physics:diagonalInertia",
        "physics:centerOfMass",
        "physics:lowerLimit",
        "physics:upperLimit",
    )
    offenders = [
        f"{n}: {line.strip()}"
        for n, line in enumerate(usd.splitlines(), 1)
        if not line.lstrip().startswith("#")
        for token in forbidden
        if token in line
    ]
    assert offenders == [], f"rover.usd is overriding CAD-owned geometry: {offenders}"


# -- the drive -------------------------------------------------------------


def test_a_velocity_drive_has_no_stiffness(usd):
    """The trap this whole file exists for.  A velocity-driven joint with any
    stiffness behaves like a position drive: the wheel fights its own target
    instead of holding it, and nothing reports an error."""
    stiffness = values(usd, "drive:angular:physics:stiffness")
    assert len(stiffness) == len(WHEELS)
    assert stiffness == [0.0] * len(WHEELS)


def test_every_wheel_gets_the_damping_config_yaml_declares(usd, physics):
    damping = values(usd, "drive:angular:physics:damping")
    expected = physics["wheel_drive"]["damping_n_m_s_per_deg"]
    assert damping == [expected] * len(WHEELS)
    assert expected > 0.0, "a velocity drive with no damping produces no torque at all"


def test_the_drive_starts_at_rest(usd):
    """A stage that loads already commanding a velocity is a rover that drives
    away the moment someone opens it in the GUI."""
    assert values(usd, "drive:angular:physics:targetVelocity") == [0.0] * len(WHEELS)


def test_the_drive_type_is_authored_for_every_wheel(usd):
    assert len(re.findall(r'drive:angular:physics:type = "force"', usd)) == len(WHEELS)


def test_max_force_is_left_to_the_urdf(usd):
    """It is the gearbox's PEAK/stall torque limit and it comes from
    <limit effort=...>.  Continuous is 5 N.m per wheel (hardware/bom/poc-v3.md)
    and is what the 40 kg mass ceiling rests on.  Restating it here would put
    a BOM number in two places, which is the drift the two-layer split exists
    to prevent."""
    assert "drive:angular:physics:maxForce" not in usd


# -- friction and contact --------------------------------------------------


def test_friction_matches_config_yaml(usd, physics):
    friction = physics["friction"]
    assert values(usd, "physics:staticFriction") == [friction["static"]]
    assert values(usd, "physics:dynamicFriction") == [friction["dynamic"]]
    assert values(usd, "physics:restitution") == [friction["restitution"]]


def test_the_wheel_material_is_the_only_friction_knob(usd, physics):
    """ "min" keeps the answer in this file rather than splitting it with
    whatever the ground happens to be - and Stage 4 replaces the ground."""
    mode = physics["friction"]["combine_mode"]
    assert f'physxMaterial:frictionCombineMode = "{mode}"' in usd


def test_all_four_wheels_are_bound_to_that_material(usd):
    bindings = re.findall(
        r"rel material:binding:physics = </weeding_rover/Physics/Materials/wheel_ground>", usd
    )
    assert len(bindings) == len(WHEELS)


def test_contact_offsets_match_config_yaml(usd, physics):
    collision = physics["collision"]
    assert values(usd, "physxCollision:contactOffset") == [collision["contact_offset_m"]] * len(
        WHEELS
    )
    assert values(usd, "physxCollision:restOffset") == [collision["rest_offset_m"]] * len(WHEELS)


def test_the_contact_offset_is_small_against_the_wheel(physics):
    """Isaac's default contact offset is 0.02 m.  sim/isaac/robots/rover/
    config.yaml:74-81 spells out what that is against this wheel: 16% of the
    current 0.125 m radius (down from 57% of the old 0.035 m one, still large
    enough to want overriding).  The configured 0.004 m is held at its
    absolute value rather than as a ratio -- 11% of the old radius, 3.2% of
    this one -- and this test is the check that it stays a small fraction of
    whatever the wheel currently is, read from the CAD table rather than
    hardcoded."""
    assert physics["collision"]["contact_offset_m"] < _wheel_radius_m() / 2


# -- solver and sleep ------------------------------------------------------


def test_solver_iterations_match_config_yaml(usd, physics):
    solver = physics["solver"]
    assert values(usd, "physxArticulation:solverPositionIterationCount") == [
        solver["position_iterations"]
    ]
    assert values(usd, "physxArticulation:solverVelocityIterationCount") == [
        solver["velocity_iterations"]
    ]


def test_velocity_iterations_stay_inside_what_tgs_wants(physics):
    """PhysX warns that more than 4 velocity iterations on a TGS scene changed
    behaviour, and warns exactly once, so it is easy to miss."""
    assert physics["solver"]["velocity_iterations"] <= 4


def test_sleeping_is_switched_off(usd, physics):
    """A parked articulation accepts a velocity target and silently ignores it.
    The wheels then read back as exactly 0.0 rad/s, which is indistinguishable
    from a drive that does not work - and it bites hardest when turning on the
    spot, where the body barely moves."""
    assert values(usd, "physxRigidBody:sleepThreshold") == [physics["sleep"]["threshold"]]
    assert values(usd, "physxRigidBody:stabilizationThreshold") == [
        physics["sleep"]["stabilization_threshold"]
    ]
    assert physics["sleep"]["threshold"] == 0.0


# -- the mirror ------------------------------------------------------------


def test_config_yaml_is_not_loaded_by_the_controller():
    """These are simulator-side physics values with no counterpart on the real
    rover.  They are not part of config/, they have no startup invariant, and
    controller/ must not read them."""
    offenders = [
        f"{path.relative_to(REPO)}:{n}"
        for path in sorted((REPO / "controller").rglob("*.py"))
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if "robots/rover/config.yaml" in line or "rover_physics" in line
    ]
    assert offenders == [], f"controller/ reached into the simulator's physics config: {offenders}"
