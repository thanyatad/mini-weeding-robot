"""CAD -> URDF sync, the second of the two unit-conversion points.

hardware/mechanical/coordinate-frames.md#unit-convention names exactly two
places where units change, and says both must have a unit test:

    cad/urdf              <-> config     mm <-> m,  deg <-> rad
    sim/isaac/adapters/   <-> Rover      mm <-> m,  deg/s <-> rad/s

``test_cad_config_sync.py`` covers ``cad/parameters`` -> ``config``.  Nothing
covered ``cad/parameters`` -> ``cad/urdf`` until this file, even though
cad/parameters/README.md#derived-values lists three entries that flow that way:
the wheel joint origin in X and in Y, and the wheel radius.

A unit slip here does not crash.  The rover simply drives at a thousand times
the wrong scale, which reads as a physics or tuning problem rather than an
arithmetic one.  That is the failure this file exists to make loud.
"""

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from controller.config import get, load_config

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "cad" / "urdf" / "weeding_rover.urdf"
PARAMETERS_CSV = REPO / "cad" / "parameters" / "parameters.csv"

#: The star topology cad/urdf/README.md#kinematic-chain describes: four wheels
#: and two cameras, every one of them a child of base_link.
WHEELS = {
    "wheel_fl": (+1, +1),
    "wheel_fr": (+1, -1),
    "wheel_rl": (-1, +1),
    "wheel_rr": (-1, -1),
}

MM_PER_M = 1000.0


def _cad() -> dict[str, float]:
    lines = [
        line
        for line in PARAMETERS_CSV.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    return {row["name"]: float(row["value"]) for row in csv.DictReader(lines)}


def _robot() -> ET.Element:
    return ET.parse(URDF).getroot()


def _joint(name: str) -> ET.Element:
    found = [j for j in _robot().findall("joint") if j.get("name") == name]
    assert found, f"{URDF} has no joint named {name}"
    return found[0]


def _link(name: str) -> ET.Element:
    found = [ln for ln in _robot().findall("link") if ln.get("name") == name]
    assert found, f"{URDF} has no link named {name}"
    return found[0]


def _xyz(element: ET.Element) -> tuple[float, float, float]:
    x, y, z = (float(v) for v in element.get("xyz").split())
    return x, y, z


def _rpy(element: ET.Element) -> tuple[float, float, float]:
    roll, pitch, yaw = (float(v) for v in (element.get("rpy") or "0 0 0").split())
    return roll, pitch, yaw


def _optical_axis(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """The camera's +Z, the direction out of the lens, expressed in base_link.

    URDF rpy is the fixed-axis convention R = Rz(yaw) Ry(pitch) Rx(roll), so
    this is that matrix's third column.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)


def test_the_kinematic_chain_is_a_star_off_base_link():
    robot = _robot()
    links = {link.get("name") for link in robot.findall("link")}
    assert links == {
        "base_link",
        "wheel_fl_link",
        "wheel_fr_link",
        "wheel_rl_link",
        "wheel_rr_link",
        "camera_front_link",
        "camera_down_link",
    }
    for joint in robot.findall("joint"):
        assert joint.find("parent").get("link") == "base_link", (
            f"{joint.get('name')} is not a child of base_link - "
            "cad/urdf/README.md#kinematic-chain requires a star, not a chain"
        )


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_wheel_joints_are_continuous_not_revolute(name):
    """A revolute wheel needs limits, and Isaac stops the wheel at them.

    The symptom is a rover that drives a little way and then halts with no
    error, which gets misread as a friction or command-timeout problem.
    """
    assert _joint(name).get("type") == "continuous"


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_wheel_joint_origin_is_the_cad_track_and_wheelbase_in_metres(name):
    cad = _cad()
    sign_x, sign_y = WHEELS[name]
    x, y, _ = _xyz(_joint(name).find("origin"))
    assert x == pytest.approx(sign_x * cad["wheelbase_mm"] / 2 / MM_PER_M, abs=1e-9)
    assert y == pytest.approx(sign_y * cad["track_width_mm"] / 2 / MM_PER_M, abs=1e-9)


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_wheel_axle_sits_one_wheel_radius_off_the_ground(name):
    """base_link is on the soil reference plane, so the axle height is the radius.

    hardware/mechanical/coordinate-frames.md puts the rover frame origin at the
    centre of the vehicle at soil reference plane level, and the wheels rest on
    that plane, so z is wheel_diameter/2 exactly.
    """
    cad = _cad()
    _, _, z = _xyz(_joint(name).find("origin"))
    assert z == pytest.approx(cad["wheel_diameter_mm"] / 2 / MM_PER_M, abs=1e-9)


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_every_wheel_turns_about_plus_y(name):
    """omega > 0 is CCW with z up, and that has to agree in all three places.

    A flipped sign here does not merely slow the row follower down: the
    feedback becomes positive, and the rover leaves the furrow faster than it
    would with no steering at all.
    """
    assert _xyz(_joint(name).find("axis")) == (0.0, 1.0, 0.0)


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_wheel_velocity_limit_follows_wheel_v_max(name):
    """If the URDF caps lower than the mixer commands, Isaac clips it silently.

    The result looks exactly like real motor deadband, but in simulation.
    """
    cad = _cad()
    wheel_v_max = float(get(load_config(), "rover.drive.wheel_v_max_mm_s"))
    expected = wheel_v_max / (cad["wheel_diameter_mm"] / 2)
    limit = float(_joint(name).find("limit").get("velocity"))
    assert limit == pytest.approx(expected, rel=1e-4)


@pytest.mark.parametrize("name", sorted(WHEELS))
def test_wheel_collision_is_a_cylinder_sized_from_cad(name):
    """cad/urdf/README.md#collision: the wheel must be a primitive, not a mesh.

    A tread mesh on a heightfield generates a very large number of contact
    points, which costs solver time and stability and buys nothing, because
    grip comes from the friction coefficient in rover.usd.
    """
    cad = _cad()
    collisions = _link(f"{name}_link").findall("collision")
    assert len(collisions) == 1
    geometry = collisions[0].find("geometry")
    assert geometry.find("mesh") is None, "wheel collision must not be a mesh"
    cylinder = geometry.find("cylinder")
    assert cylinder is not None
    assert float(cylinder.get("radius")) == pytest.approx(
        cad["wheel_diameter_mm"] / 2 / MM_PER_M, abs=1e-9
    )
    assert float(cylinder.get("length")) == pytest.approx(
        cad["wheel_width_mm"] / MM_PER_M, abs=1e-9
    )


def test_base_link_collision_is_a_box():
    geometry = _link("base_link").find("collision").find("geometry")
    assert geometry.find("box") is not None
    assert geometry.find("mesh") is None


@pytest.mark.parametrize("name", ["camera_front", "camera_down"])
def test_camera_joints_are_fixed(name):
    assert _joint(name).get("type") == "fixed"


def test_camera_origins_are_the_cad_offsets_in_metres():
    cad = _cad()
    x, y, z = _xyz(_joint("camera_front").find("origin"))
    assert x == pytest.approx(cad["camera_front_offset_x_mm"] / MM_PER_M, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(cad["camera_front_height_mm"] / MM_PER_M, abs=1e-9)

    x, y, z = _xyz(_joint("camera_down").find("origin"))
    assert x == pytest.approx(cad["camera_down_offset_x_mm"] / MM_PER_M, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(cad["camera_down_height_mm"] / MM_PER_M, abs=1e-9)


def test_front_camera_looks_forward_and_down_at_the_cad_tilt():
    """camera_front_tilt_deg is measured down from horizontal.

    This angle and camera_front_height_mm set the row follower's lookahead, and
    so its gain.  There is no formula from geometry to gain, so this test can
    only hold the URDF to the CAD number - re-tuning after a change is still a
    human step.  See cad/parameters/README.md.
    """
    cad = _cad()
    axis_x, axis_y, axis_z = _optical_axis(*_rpy(_joint("camera_front").find("origin")))
    assert axis_x > 0, "the front camera must look forward"
    assert axis_y == pytest.approx(0.0, abs=1e-9)
    below_horizontal = math.degrees(math.atan2(-axis_z, math.hypot(axis_x, axis_y)))
    assert below_horizontal == pytest.approx(cad["camera_front_tilt_deg"], abs=1e-6)


def test_down_camera_looks_straight_down():
    """camera_down_tilt_deg uses the other reference: 0 means straight down.

    The two camera tilt parameters do not share a convention, which is exactly
    the sort of thing that gets copied across wrongly - camera_front_tilt_deg
    is measured from horizontal, camera_down_tilt_deg from vertical.
    """
    cad = _cad()
    axis = _optical_axis(*_rpy(_joint("camera_down").find("origin")))
    off_vertical = math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))
    assert off_vertical == pytest.approx(cad["camera_down_tilt_deg"], abs=1e-6)


def test_every_mesh_reference_resolves():
    """A wrong mesh path or origin is the most common mistake at this stage.

    cad/exports/README.md#export-checklist says so outright.  In Isaac the
    symptom is a part floating, or spinning about the wrong point; a missing
    file at least fails loudly instead.
    """
    missing = [
        mesh.get("filename")
        for mesh in _robot().iter("mesh")
        if not (URDF.parent / mesh.get("filename")).is_file()
    ]
    assert missing == []


def test_total_mass_is_under_the_gearbox_ceiling():
    """2.3 kg comes from the 5 kg.cm continuous limit of the 250:1 gearbox
    climbing a 15 mm clod on a 35 mm wheel radius - hardware/bom/poc-v2.md."""
    total = sum(
        float(link.find("inertial").find("mass").get("value"))
        for link in _robot().findall("link")
        if link.find("inertial") is not None
    )
    assert total <= 2.3, f"URDF total mass {total:.3f} kg exceeds the 2.3 kg ceiling"
