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
from tools.generate_sim_meshes import BODY_SHELL_WIDTH_MM, mass_properties

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


def test_base_link_collision_is_boxes_that_clear_the_wheels():
    """Two boxes and a cylinder -- not one box -- and none may be a mesh.

    The frame necks in below the wheel tops -- chassis_plate_width is the
    wheel inner face spacing -- and the body above them overhangs.  The mast
    is a separate thin tube rising above the body, carrying both cameras.  One
    box cannot describe that shape without either clipping the wheels or
    throwing away the overhang, and the mast is not box-shaped at all, so the
    collision is the two boxes plus the cylinder the rover actually has.

    The overhang is not decoration -- it is the entire reason this is two
    boxes rather than one.  A test that only counted primitives and checked
    the lower box's clearance would let a future edit widen or shrink the
    upper box to the wheel-clearing width and still pass, silently flattening
    the body back down to the frame width.  So the box above the wheel tops is
    asserted equal to BODY_SHELL_WIDTH_MM (tools/generate_sim_meshes.py) --
    the source that constant's own docstring claims the URDF's literal
    tracks -- not merely wider than the wheel inner faces, which a stale
    literal could still satisfy by accident.  The mast cylinder gets the same
    treatment as test_wheel_collision_is_a_cylinder_sized_from_cad: its
    radius, length, and height are checked against CAD, not just its shape
    and count.  Both boxes' length (the dimension along the direction of
    travel, discarded by earlier versions of this test) is checked against
    body_length_mm too.

    cad/urdf/README.md#collision still holds: primitives only.
    """
    cad = _cad()
    collisions = _link("base_link").findall("collision")
    assert len(collisions) == 3, f"expected 3 base_link collisions, found {len(collisions)}"

    boxes = []
    cylinders = []
    for collision in collisions:
        geometry = collision.find("geometry")
        assert geometry.find("mesh") is None, "base_link collision must not be a mesh"
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        assert (box is not None) != (cylinder is not None), (
            "each collision must be exactly one of box or cylinder"
        )
        _, _, z = _xyz(collision.find("origin"))
        if box is not None:
            length, width, height = (float(v) for v in box.get("size").split())
            boxes.append((z - height / 2, z + height / 2, width, length))
        else:
            cylinders.append((cylinder, z))

    assert len(boxes) == 2, f"expected 2 box collisions, found {len(boxes)}"
    assert len(cylinders) == 1, f"expected 1 cylinder collision (the mast), found {len(cylinders)}"

    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    inner_faces = (cad["track_width_mm"] - cad["wheel_width_mm"]) / MM_PER_M
    expected_length = cad["body_length_mm"] / MM_PER_M
    shell_width = BODY_SHELL_WIDTH_MM / MM_PER_M
    for bottom, _top, width, length in boxes:
        assert length == pytest.approx(expected_length, abs=1e-9), (
            f"a collision box is {length * MM_PER_M:.0f} mm long, but "
            f"body_length_mm is {cad['body_length_mm']:.0f} mm"
        )
        if bottom < wheel_top - 1e-9:
            assert width <= inner_faces + 1e-9, (
                f"a collision box {width * MM_PER_M:.0f} mm wide reaches below the "
                f"wheel tops, where only {inner_faces * MM_PER_M:.0f} mm fits"
            )
        else:
            # Equals, not merely "wider than the wheels": this is the box that
            # carries tools.generate_sim_meshes.BODY_SHELL_WIDTH_MM, and the
            # URDF's literal 0.380 must track that constant rather than just
            # clear the wheels by some margin.
            assert width == pytest.approx(shell_width, abs=1e-9), (
                f"the box above the wheel tops is {width * MM_PER_M:.0f} mm wide, "
                f"but BODY_SHELL_WIDTH_MM (tools/generate_sim_meshes.py) is "
                f"{BODY_SHELL_WIDTH_MM:.0f} mm"
            )
        assert bottom >= cad["chassis_clearance_mm"] / MM_PER_M - 1e-9, (
            f"a collision box bottom at {bottom * MM_PER_M:.0f} mm is below the belly "
            f"plane at {cad['chassis_clearance_mm']:.0f} mm"
        )

    mast, mast_z = cylinders[0]
    expected_radius = cad["mast_diameter_mm"] / 2 / MM_PER_M
    expected_length = cad["mast_height_mm"] / MM_PER_M
    expected_z = (
        cad["chassis_clearance_mm"] + cad["body_height_mm"]
    ) / MM_PER_M + expected_length / 2

    mast_radius = float(mast.get("radius"))
    assert mast_radius == pytest.approx(expected_radius, abs=1e-9), (
        f"mast cylinder radius {mast_radius * MM_PER_M:.1f} mm != "
        f"{expected_radius * MM_PER_M:.1f} mm from mast_diameter_mm"
    )
    mast_length = float(mast.get("length"))
    assert mast_length == pytest.approx(expected_length, abs=1e-9), (
        f"mast cylinder length {mast_length * MM_PER_M:.0f} mm != "
        f"{expected_length * MM_PER_M:.0f} mm from mast_height_mm"
    )
    assert mast_z == pytest.approx(expected_z, abs=1e-9), (
        f"mast cylinder centred at z={mast_z * MM_PER_M:.0f} mm, expected "
        f"{expected_z * MM_PER_M:.0f} mm from chassis_clearance_mm + body_height_mm "
        "+ mast_height_mm / 2"
    )


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
    """40 kg is what 4 x 5 N.m continuous moves up a 15 degree grade on a
    0.125 m wheel radius, against rolling resistance - hardware/bom/poc-v3.md.

    The old ceiling was 2.3 kg, from a 250:1 micro gearbox that is not on this
    machine.  The ceiling is a property of the drivetrain, so it moved with it;
    it is not a budget anyone may relax.  The estimate is 35 kg +/-20%, and the
    upper end of that band is 42 kg -- so this test is expected to fail if the
    real parts come in heavy, and the answer then is a stronger motor.
    """
    total = sum(
        float(link.find("inertial").find("mass").get("value"))
        for link in _robot().findall("link")
        if link.find("inertial") is not None
    )
    assert total <= 40.0, f"URDF total mass {total:.3f} kg exceeds the 40 kg ceiling"


@pytest.mark.parametrize(
    "link_name,model_key",
    [("base_link", "base_link"), ("wheel_fl_link", "wheel")],
)
def test_urdf_inertial_matches_the_mass_model(link_name, model_key):
    """The <inertial> blocks were pasted from
    ``python tools/generate_sim_meshes.py --print-inertia`` (see the block
    comment at the top of this URDF), but nothing kept them pinned there.
    Change wheel_width_mm and the wheel's ixx/izz go stale silently -- the
    only thing test_total_mass_is_under_the_gearbox_ceiling checks is the sum
    of masses, and every individual mass is still exactly right while the
    inertias it is paired with are not.

    Tolerances match the precision --print-inertia writes: mass to 6 decimal
    places, the origin to 5, the inertia diagonal to 9.  If this fails, re-run
    ``python tools/generate_sim_meshes.py --print-inertia`` and paste the
    result for the failing link into cad/urdf/weeding_rover.urdf.
    """
    expected = mass_properties()[model_key]

    inertial = _link(link_name).find("inertial")
    assert inertial is not None, f"{link_name} has no <inertial> block"

    mass = float(inertial.find("mass").get("value"))
    origin = _xyz(inertial.find("origin"))
    inertia = inertial.find("inertia")

    assert mass == pytest.approx(expected.mass, abs=1e-6), (
        f'{link_name} <mass value="{mass}"/> != '
        f"{expected.mass:.6f} from generate_sim_meshes.mass_properties()"
    )
    for axis, (actual, want) in enumerate(zip(origin, expected.com, strict=True)):
        assert actual == pytest.approx(want, abs=1e-5), (
            f"{link_name} <inertial><origin> axis {axis} is {actual} but "
            f"mass_properties() gives {want:.5f}"
        )
    for name, want in (
        ("ixx", expected.ixx),
        ("iyy", expected.iyy),
        ("izz", expected.izz),
    ):
        actual = float(inertia.get(name))
        assert actual == pytest.approx(want, abs=5e-9), (
            f'{link_name} <inertia {name}="{actual}"/> != {want:.9f} from '
            "generate_sim_meshes.mass_properties()"
        )
