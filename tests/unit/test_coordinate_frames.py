"""The verification checklist from hardware/mechanical/coordinate-frames.md.

That document ends with a list of six checks and names this file as the place
they live.  Until now the file did not exist, so the checklist was a promise
rather than a test.  Each test below is one line of it, in order.

The sign of ``omega`` is the trap the document singles out.  It is worse than
an offset error: flipped, the row follower's feedback becomes positive and the
rover leaves the furrow FASTER than it would with no steering at all.  Three
places have to agree - ``Rover.drive``, the mixing, and the wheel joint
velocity in Isaac - and nothing about a wrong answer looks wrong.

The last check is a static one.  Code that reads ground truth works perfectly
in simulation and fails in a vegetable bed, which is the most expensive failure
this architecture has.
"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from controller.config import get, load_config
from controller.motion.row_estimate import RowEstimate
from controller.motion.row_follower import RowFollower
from controller.rover.mixing import mix
from sim.isaac.adapters.wheel_adapter import WheelAdapter

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "cad" / "urdf" / "weeding_rover.urdf"


def _follower() -> RowFollower:
    return RowFollower.from_config(load_config(), backend="isaac")


def _estimate(lateral_err: float, heading_err: float = 0.0) -> RowEstimate:
    return RowEstimate(
        valid=True,
        lateral_err=lateral_err,
        heading_err=heading_err,
        confidence=0.9,
        green_fraction=0.5,
    )


# -- [1] lateral_err > 0  ->  omega < 0 ------------------------------------


def test_a_furrow_to_the_right_steers_right():
    """+lateral_err means the furrow is to the right of the image centre, so
    the rover must turn right, and right is negative omega."""
    _v, omega = _follower().step(_estimate(+0.5))
    assert omega < 0.0


def test_a_furrow_to_the_left_steers_left():
    _v, omega = _follower().step(_estimate(-0.5))
    assert omega > 0.0


# -- [2] omega > 0  ->  v_right > v_left -----------------------------------


def test_a_left_turn_runs_the_right_wheels_faster():
    """omega > 0 is CCW with Z up, which is a left turn.  The outside wheel of
    a left turn is the right one."""
    wheels = mix(100.0, +40.0, track_width_mm=120.0, wheel_v_max_mm_s=202.0)
    assert wheels.right_mm_s > wheels.left_mm_s


def test_the_same_sign_survives_the_adapter_into_joint_velocities():
    """The third of the three places that must agree.  mixing could be right
    and the adapter could still flip a side."""
    sides = WheelAdapter.from_config(load_config()).wheel_velocities(100.0, +40.0)
    assert sides.right_rad_s > sides.left_rad_s


def test_a_positive_wheel_joint_velocity_drives_that_wheel_forward():
    """Both wheels take the axis (0, 1, 0), so neither side is mirrored and
    both share a sign.  With Z up and X forward, a positive rotation about +Y
    moves the contact patch backward, which drives the wheel forward.

    If a future URDF mirrored the right-hand axes, the adapter would have to
    negate one side and this test is what would say so.
    """
    axes = {
        joint.get("name"): tuple(float(v) for v in joint.find("axis").get("xyz").split())
        for joint in ET.parse(URDF).getroot().findall("joint")
        if joint.get("type") == "continuous"
    }
    assert len(axes) == 4
    assert set(axes.values()) == {(0.0, 1.0, 0.0)}


# -- [3] mm <-> m at the urdf and adapter edges ----------------------------


def test_mm_to_m_at_the_urdf_edge():
    config = load_config()
    origins = {
        joint.get("name"): tuple(float(v) for v in joint.find("origin").get("xyz").split())
        for joint in ET.parse(URDF).getroot().findall("joint")
        if joint.get("type") == "continuous"
    }
    half_track_m = get(config, "rover.track_width_mm") / 2000.0
    half_base_m = get(config, "rover.wheelbase_mm") / 2000.0

    for name, (x, y, z) in origins.items():
        assert abs(x) == pytest.approx(half_base_m), name
        assert abs(y) == pytest.approx(half_track_m), name
        assert z == pytest.approx(get(config, "rover.wheel_diameter_mm") / 2000.0), name


def test_mm_to_m_at_the_adapter_edge():
    """The adapter divides mm/s by a radius, so the radius it uses must be the
    same length the URDF's collision cylinder is - just written in mm."""
    radius_m = float(
        next(
            link.find("collision/geometry/cylinder").get("radius")
            for link in ET.parse(URDF).getroot().findall("link")
            if link.get("name") == "wheel_fl_link"
        )
    )
    adapter = WheelAdapter.from_config(load_config())
    assert adapter.wheel_radius_mm == pytest.approx(radius_m * 1000.0)


# -- [4] deg <-> rad at the urdf edge --------------------------------------


def test_omega_max_40_deg_s_is_0_698_rad_s():
    """The document's own worked example."""
    omega_max_deg_s = get(load_config(), "rover.drive.omega_max_deg_s")
    assert omega_max_deg_s == 40.0
    assert math.radians(omega_max_deg_s) == pytest.approx(0.698, abs=0.001)


def test_the_wheel_velocity_limit_is_the_config_limit_in_rad_s():
    config = load_config()
    limit_rad_s = float(
        next(
            joint.find("limit").get("velocity")
            for joint in ET.parse(URDF).getroot().findall("joint")
            if joint.get("name") == "wheel_fl"
        )
    )
    expected = get(config, "rover.drive.wheel_v_max_mm_s") / (
        get(config, "rover.wheel_diameter_mm") / 2.0
    )
    assert limit_rad_s == pytest.approx(round(expected, 4))


# -- [5] camera_down pixel -> rover frame -> pixel --------------------------


@pytest.mark.skip(
    reason="camera_down homography arrives with perception/calibration.py at V2 — "
    "V1-ISAAC.md lists both as out of scope for V1"
)
def test_a_pixel_survives_a_round_trip_through_the_rover_frame():
    """The sixth checklist line, deliberately left unimplemented rather than
    quietly dropped.  Nothing calibrates camera_down yet, so there is no
    homography to round-trip through."""


# -- [6] no ground truth on the control path -------------------------------

#: Isaac's ways of answering "where am I".  ``SingleXFormPrim`` and
#: ``Articulation`` are on the list because get_world_pose() hangs off them:
#: the import is the tell, not the call.
GROUND_TRUTH_TOKENS = (
    "get_world_pose",
    "get_world_poses",
    "get_local_pose",
    "ground_truth",
    "groundtruth",
    "SingleXFormPrim",
    "XFormPrim",
    "ROVER_BASE_PRIM_PATH",
)


@pytest.mark.parametrize("directory", ("controller", "perception"))
def test_nothing_on_the_control_path_can_ask_isaac_where_it_is(directory):
    """Isaac knows where the rover is and the controller must never ask.

    The bed -> rover transform does not exist at runtime: there is no
    localization, no encoders, no odometry.  Code written against a pose works
    in simulation and fails in the field, and that is the most expensive
    failure mode this architecture has.  Whoever needs coordinates in the bed
    frame has to go and build M2, not reach into the simulator.
    """
    offenders = [
        f"{path.relative_to(REPO)}:{n}  {line.strip()}"
        for path in sorted((REPO / directory).rglob("*.py"))
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if not line.lstrip().startswith("#")
        for token in GROUND_TRUTH_TOKENS
        if re.search(rf"(?<![A-Za-z_]){re.escape(token)}(?![A-Za-z_])", line)
    ]
    assert offenders == [], f"ground truth reached {directory}/: {offenders}"


def test_the_simulators_own_harness_is_allowed_to_read_it():
    """The rule is about the control path, not about the simulator.
    sim/isaac/app.py measures the rover against its commands and has to read
    ground truth to do it — this test exists so that nobody 'fixes' the check
    above by making it global and then quietly deletes the measurement."""
    app = (REPO / "sim" / "isaac" / "app.py").read_text(encoding="utf-8")
    assert "get_world_pose" in app


def test_the_isaac_backend_exposes_no_pose_of_its_own():
    """The fence that makes the static check above worth having: even with a
    live stage behind it, IsaacRover has nothing pose-shaped on it."""
    from controller.rover.isaac_rover import IsaacRover

    public = {name for name in dir(IsaacRover) if not name.startswith("_")}
    assert not {name for name in public if "pose" in name.lower()}
    assert not {name for name in public if "position" in name.lower()}
