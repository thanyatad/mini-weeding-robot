"""The four wheel joint handles, and the fan-out from two sides to four joints.

Everything in this file runs without Isaac on purpose.  The names and the
left/right grouping are the part that silently rots when CAD changes, and a
test that needs a GPU to notice is a test nobody runs.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sim.isaac.robots.rover.wheels import LEFT_JOINTS, RIGHT_JOINTS, WHEEL_JOINTS, joint_targets

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "cad" / "urdf" / "weeding_rover.urdf"


def _urdf_wheel_joints() -> dict[str, tuple[float, float, float]]:
    return {
        j.get("name"): tuple(float(v) for v in j.find("origin").get("xyz").split())
        for j in ET.parse(URDF).getroot().findall("joint")
        if j.get("type") == "continuous"
    }


def test_the_handles_name_exactly_the_urdf_continuous_joints():
    """A wheel renamed in CAD must fail here, not by turning into a wheel that
    silently never receives a command."""
    assert set(WHEEL_JOINTS) == set(_urdf_wheel_joints())


def test_the_sides_partition_the_wheels():
    assert set(LEFT_JOINTS) | set(RIGHT_JOINTS) == set(WHEEL_JOINTS)
    assert set(LEFT_JOINTS) & set(RIGHT_JOINTS) == set()
    assert len(LEFT_JOINTS) == len(RIGHT_JOINTS) == 2


def test_left_and_right_match_the_sign_of_y_in_the_urdf():
    """coordinate-frames.md: +Y is left.  Swapping these two tuples is the
    cheapest way to build a rover that turns the wrong way, and it would pass
    every test that only looks at magnitudes."""
    origins = _urdf_wheel_joints()
    for name in LEFT_JOINTS:
        assert origins[name][1] > 0, f"{name} is listed left but sits at y={origins[name][1]}"
    for name in RIGHT_JOINTS:
        assert origins[name][1] < 0, f"{name} is listed right but sits at y={origins[name][1]}"


def test_one_value_per_side_reaches_two_joints():
    """The real rover wires its two motors per side in parallel, so front and
    rear cannot be commanded apart.  sim/isaac/README.md#rover-model requires
    the simulation to do the same."""
    targets = joint_targets(1.5, -2.5)

    assert set(targets) == set(WHEEL_JOINTS)
    assert [targets[name] for name in LEFT_JOINTS] == [1.5, 1.5]
    assert [targets[name] for name in RIGHT_JOINTS] == [-2.5, -2.5]


def test_targets_come_back_in_the_declared_joint_order():
    """The order is what indexes into Isaac's DOF array; a dict that reordered
    itself would drive the wrong wheel with no error."""
    assert tuple(joint_targets(0.0, 0.0)) == WHEEL_JOINTS


def test_importing_the_handles_does_not_need_isaac():
    """tests/unit must stay GPU-free.  If this module grew a top-level
    ``from isaacsim import ...`` the whole directory would stop collecting."""
    import sim.isaac.robots.rover.wheels as module

    source = Path(module.__file__).read_text(encoding="utf-8")
    for line in source.splitlines():
        if line.startswith(("import ", "from ")):
            assert "isaac" not in line and "omni" not in line and "pxr" not in line, (
                f"top-level Isaac import: {line!r}"
            )


def test_driving_a_stage_without_an_articulation_says_so():
    from sim.isaac.robots.rover.wheels import WheelJoints

    with pytest.raises(NotImplementedError):
        WheelJoints(None).set_side_velocities(1.0, 1.0)
