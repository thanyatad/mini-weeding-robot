"""Static checks on layering — the failures these catch are the ones that work
in sim and break on hardware, which is the most expensive kind here.

These are grep-level tests on purpose.  The code they guard does not exist yet
at V0; the point is that the day it does, the check is already in place.
"""

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]

#: Everything that runs on the real rover.  None of it may know about a pose.
#:
#: ``perception`` is here for the same reason as the control path: the real
#: rover has no localisation (hardware/mechanical/coordinate-frames.md), so an
#: estimator that reached for one would work in Isaac and have nothing to read
#: in the field.  The front camera is uncalibrated on purpose, which leaves the
#: estimator no way to produce a position even if it wanted to.
POSE_FREE_DIRS = (
    "controller/workflow",
    "controller/motion",
    "controller/safety",
    "perception",
)


def _python_sources(*relative_dirs: str) -> list[Path]:
    return [p for d in relative_dirs for p in sorted((REPO / d).rglob("*.py"))]


@pytest.mark.parametrize("directory", POSE_FREE_DIRS)
def test_nothing_on_the_control_path_depends_on_a_pose(directory):
    """FakeRover has an internal pose and FakeRowSensor may read it — it is the
    harness.  If this leaks into the control path, that code breaks when the
    backend becomes esp32, which has no pose to give."""
    offenders = [
        f"{path.relative_to(REPO)}:{n}"
        for path in _python_sources(directory)
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if re.search(r"(?<![A-Za-z])poses?", line, re.IGNORECASE)
    ]
    assert offenders == [], f"pose reached the control path: {offenders}"


def test_the_harness_lives_in_tests_not_in_controller():
    """FakeRowSensor and FakeLoop read FakeRover.pose.  Shipping them under
    controller/ would put a pose-reading import one line away from production
    code."""
    assert not list((REPO / "controller").rglob("fake_loop.py"))
    assert not list((REPO / "controller").rglob("*row_sensor*.py"))
    assert (REPO / "tests" / "harness" / "fake_loop.py").is_file()


def test_no_controller_or_perception_code_reads_ground_truth_from_isaac():
    """Isaac knows where the rover is; the controller must never ask.  Code that
    does works in simulation and fails in the field."""
    forbidden = ("get_world_pose", "ground_truth", "groundtruth")
    offenders = [
        f"{path.relative_to(REPO)}:{n}"
        for path in _python_sources("controller", "perception")
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if any(token in line for token in forbidden)
    ]
    assert offenders == [], f"ground truth reached the control path: {offenders}"


def test_mixing_stays_free_of_framework_imports():
    """The formula is mirrored in C++ and both sides are held to the same CSV.
    An import here is a divergence waiting to happen."""
    source = (REPO / "controller" / "rover" / "mixing.py").read_text(encoding="utf-8")
    for banned in ("import numpy", "import yaml", "from controller.config"):
        assert banned not in source
