"""The scenario files that need neither a GPU nor a rendered frame.

``tests/README.md`` names ten.  Eight of them describe a rover following a row,
which needs the V1 estimator and Isaac; two describe the link and the button,
which need only Controller + Bridge + ESP32 emulator.  Those two run here.

The files are listed rather than globbed.  A glob would quietly pick up an
Isaac scenario the day somebody adds one and fail with a confusing error, or --
worse -- quietly stop running one that got renamed.  Listing them means both
mistakes fail as themselves, which is what
:func:`test_every_scenario_file_is_accounted_for` is for.
"""

from __future__ import annotations

import pytest

from tests.harness.scenario import (
    SCENARIOS_DIR,
    check_scenario,
    load_scenario,
    run_scenario,
)

#: Runnable today: Controller + Bridge + emulator, no GPU, no Isaac.
RUNNABLE = (
    "link_lost.yaml",
    "estop_during_drive.yaml",
)

#: The other eight from tests/README.md.  Each needs rendered frames and the V1
#: row estimator, and none of them exists yet -- they are named here so that
#: adding one fails this file rather than silently landing in a runner that
#: cannot honour it.
NEEDS_PERCEPTION = (
    "row_straight.yaml",
    "row_curved.yaml",
    "row_tilted_start.yaml",
    "crop_gap_midrow.yaml",
    "row_end.yaml",
    "row_lost.yaml",
    "soil_rough.yaml",
    "weed_in_furrow.yaml",
)


@pytest.mark.parametrize("filename", RUNNABLE)
def test_scenario(filename):
    scenario = load_scenario(SCENARIOS_DIR / filename)
    check_scenario(scenario, run_scenario(scenario))


def test_every_scenario_file_is_accounted_for():
    """A scenario file nothing runs is a file that proves nothing while looking
    like it proves something.  Adding one to tests/scenarios/ must mean
    classifying it here."""
    present = {path.name for path in SCENARIOS_DIR.glob("*.yaml")}
    classified = set(RUNNABLE) | set(NEEDS_PERCEPTION)

    assert present - classified == set(), "a scenario file no runner claims"
    assert set(RUNNABLE) <= present, "a scenario listed as runnable is missing"


def test_the_two_runnable_scenarios_are_the_ones_that_need_no_frames():
    """The split is not arbitrary: these two describe the link and the button,
    and neither asks where the rover is."""
    assert set(RUNNABLE).isdisjoint(NEEDS_PERCEPTION)
    assert len(RUNNABLE) + len(NEEDS_PERCEPTION) == 10, "tests/README.md names ten"
