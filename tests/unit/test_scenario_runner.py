"""The runner's loudness, which is the only property that makes a scenario
file worth writing.

A scenario that passes because the runner skipped its expectation is worse than
no scenario at all: it reads in review as a checked requirement, and nobody
looks again.  So every one of these tests feeds the runner a file that is wrong
in one specific way and insists it refuses, in the same spirit as
``config.with_overrides`` refusing a dotted path that does not exist.

Three ways to be wrong, and the third is the one that hides:

    an unknown key in expect:      nothing could check it
    an unknown key in set:         nothing could do it
    a key nothing measured         something *could* have checked it, and the
                                   run gave it nothing to check against
"""

from __future__ import annotations

import textwrap

import pytest

from tests.harness.scenario import (
    ACTIONS,
    EXPECTATIONS,
    SCENARIOS_DIR,
    ScenarioError,
    check_scenario,
    load_all,
    load_scenario,
    run_scenario,
)

MINIMAL = """\
name: minimal
bed:
  bed_length_mm: 400
expect:
  final_state: STOPPED
"""


def write(tmp_path, text, name="scratch.yaml"):
    path = tmp_path / name
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


# -- what the loader refuses ------------------------------------------------


def test_an_unknown_expect_key_stops_the_load(tmp_path):
    path = write(
        tmp_path,
        """\
        name: wrong
        expect:
          final_state: STOPPED
          row_following: works
        """,
    )

    with pytest.raises(ScenarioError, match="row_following"):
        load_scenario(path)


def test_the_refusal_names_what_the_runner_does_check(tmp_path):
    """The message has to be actionable: somebody writing a scenario needs the
    vocabulary, not the news that theirs is not in it."""
    path = write(tmp_path, "name: wrong\nexpect:\n  distance_travelled_mm: 1900\n")

    with pytest.raises(ScenarioError, match="final_state"):
        load_scenario(path)


def test_an_unknown_top_level_key_stops_the_load(tmp_path):
    path = write(tmp_path, MINIMAL + "randomization:\n  seeds: 5\n")

    with pytest.raises(ScenarioError, match="randomization"):
        load_scenario(path)


def test_a_bed_key_the_harness_cannot_lay_out_stops_the_load(tmp_path):
    """row_curvature_mm is in tests/README.md and the harness has no curve to
    give it.  Accepting it would mean running a straight row and reporting it
    as a curved one."""
    path = write(
        tmp_path,
        """\
        name: wrong
        bed:
          bed_length_mm: 2000
          row_curvature_mm: 40
        expect:
          final_state: STOPPED
        """,
    )

    with pytest.raises(ScenarioError, match="row_curvature_mm"):
        load_scenario(path)


def test_an_unknown_set_key_stops_the_load(tmp_path):
    path = write(
        tmp_path,
        """\
        name: wrong
        expect:
          final_state: STOPPED
        events:
          - at: 1.0
            set:
              camera: unplugged
        """,
    )

    with pytest.raises(ScenarioError, match="camera"):
        load_scenario(path)


def test_a_file_with_no_expect_stops_the_load(tmp_path):
    path = write(tmp_path, "name: wrong\nbed:\n  bed_length_mm: 400\n")

    with pytest.raises(ScenarioError, match="expect"):
        load_scenario(path)


def test_an_empty_expect_stops_the_load(tmp_path):
    path = write(tmp_path, "name: wrong\nexpect: {}\n")

    with pytest.raises(ScenarioError, match="expect"):
        load_scenario(path)


def test_a_malformed_event_stops_the_load(tmp_path):
    path = write(
        tmp_path,
        """\
        name: wrong
        expect:
          final_state: STOPPED
        events:
          - set:
              estop: true
        """,
    )

    with pytest.raises(ScenarioError, match="at"):
        load_scenario(path)


def test_events_are_ordered_by_when_they_happen(tmp_path):
    """``at`` means what it says whatever order the file lists them in — the
    runner walks the list once and never looks back."""
    path = write(
        tmp_path,
        """\
        name: ordered
        expect:
          final_state: STOPPED
        events:
          - at: 3.0
            set:
              link: cut
          - at: 1.0
            set:
              estop: true
        """,
    )

    assert [event.at_s for event in load_scenario(path).events] == [1.0, 3.0]


# -- what the runner refuses ------------------------------------------------


def test_an_expectation_the_run_never_measured_fails_loudly(tmp_path):
    """The quiet failure this whole file exists for.  ``commanded_after_estop``
    is a key the runner knows, so the load passes — but nothing pressed the
    button, so there is no such moment to read a command after.  Answering it
    with the standing command would report a scenario as passed on evidence it
    never gathered."""
    path = write(
        tmp_path,
        """\
        name: never-pressed
        bed:
          bed_length_mm: 400
        expect:
          commanded_after_estop: { v_mm_s: 0 }
        """,
    )
    scenario = load_scenario(path)

    with pytest.raises(ScenarioError, match="never produced it"):
        check_scenario(scenario, run_scenario(scenario))


def test_an_ordering_expectation_needs_both_halves_to_have_happened(tmp_path):
    """``motors_died_before_link_lost`` is about the 300 < 500 pair.  With only
    one of the two it would be trivially true of a run in which the Pi never
    noticed anything at all."""
    path = write(
        tmp_path,
        """\
        name: no-link-event
        bed:
          bed_length_mm: 400
        expect:
          motors_died_before_link_lost: true
        """,
    )
    scenario = load_scenario(path)

    with pytest.raises(ScenarioError, match="never produced it"):
        check_scenario(scenario, run_scenario(scenario))


def test_an_event_the_run_never_reached_fails_loudly(tmp_path):
    """A row that ends at 4 s cannot have had its cable pulled at 60.  The
    expectations might still pass — and they would be about a different story
    than the one the file tells."""
    path = write(
        tmp_path,
        """\
        name: too-late
        bed:
          bed_length_mm: 400
        expect:
          final_state: STOPPED
        events:
          - at: 60.0
            set:
              link: cut
        """,
    )

    with pytest.raises(ScenarioError, match="before the event"):
        run_scenario(load_scenario(path))


def test_a_field_that_is_not_part_of_a_mapping_expectation_fails_loudly(tmp_path):
    path = write(
        tmp_path,
        """\
        name: wrong-field
        bed:
          bed_length_mm: 2000
        expect:
          commanded_after_estop: { speed_mm_s: 0 }
        events:
          - at: 1.0
            set:
              estop: true
        """,
    )
    scenario = load_scenario(path)

    with pytest.raises(ScenarioError, match="speed_mm_s"):
        check_scenario(scenario, run_scenario(scenario))


def test_a_range_with_a_bound_the_runner_does_not_know_fails_loudly(tmp_path):
    path = write(
        tmp_path,
        """\
        name: wrong-bound
        bed:
          bed_length_mm: 2000
        expect:
          motors_died_within_ms: { under: 300 }
        events:
          - at: 1.0
            set:
              link: cut
        """,
    )
    scenario = load_scenario(path)

    with pytest.raises(ScenarioError, match="min"):
        check_scenario(scenario, run_scenario(scenario))


# -- a failing expectation still fails --------------------------------------


def test_a_wrong_expectation_fails_the_scenario(tmp_path):
    """The runner being loud about what it cannot check is worth nothing if it
    is quiet about what it can."""
    path = write(
        tmp_path,
        """\
        name: wrong-answer
        bed:
          bed_length_mm: 2000
        expect:
          final_state: DRIVING_ROW
        events:
          - at: 1.0
            set:
              estop: true
        """,
    )
    scenario = load_scenario(path)

    with pytest.raises(AssertionError, match="ESTOP"):
        check_scenario(scenario, run_scenario(scenario))


# -- the files in the repo --------------------------------------------------


def test_every_scenario_file_in_the_repo_loads():
    assert [scenario.name for scenario in load_all()] == ["estop-during-drive", "link-lost"]


def test_every_expectation_used_by_a_real_scenario_is_in_the_vocabulary():
    """Trivially true by construction — the loader enforces it.  Asserted
    anyway because the interesting half is the reverse: it prints which keys
    the vocabulary carries that no file yet uses."""
    used = {key for scenario in load_all() for key in scenario.expect}

    assert used <= set(EXPECTATIONS)
    assert set(EXPECTATIONS) - used == set(), (
        "a vocabulary key no scenario exercises is a checker nothing tests"
    )


def test_every_action_the_rig_offers_is_exercised_by_a_real_scenario():
    used = {key for scenario in load_all() for event in scenario.events for key in event.set}

    assert set(ACTIONS) - used == set(), "an action no scenario uses is untested scaffolding"


def test_the_scenarios_directory_is_where_tests_readme_says():
    assert SCENARIOS_DIR.name == "scenarios"
    assert SCENARIOS_DIR.parent.name == "tests"
