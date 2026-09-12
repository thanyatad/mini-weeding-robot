"""YAML scenario definitions, loaded and run against Controller + Bridge + board.

``tests/README.md`` names ten scenario files and gives their shape.  Eight of
them need rendered frames or Isaac and are not written yet; the two that need
neither run here, against the same ``Bench`` the integration tests drive.

    link_lost.yaml            the cable goes -> the motors die on the board's
                              own account, before the Pi says a word
    estop_during_drive.yaml   the button goes down mid-row

The vocabulary is exactly what the harness can measure
------------------------------------------------------
Every key in ``expect:`` is dispatched through :data:`EXPECTATIONS`, and a key
with no entry there stops the load.  Every key that *is* dispatched must find
its measurement in the observations, and a measurement that was never taken
stops the run.  Both directions matter, and the second is the one that is easy
to get wrong: an expectation the runner silently skips is worse than no
scenario file at all, because it reads in review like something that was
checked.

So the vocabulary is small on purpose.  It grows with the scenarios that need
it — ``distance_travelled_mm``, ``min_clearance_mm`` and the rest belong to the
eight files that run against a real estimator, and adding them now would mean
eight keys nothing measures and one more place for a scenario to pass by
default.

The numbers in a scenario file are requirements, not observations
-----------------------------------------------------------------
``motors_died_within_ms: {max: 300}`` is ``safety.command_timeout_ms`` written
out as what the rover must do, deliberately not read from the config at run
time.  Read from config it would agree with itself for ever, including on the
day somebody raises the timeout to make something else pass.  Written down, a
board that needs longer than the spec allows fails the scenario.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from controller.workflow import RoverState
from tests.harness.bench import BED_LENGTH_MM, Bench

SCENARIOS_DIR = Path(__file__).resolve().parents[2] / "tests" / "scenarios"

#: Rounds run after the state machine leaves DRIVING_ROW.  Telemetry is
#: published at 20 Hz and the loop runs at 10, so the board's last word about a
#: run always arrives after the run has ended.
SETTLE_ROUNDS = 5

#: A bound on a scenario that never finishes.  The rover knows nothing about it.
MAX_ROUNDS = 400

_TOP_LEVEL = frozenset({"name", "bed", "events", "expect"})
_BED_KEYS = frozenset({"bed_length_mm"})
_EVENT_KEYS = frozenset({"at", "set"})


class ScenarioError(Exception):
    """A scenario file that cannot be run, or cannot be believed.

    One class for every way a file is wrong, because every one of them has the
    same consequence: nothing about the rover has been proven, and saying so
    loudly is the whole job.
    """


@dataclass(frozen=True)
class ScenarioEvent:
    """Something done to the rig part-way through the run."""

    at_s: float
    #: What to do, in the file's own words: ``{"estop": True}``, ``{"link": "cut"}``.
    set: dict[str, Any]


@dataclass(frozen=True)
class Scenario:
    name: str
    path: Path
    bed_length_mm: float
    events: tuple[ScenarioEvent, ...]
    expect: dict[str, Any]


# -- what a scenario may do to the rig --------------------------------------


def _press_estop(bench: Bench, value: Any) -> None:
    if not isinstance(value, bool):
        raise ScenarioError(f"estop takes true or false, got {value!r}")
    bench.board.press_estop() if value else bench.board.release_estop()


def _cut_link(bench: Bench, value: Any) -> None:
    if value != "cut":
        raise ScenarioError(f"link takes 'cut', got {value!r} — a cut cable does not come back")
    bench.wire.cut()


#: ``set:`` keys, and the hand that does each one.  Both are hardware: the
#: button and the cable.  Nothing here reaches into the controller, which is
#: what keeps a scenario a description of the world rather than of the code.
ACTIONS = {
    "estop": _press_estop,
    "link": _cut_link,
}


# -- what a scenario may expect ---------------------------------------------


@dataclass(frozen=True)
class _Equals:
    """``key: value`` — the observation must equal it exactly."""

    field: str

    def check(self, observed: Any, expected: Any) -> None:
        assert observed == expected, f"expected {expected!r}, observed {observed!r}"


@dataclass(frozen=True)
class _Range:
    """``key: {min: a, max: b}`` — either bound may be left out."""

    field: str

    def check(self, observed: Any, expected: Any) -> None:
        if not isinstance(expected, dict) or not expected or set(expected) - {"min", "max"}:
            raise ScenarioError(f"a range takes 'min' and/or 'max', got {expected!r}")
        if "min" in expected:
            assert observed >= expected["min"], (
                f"expected at least {expected['min']}, got {observed}"
            )
        if "max" in expected:
            assert observed <= expected["max"], (
                f"expected at most {expected['max']}, got {observed}"
            )


@dataclass(frozen=True)
class _Fields:
    """``key: {v_mm_s: 0}`` — every named field of a mapping, and no others."""

    field: str

    def check(self, observed: Any, expected: Any) -> None:
        if not isinstance(expected, dict) or not expected:
            raise ScenarioError(f"expected a mapping of fields, got {expected!r}")
        unknown = set(expected) - set(observed)
        if unknown:
            raise ScenarioError(
                f"{sorted(unknown)} is not a field of {self.field} — it carries {sorted(observed)}"
            )
        for name, value in expected.items():
            assert observed[name] == value, (
                f"expected {name}={value!r}, observed {observed[name]!r}"
            )


#: Every key ``expect:`` may name, and the observation each one is checked
#: against.  A key that is not here stops the load; see the module docstring.
EXPECTATIONS: dict[str, Any] = {
    "final_state": _Equals("final_state"),
    "error_code": _Equals("error_code"),
    "stop_reason": _Equals("stop_reason"),
    "motors_enabled": _Equals("motors_enabled"),
    "motors_died_within_ms": _Range("motors_died_within_ms"),
    "motors_died_before_link_lost": _Equals("motors_died_before_link_lost"),
    "commanded_after_estop": _Fields("commanded_after_estop"),
}


# -- loading ----------------------------------------------------------------


def load_scenario(path: Path) -> Scenario:
    """Read one YAML file, refusing anything the runner cannot honour."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ScenarioError(f"{path.name}: a scenario is a mapping at the top level")

    unknown = set(document) - _TOP_LEVEL
    if unknown:
        raise ScenarioError(
            f"{path.name}: unknown top-level key(s) {sorted(unknown)} — "
            f"the runner honours {sorted(_TOP_LEVEL)}"
        )
    for required in ("name", "expect"):
        if required not in document:
            raise ScenarioError(
                f"{path.name}: no {required!r} — a scenario without one proves nothing"
            )

    bed = document.get("bed") or {}
    unknown = set(bed) - _BED_KEYS
    if unknown:
        raise ScenarioError(
            f"{path.name}: unknown bed key(s) {sorted(unknown)} — the harness lays out "
            f"a straight row of a given length and nothing else yet"
        )

    expect = document["expect"]
    if not isinstance(expect, dict) or not expect:
        raise ScenarioError(f"{path.name}: 'expect' is a non-empty mapping")
    unknown = set(expect) - set(EXPECTATIONS)
    if unknown:
        raise ScenarioError(
            f"{path.name}: nothing checks {sorted(unknown)} — an expectation the runner "
            f"skips reads like one that passed.  Known: {sorted(EXPECTATIONS)}"
        )

    return Scenario(
        name=document["name"],
        path=path,
        bed_length_mm=float(bed.get("bed_length_mm", BED_LENGTH_MM)),
        events=_load_events(path, document.get("events") or []),
        expect=expect,
    )


def _load_events(path: Path, raw: Any) -> tuple[ScenarioEvent, ...]:
    if not isinstance(raw, list):
        raise ScenarioError(f"{path.name}: 'events' is a list")

    events = []
    for entry in raw:
        if not isinstance(entry, dict) or set(entry) != _EVENT_KEYS:
            raise ScenarioError(f"{path.name}: an event is {sorted(_EVENT_KEYS)}, got {entry!r}")
        unknown = set(entry["set"]) - set(ACTIONS)
        if unknown:
            raise ScenarioError(
                f"{path.name}: nothing can set {sorted(unknown)} — the rig offers {sorted(ACTIONS)}"
            )
        events.append(ScenarioEvent(at_s=float(entry["at"]), set=dict(entry["set"])))

    # Sorted so that "at" means what it says whatever order the file lists them
    # in; the runner walks the list once and never looks back.
    return tuple(sorted(events, key=lambda event: event.at_s))


def load_all(directory: Path = SCENARIOS_DIR) -> list[Scenario]:
    return [load_scenario(path) for path in sorted(directory.glob("*.yaml"))]


# -- running ----------------------------------------------------------------


def run_scenario(scenario: Scenario) -> dict[str, Any]:
    """Drive the scenario and return everything it turned out to be possible
    to measure.  Checking is :func:`check_scenario`'s job."""
    bench = Bench(row_length_mm=scenario.bed_length_mm)
    pending = list(scenario.events)
    fired_at_ms: dict[str, float] = {}

    for _ in range(MAX_ROUNDS):
        while pending and bench.now_ms >= pending[0].at_s * 1000.0:
            event = pending.pop(0)
            for key, value in event.set.items():
                ACTIONS[key](bench, value)
                fired_at_ms[key] = bench.now_ms
        if bench.state is not RoverState.DRIVING_ROW:
            break
        bench.round()

    # Reported below rather than bench.now_ms, which the settle rounds have
    # already moved past the event the run never reached.
    ended_at_ms, ended_in = bench.now_ms, bench.state.name

    for _ in range(SETTLE_ROUNDS):
        bench.round()

    if pending:
        # The run ended before the file's own story did.  Whatever the
        # expectations then say, they are not about the thing the file
        # describes.
        raise ScenarioError(
            f"{scenario.path.name}: the run reached {ended_in} at {ended_at_ms:.0f} ms, "
            f"before the event at {pending[0].at_s} s — the scenario never happened"
        )

    return _observe(bench, fired_at_ms)


def _observe(bench: Bench, fired_at_ms: dict[str, float]) -> dict[str, Any]:
    """Everything measurable about the finished run.

    A key is absent when the run gave no occasion to measure it — the button
    was never pressed, the motors never died.  Absent and not ``None``: a
    scenario may legitimately expect ``error_code: null``, so "no value" and
    "the value is nothing" have to be different answers.
    """
    observed: dict[str, Any] = {
        "final_state": bench.state.name,
        "error_code": None if bench.machine.error_code is None else bench.machine.error_code.value,
        "stop_reason": bench.machine.stop_reason,
        # From the board and not from telemetry, because a scenario describes
        # the rig rather than what the Pi believes about it.  Down a cut cable
        # the last telemetry the Pi received still says the motors are enabled,
        # and it says so for ever -- which is the whole reason link_lost is a
        # fault at all.  Read from there, this expectation would be silently
        # unmeasurable in exactly the scenario the link is under test in.
        "motors_enabled": bench.board.motors_enabled,
    }

    if bench.motors_died_at_ms is not None and "link" in fired_at_ms:
        observed["motors_died_within_ms"] = bench.motors_died_at_ms - fired_at_ms["link"]
    if bench.motors_died_at_ms is not None and bench.link_lost_at_ms is not None:
        # Both halves of the §8.5 pair have to have happened for the ordering
        # between them to mean anything.  With only one, "the motors died
        # first" would be true of a run in which the Pi never noticed at all.
        observed["motors_died_before_link_lost"] = bench.motors_died_at_ms < bench.link_lost_at_ms
    if bench.estop_fired_at_ms is not None:
        # The board's own account of what it is doing, which reached the Pi
        # because this scenario leaves the cable alone.  The controller's echo
        # in get_drive_state() still holds the last drive it sent.
        observed["commanded_after_estop"] = bench.rover.last_state["commanded"]

    return observed


def check_scenario(scenario: Scenario, observed: dict[str, Any]) -> None:
    """Check every expectation the file names, and fail on one that cannot be
    checked rather than passing over it."""
    for key, expected in scenario.expect.items():
        expectation = EXPECTATIONS[key]
        if expectation.field not in observed:
            raise ScenarioError(
                f"{scenario.path.name}: expects {key!r}, but the run never produced it — "
                f"measured {sorted(observed)}"
            )
        expectation.check(observed[expectation.field], expected)
