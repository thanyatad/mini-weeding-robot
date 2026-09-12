"""The detector for link_lost, and the line between detecting and handling.

``RowRun.report_fault()`` already decides what a fault does: it takes the
transition first, and calls ``stop()`` for everything except
``command_timeout``.  A monitor that also stopped the rover would have put that
decision in two places, and the day one of them changes the other keeps doing
what it always did.

So the assertions here are about what the monitor does *not* do as much as what
it does, and they are made on a RecordingRover rather than a mock: a second
``stop()`` in the journal is exactly the drift this is guarding against.
"""

import pytest

from controller.config import get, load_config, with_overrides
from controller.motion import RowFollower
from controller.safety import LinkMonitor, RowLossWatchdog
from controller.workflow import (
    ErrorCode,
    Event,
    IllegalTransition,
    RoverState,
    RowRun,
    StateMachine,
)
from tests.harness import RecordingRover

LINK_LOST_MS = 500.0


def make_run(rover, config=None, *, driving=True):
    """A RowRun built from the real pieces, in DRIVING_ROW unless told otherwise."""
    config = config if config is not None else load_config()
    machine = StateMachine()
    run = RowRun(
        machine=machine,
        rover=rover,
        follower=RowFollower.from_config(config, backend="fake"),
        watchdog=RowLossWatchdog.from_config(rover, config),
    )
    if driving:
        machine.fire(Event.VALIDATE_OK)
        run.start()
    return run


@pytest.fixture
def rover():
    return RecordingRover()


@pytest.fixture
def run(rover):
    return make_run(rover)


def test_a_fresh_link_does_not_trip(run, rover):
    monitor = LinkMonitor(run, link_lost_ms=LINK_LOST_MS)

    assert monitor.tick(link_age_ms=0.0) is False
    assert monitor.tick(link_age_ms=100.0) is False
    assert monitor.tick(link_age_ms=LINK_LOST_MS - 1) is False

    assert run.state is RoverState.DRIVING_ROW
    assert rover.calls == []


def test_it_trips_at_exactly_link_lost_ms(run):
    monitor = LinkMonitor(run, link_lost_ms=LINK_LOST_MS)

    assert monitor.tick(link_age_ms=LINK_LOST_MS) is True
    assert run.state is RoverState.ERROR
    assert run.machine.error_code is ErrorCode.LINK_LOST


def test_the_trip_moves_with_the_configured_ceiling(rover):
    """Read from config, not from a constant here: proven by moving the number
    and watching the trip move with it."""
    config = with_overrides(load_config(), {"safety.link_lost_ms": 1200})
    monitor = LinkMonitor.from_config(make_run(rover, config), config)

    assert monitor.link_lost_ms == 1200
    assert monitor.tick(link_age_ms=600.0) is False
    assert monitor.tick(link_age_ms=1200.0) is True


def test_from_config_reads_safety_link_lost_ms(run):
    config = load_config()

    assert LinkMonitor.from_config(run, config).link_lost_ms == get(config, "safety.link_lost_ms")


def test_it_hands_the_fault_over_and_does_not_touch_the_rover(run, rover):
    """The one stop() in the journal is RowRun's, taken on the way into ERROR.
    A monitor that braked as well would leave two."""
    LinkMonitor(run, link_lost_ms=LINK_LOST_MS).tick(link_age_ms=LINK_LOST_MS)

    assert rover.calls == [("stop",)]
    assert rover.drive_calls == []


def test_it_trips_once(run, rover):
    """The link does not come back by itself, so link_age_ms keeps climbing.
    A monitor that fired every tick would try to transition out of ERROR, which
    the state machine rejects, and would brake a rover already stopped."""
    monitor = LinkMonitor(run, link_lost_ms=LINK_LOST_MS)

    assert monitor.tick(link_age_ms=LINK_LOST_MS) is True
    assert monitor.tick(link_age_ms=900.0) is False
    assert monitor.tick(link_age_ms=5000.0) is False

    assert rover.calls == [("stop",)]


def test_reset_arms_it_again(run):
    """Cleared for the next run, the way the row-loss watchdog is."""
    monitor = LinkMonitor(run, link_lost_ms=LINK_LOST_MS)
    monitor.tick(link_age_ms=LINK_LOST_MS)
    assert monitor.tripped is True

    monitor.reset()

    assert monitor.tripped is False


def test_a_fault_for_a_run_that_is_not_driving_is_the_run_s_to_refuse(rover):
    """The monitor does not check the state first, because that check already
    exists in one place: the state machine rejects the transition and nothing
    is commanded on the way out.  A monitor with its own copy of the rule is a
    second place for it to change."""
    run = make_run(rover, driving=False)
    monitor = LinkMonitor(run, link_lost_ms=LINK_LOST_MS)

    with pytest.raises(IllegalTransition):
        monitor.tick(link_age_ms=LINK_LOST_MS)

    assert run.state is RoverState.BOOT
    assert rover.calls == []
