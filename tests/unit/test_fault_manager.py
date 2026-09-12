"""The one place a wire code becomes a workflow event.

``bridge/protocol.py`` says of ``Fault.code`` that it is "a §8.5 string rather
than a ``controller.workflow.ErrorCode``: this package carries and does not
decide, and the controller maps the code to the event it takes".  That mapping
is what is under test here — it was inlined in an integration bench before this
module existed, which meant the shipped path was the one thing the tests were
not exercising.

Routing and recording is the whole of the job.  ``RowRun.report_fault()``
already owns what a fault *does*, and the assertions below are as much about
the manager not doing it a second time: the single ``stop()`` in a
RecordingRover's journal is RowRun's, and a second entry is the drift this
guards against.

The trap the table exists for
-----------------------------
``Event(fault.code)`` works for four of the seven codes and quietly fails for
the one that matters most: ``ErrorCode.EMERGENCY_STOP`` is ``"emergency_stop"``
but the event is ``Event.ESTOP`` — ``"estop"``.  A router built on the strings
lining up would do nothing at all for the button.
"""

import json

import pytest

from bridge.protocol import Fault
from controller.config import load_config
from controller.motion import RowFollower
from controller.safety import RowLossWatchdog
from controller.safety.fault_manager import (
    FAULT_CODE_EVENTS,
    FaultManager,
    UnroutableFault,
)
from controller.workflow import (
    ErrorCode,
    Event,
    RoverState,
    RowRun,
    StateMachine,
)
from tests.harness import RecordingRover
from tests.harness.json_schema import SCHEMA_PATH


def make_run(rover, *, driving=True):
    config = load_config()
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


@pytest.fixture
def manager(run):
    return FaultManager(run)


def fault(code, message="something went wrong", id=None):
    return Fault(code=code, message=message, id=id)


# -- the table --------------------------------------------------------------


def test_the_table_covers_exactly_the_codes_the_schema_defines():
    """§8.5's list of codes lives in protocol/schema.json, and this is the
    controller's dictionary for it.  Pinned against the schema rather than
    against a literal here, so a code added to the wire fails this test instead
    of falling through the router unnoticed."""
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    codes = set(schema["$defs"]["error_code"]["enum"])

    assert set(FAULT_CODE_EVENTS) == codes


def test_emergency_stop_maps_to_estop_and_not_to_its_own_name():
    """The one code whose string does not match its event.  A router that
    called Event(fault.code) would raise here, and one that swallowed the
    failure would do nothing for the button."""
    assert FAULT_CODE_EVENTS["emergency_stop"] is Event.ESTOP

    with pytest.raises(ValueError):
        Event("emergency_stop")


@pytest.mark.parametrize(
    "code, event",
    [
        ("row_lost", Event.ROW_LOST),
        ("link_lost", Event.LINK_LOST),
        ("camera_timeout", Event.CAMERA_TIMEOUT),
        ("command_timeout", Event.COMMAND_TIMEOUT),
        ("config_invalid", Event.VALIDATE_FAILED),
        ("communication_lost", Event.COMMUNICATION_LOST),
        ("emergency_stop", Event.ESTOP),
    ],
)
def test_every_code_maps_to_the_event_5_3_draws_for_it(code, event):
    assert FAULT_CODE_EVENTS[code] is event


# -- routing ----------------------------------------------------------------


@pytest.mark.parametrize(
    "code, error_code",
    [
        ("link_lost", ErrorCode.LINK_LOST),
        ("camera_timeout", ErrorCode.CAMERA_TIMEOUT),
        ("command_timeout", ErrorCode.COMMAND_TIMEOUT),
        ("communication_lost", ErrorCode.COMMUNICATION_LOST),
    ],
)
def test_a_loop_external_fault_goes_to_report_fault(manager, run, code, error_code):
    event = manager.handle(fault(code))

    assert event is FAULT_CODE_EVENTS[code]
    assert run.state is RoverState.ERROR
    assert run.machine.error_code is error_code


@pytest.mark.parametrize("code", ["link_lost", "camera_timeout", "communication_lost"])
def test_report_fault_still_owns_the_braking(manager, rover, code):
    """One stop(), and it is RowRun's.  A manager that braked as well would put
    the rule in two places, and the copy that is not report_fault() would go on
    doing what it always did the day the other changed."""
    manager.handle(fault(code))

    assert rover.calls == [("stop",)]


def test_command_timeout_is_still_the_exception_that_commands_nothing(manager, rover):
    """The board has already killed the motors, so there is nothing left to
    stop and nothing to say to it (§8.5).  That rule lives in report_fault();
    routing must not have quietly restored the stop."""
    manager.handle(fault("command_timeout", message="no drive command for 312 ms"))

    assert rover.calls == []


def test_emergency_stop_goes_to_the_state_machine(manager, run):
    """Not to report_fault(), which rejects it: FAULT_EVENTS leaves the button
    out because ESTOP is legal from everywhere, not only from DRIVING_ROW."""
    event = manager.handle(fault("emergency_stop", message="reset refused"))

    assert event is Event.ESTOP
    assert run.state is RoverState.ESTOP
    assert run.machine.error_code is ErrorCode.EMERGENCY_STOP


def test_an_estop_fault_commands_nothing(manager, rover):
    """The rail is cut electrically before this code runs (§11.4).  A manager
    that braked here would be the software layer doing the hardware layer's
    job."""
    manager.handle(fault("emergency_stop"))

    assert rover.calls == []


def test_an_estop_fault_from_boot_lands_in_estop(rover):
    """No run is driving, and the button does not care.  report_fault() would
    have been refused here; the machine takes it from every state."""
    run = make_run(rover, driving=False)

    FaultManager(run).handle(fault("emergency_stop"))

    assert run.state is RoverState.ESTOP
    assert rover.calls == []


def test_it_hands_over_and_never_drives(manager, rover):
    manager.handle(fault("link_lost"))

    assert rover.drive_calls == []


# -- the codes with no wire path --------------------------------------------


def test_row_lost_is_refused_and_names_its_real_owner(manager, run, rover):
    """row_lost is the row-loss watchdog's verdict, and the watchdog brakes
    before it fires.  Routing it from here would be a second, brakeless path to
    a transition that already has an owner — so it is in the table as part of
    the §8.5 dictionary, and refused at dispatch."""
    with pytest.raises(UnroutableFault, match="watchdog"):
        manager.handle(fault("row_lost"))

    assert run.state is RoverState.DRIVING_ROW
    assert rover.calls == []


def test_config_invalid_is_refused_and_names_its_real_owner(manager, run):
    """Validation happens before a run starts, and a board has no opinion on
    this controller's config file."""
    with pytest.raises(UnroutableFault, match="startup"):
        manager.handle(fault("config_invalid"))

    assert run.state is RoverState.DRIVING_ROW


def test_an_unknown_code_is_refused_loudly(manager, run, rover):
    """A code the catalog does not define is a board speaking a protocol this
    controller does not know.  Ignoring it would be the controller carrying on
    while something upstream has already gone wrong."""
    with pytest.raises(UnroutableFault, match="endstop_failure"):
        manager.handle(fault("endstop_failure"))

    assert run.state is RoverState.DRIVING_ROW
    assert rover.calls == []


# -- recording --------------------------------------------------------------


def test_it_records_the_fault_with_its_message_and_id(manager):
    """The single place a fault is logged.  ``message`` and ``id`` are dropped
    the moment a code becomes an Event, and they are the whole of what makes a
    log entry worth reading."""
    arrived = fault("communication_lost", message="no ack for command 77", id=77)

    manager.handle(arrived)

    assert manager.history == [arrived]
    assert manager.history[0].message == "no ack for command 77"
    assert manager.history[0].id == 77


def test_a_refused_fault_is_still_recorded(manager):
    """Recorded on arrival, not on success.  A fault that could not be routed
    is the one most worth finding in the log afterwards."""
    with pytest.raises(UnroutableFault):
        manager.handle(fault("endstop_failure", message="from a board that is not ours"))

    assert [f.code for f in manager.history] == ["endstop_failure"]


def test_nothing_arrives_in_the_history_unasked(manager):
    assert manager.history == []


# -- the list the poll() loop actually hands over ---------------------------


def test_handle_all_routes_every_fault_in_order(manager, run):
    """``Esp32Rover.poll()`` returns a list, and the first fault in it can end
    the run — so the second is handed to a state machine that has already
    moved.  Whether that is legal is the machine's to say, not a guard here:
    ERROR has no edge for a second communication_lost, and every edge there is
    for the button."""
    events = manager.handle_all([fault("communication_lost"), fault("emergency_stop")])

    assert events == [Event.COMMUNICATION_LOST, Event.ESTOP]
    assert run.state is RoverState.ESTOP


def test_handle_all_on_an_empty_poll_does_nothing(manager, run, rover):
    assert manager.handle_all([]) == []
    assert run.state is RoverState.DRIVING_ROW
    assert rover.calls == []
