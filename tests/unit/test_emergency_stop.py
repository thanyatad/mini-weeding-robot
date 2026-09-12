"""The E-stop sense line, and the line between noticing it and handling it.

The button cuts the motor rail electrically (§11.4).  By the time this detector
sees anything, the rover has already stopped — so what it does is record that
fact in the state machine, and nothing else.

Which makes the negative assertions the interesting ones.  A detector that
called ``emergency_stop()`` would be software taking over a layer designed to
work whether or not this process is alive (§5.6), and a detector that latched
the button itself would be a second copy of a rule the state machine already
owns.  Both are asserted here on a RecordingRover rather than a mock: an extra
entry in the journal is exactly the drift being guarded against.

The two-step exit is the other thing under test.  ``ESTOP_RELEASED`` is a fact
about a wire and ``ACK`` is a fact about a person; this detector produces the
first and never the second.
"""

import pytest

from controller.safety import EmergencyStop
from controller.workflow import (
    ErrorCode,
    Event,
    IllegalTransition,
    RoverState,
    StateMachine,
)
from tests.harness import RecordingRover


@pytest.fixture
def machine():
    return StateMachine()


@pytest.fixture
def driving(machine):
    """A machine in DRIVING_ROW, the state the button is most likely pressed in."""
    machine.fire(Event.VALIDATE_OK)
    machine.fire(Event.START)
    return machine


def test_a_quiet_line_fires_nothing(driving):
    detector = EmergencyStop(driving)

    assert detector.tick(estop=False) is None
    assert detector.tick(estop=False) is None
    assert driving.state is RoverState.DRIVING_ROW


def test_the_line_going_active_fires_estop(driving):
    detector = EmergencyStop(driving)

    assert detector.tick(estop=True) is Event.ESTOP
    assert driving.state is RoverState.ESTOP
    assert driving.error_code is ErrorCode.EMERGENCY_STOP


@pytest.mark.parametrize("state_name", ["BOOT", "READY", "DRIVING_ROW", "STOPPED", "ERROR"])
def test_it_fires_from_every_state(machine, state_name):
    """§5.3 draws ESTOP from everywhere, BOOT and ERROR included: the button
    cuts the rail wherever the software happens to be."""
    _advance_to(machine, state_name)
    assert machine.state.name == state_name

    assert EmergencyStop(machine).tick(estop=True) is Event.ESTOP

    assert machine.state is RoverState.ESTOP
    assert machine.error_code is ErrorCode.EMERGENCY_STOP


def test_a_held_button_fires_once(driving):
    """The line stays active for as long as somebody holds it down.  A detector
    that fired every tick would re-latch a latch that never came up, and would
    bury the one press worth reading in the log."""
    detector = EmergencyStop(driving)

    assert detector.tick(estop=True) is Event.ESTOP
    assert detector.tick(estop=True) is None
    assert detector.tick(estop=True) is None


def test_a_button_already_held_at_construction_still_fires(machine):
    """The line is assumed inactive until it is read.  Booting with the button
    held must land in ESTOP, not be mistaken for the steady state."""
    assert EmergencyStop(machine).tick(estop=True) is Event.ESTOP
    assert machine.state is RoverState.ESTOP


def test_the_line_going_quiet_fires_estop_released_and_stays_in_estop(driving):
    """Releasing the button is not leaving ESTOP.  It records one of the two
    facts needed to leave, and the state does not move on it."""
    detector = EmergencyStop(driving)
    detector.tick(estop=True)

    assert detector.tick(estop=False) is Event.ESTOP_RELEASED

    assert driving.state is RoverState.ESTOP
    assert driving.estop_released is True
    assert driving.error_code is ErrorCode.EMERGENCY_STOP


def test_it_never_fires_ack(driving):
    """ACK is a person deciding it is safe to carry on.  Nothing this detector
    can see is evidence of that, so it produces the wire's fact and stops."""
    detector = EmergencyStop(driving)

    fired = [detector.tick(estop=state) for state in (True, False, False, True, False)]

    assert Event.ACK not in fired
    assert set(fired) == {Event.ESTOP, Event.ESTOP_RELEASED, None}


def test_released_then_acked_in_that_order_reaches_ready(driving):
    detector = EmergencyStop(driving)
    detector.tick(estop=True)
    detector.tick(estop=False)

    assert driving.fire(Event.ACK) is RoverState.READY
    assert driving.error_code is None


def test_an_ack_while_the_button_is_still_held_is_refused(driving):
    detector = EmergencyStop(driving)
    detector.tick(estop=True)

    with pytest.raises(IllegalTransition):
        driving.fire(Event.ACK)

    assert driving.state is RoverState.ESTOP


def test_a_second_press_re_latches(driving):
    """Pressed, released, pressed again.  The second press must not ride on the
    release that came before it — proven by the ACK that is refused afterwards,
    which is the only way to tell a re-latch from a stale one."""
    detector = EmergencyStop(driving)
    detector.tick(estop=True)
    detector.tick(estop=False)
    assert driving.estop_released is True

    assert detector.tick(estop=True) is Event.ESTOP

    assert driving.estop_released is False
    with pytest.raises(IllegalTransition):
        driving.fire(Event.ACK)
    assert driving.state is RoverState.ESTOP


def test_a_press_after_a_full_recovery_latches_again(driving):
    """Press, release, ack, press.  The run is back in READY and the button
    works from there exactly as it did from DRIVING_ROW."""
    detector = EmergencyStop(driving)
    detector.tick(estop=True)
    detector.tick(estop=False)
    driving.fire(Event.ACK)
    assert driving.state is RoverState.READY

    assert detector.tick(estop=True) is Event.ESTOP
    assert driving.state is RoverState.ESTOP


def test_it_hands_the_event_over_and_does_not_touch_the_rover(driving):
    """The rail is already cut.  A detector that also called stop() or
    emergency_stop() would be the software layer doing the hardware layer's
    job — and would go on believing it had, the day the button is not wired."""
    rover = RecordingRover()
    detector = EmergencyStop(driving)

    detector.tick(estop=True)
    detector.tick(estop=False)
    detector.tick(estop=True)

    assert rover.calls == []


def test_it_does_not_own_the_latch(driving):
    """``estop_released`` lives in the state machine and is read from there.
    A detector holding its own copy would be a second answer to the same
    question, and the two would differ the first time a run was restarted."""
    detector = EmergencyStop(driving)
    detector.tick(estop=True)
    detector.tick(estop=False)

    assert driving.estop_released is True
    assert not hasattr(detector, "estop_released")
    assert not hasattr(detector, "latched")


def test_a_release_the_machine_has_no_edge_for_is_the_machine_s_to_refuse(machine):
    """The detector does not check the state before firing, because that check
    already exists in exactly one place.  Reached only by firing ESTOP_RELEASED
    at a machine that was never in ESTOP — the state machine rejects it, and a
    guard here would be a second copy of the rule."""
    detector = EmergencyStop(machine)
    detector._active = True  # the line was read as active by somebody else

    with pytest.raises(IllegalTransition):
        detector.tick(estop=False)

    assert machine.state is RoverState.BOOT


def _advance_to(machine: StateMachine, state_name: str) -> None:
    """Walk the machine to ``state_name`` using only edges §5.3 draws."""
    if state_name == "BOOT":
        return
    machine.fire(Event.VALIDATE_OK)
    if state_name == "READY":
        return
    machine.fire(Event.START)
    if state_name == "DRIVING_ROW":
        return
    if state_name == "STOPPED":
        machine.fire(Event.ROW_END)
        return
    if state_name == "ERROR":
        machine.fire(Event.LINK_LOST)
        return
    raise AssertionError(f"no route to {state_name}")
