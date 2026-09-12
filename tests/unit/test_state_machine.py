"""The transition table of spec §5.3, edge by edge — including the edges that
must not exist.

A state machine tested only on its legal edges is a state machine that will
happily accept `start` while it is in ERROR.  Every (state, event) pair in the
product is asserted here: the ones the diagram draws land where it says, and
the ones it does not draw are rejected.

Nothing here drives, reads a camera, or sleeps.  The machine owns transitions
and nothing else, which is what makes this file arithmetic rather than a
simulation.
"""

import itertools

import pytest

from controller.workflow import (
    ROW_END_SUSPECTED,
    ErrorCode,
    Event,
    IllegalTransition,
    RoverState,
    StateMachine,
)

#: Every edge spec §5.3 and controller/README.md draw, as
#: ``(state, event) -> destination``.  Anything absent is illegal.
#:
#: ``(ESTOP, ACK)`` is deliberately absent: leaving ESTOP needs the button
#: released *and* an ack, so the ack alone is rejected.  Its two-step path has
#: its own tests below.
LEGAL = {
    (RoverState.BOOT, Event.VALIDATE_OK): RoverState.READY,
    (RoverState.BOOT, Event.VALIDATE_FAILED): RoverState.ERROR,
    (RoverState.READY, Event.START): RoverState.DRIVING_ROW,
    (RoverState.DRIVING_ROW, Event.ROW_END): RoverState.STOPPED,
    (RoverState.DRIVING_ROW, Event.ROW_LOST): RoverState.ERROR,
    (RoverState.DRIVING_ROW, Event.LINK_LOST): RoverState.ERROR,
    (RoverState.DRIVING_ROW, Event.CAMERA_TIMEOUT): RoverState.ERROR,
    (RoverState.DRIVING_ROW, Event.COMMAND_TIMEOUT): RoverState.ERROR,
    (RoverState.DRIVING_ROW, Event.COMMUNICATION_LOST): RoverState.ERROR,
    (RoverState.STOPPED, Event.ACK): RoverState.READY,
    (RoverState.ERROR, Event.ACK): RoverState.READY,
    (RoverState.ESTOP, Event.ESTOP_RELEASED): RoverState.ESTOP,
    **{(state, Event.ESTOP): RoverState.ESTOP for state in RoverState},
}


def machine_in(state: RoverState) -> StateMachine:
    """A machine sitting in ``state``, reached only through legal edges."""
    machine = StateMachine()
    if state is RoverState.BOOT:
        return machine
    if state is RoverState.ESTOP:
        machine.fire(Event.ESTOP)
        return machine
    if state is RoverState.ERROR:
        machine.fire(Event.VALIDATE_FAILED)
        return machine

    machine.fire(Event.VALIDATE_OK)
    if state is RoverState.READY:
        return machine
    machine.fire(Event.START)
    if state is RoverState.DRIVING_ROW:
        return machine
    machine.fire(Event.ROW_END)
    assert machine.state is RoverState.STOPPED
    return machine


class TestTheTransitionTable:
    @pytest.mark.parametrize(
        "state, event",
        [pair for pair in itertools.product(RoverState, Event) if pair in LEGAL],
    )
    def test_a_drawn_edge_lands_where_the_diagram_says(self, state, event):
        machine = machine_in(state)
        assert machine.fire(event) is LEGAL[(state, event)]
        assert machine.state is LEGAL[(state, event)]

    @pytest.mark.parametrize(
        "state, event",
        [pair for pair in itertools.product(RoverState, Event) if pair not in LEGAL],
    )
    def test_an_undrawn_edge_is_rejected_and_leaves_the_state_alone(self, state, event):
        """A rejected transition must not half-apply.  A machine that ends up in
        ERROR because it was asked to do something illegal has lost the reason
        it is in ERROR."""
        machine = machine_in(state)
        with pytest.raises(IllegalTransition) as excinfo:
            machine.fire(event)

        assert state.name in str(excinfo.value)
        assert event.name.lower() in str(excinfo.value)
        assert machine.state is state

    def test_every_state_and_event_pair_is_accounted_for(self):
        """Guards the two parametrized tests above: if a state or an event is
        added and nobody decides what it does, the product grows and this
        catches it before the table silently stops covering it."""
        assert len(list(itertools.product(RoverState, Event))) == len(RoverState) * len(Event)
        assert set(LEGAL) <= set(itertools.product(RoverState, Event))


class TestStates:
    def test_it_starts_in_boot(self):
        assert StateMachine().state is RoverState.BOOT

    def test_the_six_states_are_the_six_the_spec_names(self):
        assert [(s.name, s.value) for s in RoverState] == [
            ("BOOT", 1),
            ("READY", 2),
            ("DRIVING_ROW", 3),
            ("STOPPED", 4),
            ("ERROR", 5),
            ("ESTOP", 6),
        ]

    def test_a_fresh_machine_carries_no_reason_and_no_error(self):
        machine = StateMachine()
        assert machine.stop_reason is None
        assert machine.error_code is None


class TestStoppedIsANormalEndOfRun:
    """STOPPED is not a fault.  The scenario contract in tests/README.md is
    ``final_state: STOPPED``, ``stop_reason: row_end_suspected``,
    ``error_code: null`` — modelling the end of a row as an error makes
    ``row_end`` and ``crop_gap_midrow`` indistinguishable at the one point
    where they have to differ."""

    def test_row_end_stops_with_a_reason_and_no_error_code(self):
        machine = machine_in(RoverState.DRIVING_ROW)
        machine.fire(Event.ROW_END)

        assert machine.state is RoverState.STOPPED
        assert machine.stop_reason == ROW_END_SUSPECTED
        assert machine.error_code is None

    def test_the_reason_is_named_as_a_suspicion_not_a_measurement(self):
        """The name is the warning: green_fraction is a heuristic, and nothing
        at V0 has measured that it separates a gap from the end of a row."""
        assert ROW_END_SUSPECTED == "row_end_suspected"

    def test_acking_a_stop_clears_the_reason(self):
        machine = machine_in(RoverState.STOPPED)
        machine.fire(Event.ACK)

        assert machine.state is RoverState.READY
        assert machine.stop_reason is None


class TestErrorCodes:
    @pytest.mark.parametrize(
        "event, code",
        [
            (Event.ROW_LOST, ErrorCode.ROW_LOST),
            (Event.LINK_LOST, ErrorCode.LINK_LOST),
            (Event.CAMERA_TIMEOUT, ErrorCode.CAMERA_TIMEOUT),
            (Event.COMMAND_TIMEOUT, ErrorCode.COMMAND_TIMEOUT),
            (Event.COMMUNICATION_LOST, ErrorCode.COMMUNICATION_LOST),
        ],
    )
    def test_a_fault_while_driving_records_its_code(self, event, code):
        machine = machine_in(RoverState.DRIVING_ROW)
        machine.fire(event)

        assert machine.state is RoverState.ERROR
        assert machine.error_code is code
        assert machine.stop_reason is None

    def test_failing_validation_at_boot_is_config_invalid(self):
        machine = StateMachine()
        machine.fire(Event.VALIDATE_FAILED)

        assert machine.state is RoverState.ERROR
        assert machine.error_code is ErrorCode.CONFIG_INVALID

    def test_acking_an_error_clears_the_code(self):
        machine = machine_in(RoverState.DRIVING_ROW)
        machine.fire(Event.ROW_LOST)
        machine.fire(Event.ACK)

        assert machine.state is RoverState.READY
        assert machine.error_code is None

    def test_the_codes_are_the_ones_spec_8_5_lists(self):
        assert {code.value for code in ErrorCode} == {
            "row_lost",
            "link_lost",
            "camera_timeout",
            "command_timeout",
            "config_invalid",
            "communication_lost",
            "emergency_stop",
        }


class TestEstop:
    """ESTOP is not one more edge out of DRIVING_ROW.  The button cuts the motor
    rail electrically from wherever the software happens to be, so every state
    has that edge — and getting back out takes two separate things."""

    @pytest.mark.parametrize("state", list(RoverState))
    def test_the_button_is_reachable_from_every_state(self, state):
        machine = machine_in(state)
        assert machine.fire(Event.ESTOP) is RoverState.ESTOP
        assert machine.error_code is ErrorCode.EMERGENCY_STOP

    def test_an_ack_alone_does_not_leave_estop(self):
        """Software acking its way past a button that is still held down is the
        failure this guards."""
        machine = machine_in(RoverState.ESTOP)
        with pytest.raises(IllegalTransition, match="still held"):
            machine.fire(Event.ACK)
        assert machine.state is RoverState.ESTOP

    def test_releasing_alone_does_not_leave_estop(self):
        machine = machine_in(RoverState.ESTOP)
        assert machine.fire(Event.ESTOP_RELEASED) is RoverState.ESTOP

    def test_released_then_acked_returns_to_ready(self):
        machine = machine_in(RoverState.ESTOP)
        machine.fire(Event.ESTOP_RELEASED)

        assert machine.fire(Event.ACK) is RoverState.READY
        assert machine.error_code is None

    def test_pressing_the_button_again_relatches_it(self):
        """Release, press again, then ack: the ack must not ride on the earlier
        release."""
        machine = machine_in(RoverState.ESTOP)
        machine.fire(Event.ESTOP_RELEASED)
        machine.fire(Event.ESTOP)

        with pytest.raises(IllegalTransition):
            machine.fire(Event.ACK)

    def test_it_forgets_the_release_after_leaving(self):
        """A second E-stop must need its own release, not the previous one."""
        machine = machine_in(RoverState.ESTOP)
        machine.fire(Event.ESTOP_RELEASED)
        machine.fire(Event.ACK)
        machine.fire(Event.ESTOP)

        with pytest.raises(IllegalTransition):
            machine.fire(Event.ACK)

    def test_an_estop_during_a_run_overwrites_a_pending_error(self):
        machine = machine_in(RoverState.DRIVING_ROW)
        machine.fire(Event.ROW_LOST)
        machine.fire(Event.ESTOP)

        assert machine.state is RoverState.ESTOP
        assert machine.error_code is ErrorCode.EMERGENCY_STOP


class TestTheDetectorBoundary:
    """`link_lost`, `camera_timeout`, `command_timeout` and `communication_lost`
    are drawn by §5.3, so the edges exist.  Nothing in V0 raises them: their
    detectors are the protocol and link work, and a half-built detector left
    behind here would read as a working one."""

    def test_no_detector_ships_with_the_state_machine(self):
        import controller.workflow.state_machine as module

        assert not hasattr(module, "LinkMonitor")
        assert not hasattr(module, "detect_link_lost")

    def test_the_machine_never_raises_an_event_by_itself(self):
        """It has no clock and no input of its own — every transition is a call
        from outside.  Anything else would be a detector in disguise."""
        machine = machine_in(RoverState.DRIVING_ROW)
        for _ in range(100):
            assert machine.state is RoverState.DRIVING_ROW
