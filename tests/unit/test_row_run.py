"""The §5.5 control loop: estimator -> watchdog -> follower -> rover, and what
a watchdog trip does to the state machine.

Driven with synthetic RowEstimates rather than through FakeLoop, so that the
sequence of frames is chosen rather than emerging from geometry.  The
closed-loop version of the same two endings lives in tests/unit/test_fake_loop.py.
"""

import pytest

from controller.config import load_config
from controller.motion import RowEstimate, RowFollower
from controller.safety import RowLossWatchdog
from controller.workflow import (
    ROW_END_SUSPECTED,
    ErrorCode,
    Event,
    NotDriving,
    RoverState,
    RowRun,
    StateMachine,
)
from tests.harness import RecordingRover

TRIP_AT = 0.30


def invalid(green_fraction=0.0) -> RowEstimate:
    return RowEstimate(
        valid=False,
        lateral_err=0.0,
        heading_err=0.0,
        confidence=0.0,
        green_fraction=green_fraction,
    )


def valid(lateral_err=0.2, heading_err=0.0) -> RowEstimate:
    return RowEstimate(
        valid=True,
        lateral_err=lateral_err,
        heading_err=heading_err,
        confidence=0.8,
        green_fraction=0.4,
    )


@pytest.fixture
def rover():
    return RecordingRover()


@pytest.fixture
def follower():
    return RowFollower.from_config(load_config(), backend="fake")


@pytest.fixture
def run(rover, follower):
    """A run already in DRIVING_ROW, reached through the real transitions."""
    machine = StateMachine()
    row_run = RowRun(
        machine=machine,
        rover=rover,
        follower=follower,
        watchdog=RowLossWatchdog(rover, row_loss_frames=3, green_fraction_trip=TRIP_AT),
    )
    machine.fire(Event.VALIDATE_OK)
    row_run.start()
    return row_run


class TestGettingIntoTheRow:
    def test_start_moves_a_ready_machine_into_the_row(self, rover, follower):
        machine = StateMachine()
        row_run = RowRun(
            machine=machine,
            rover=rover,
            follower=follower,
            watchdog=RowLossWatchdog(rover, 3, TRIP_AT),
        )
        machine.fire(Event.VALIDATE_OK)

        assert row_run.start() is RoverState.DRIVING_ROW

    def test_it_cannot_start_before_validation_has_passed(self, rover, follower):
        """BOOT -> DRIVING_ROW is not an edge.  Starting a run on a config that
        never validated is exactly what invariant checking is there to stop."""
        from controller.workflow import IllegalTransition

        row_run = RowRun(
            machine=StateMachine(),
            rover=rover,
            follower=follower,
            watchdog=RowLossWatchdog(rover, 3, TRIP_AT),
        )
        with pytest.raises(IllegalTransition):
            row_run.start()


class TestAValidEstimate:
    def test_it_drives_what_the_follower_returns(self, run, rover, follower):
        est = valid(lateral_err=0.2)
        run.step(est)

        assert rover.calls == [("drive", *follower.step(est))]

    def test_it_stays_in_the_row(self, run):
        assert run.step(valid()) is RoverState.DRIVING_ROW

    def test_it_resets_the_watchdog(self, run):
        run.step(invalid())
        run.step(invalid())
        run.step(valid())

        assert run.watchdog.consecutive_invalid == 0


class TestAnInvalidEstimate:
    def test_it_commands_nothing_while_the_watchdog_counts(self, run, rover):
        """§5.5 `continue`s past the rover.  The standing command carries on for
        up to row_loss_frames, which is the runaway budget invariant 2 reserves
        — and is what lets a gap mid-row be ridden out instead of stopped on."""
        run.step(invalid())
        run.step(invalid())

        assert rover.calls == []
        assert run.state is RoverState.DRIVING_ROW

    def test_the_third_one_ends_the_run(self, run):
        run.step(invalid())
        run.step(invalid())

        assert run.step(invalid()) is not RoverState.DRIVING_ROW


class TestTheTwoEndings:
    """The same three invalid frames, told apart only by green_fraction (§5.4)."""

    def test_green_gone_is_a_normal_end_of_run(self, run, rover):
        for _ in range(3):
            run.step(invalid(green_fraction=0.0))

        assert run.state is RoverState.STOPPED
        assert run.machine.stop_reason == ROW_END_SUSPECTED
        assert run.machine.error_code is None
        assert rover.calls == [("stop",)]

    def test_green_still_there_is_a_navigation_failure(self, run, rover):
        for _ in range(3):
            run.step(invalid(green_fraction=0.9))

        assert run.state is RoverState.ERROR
        assert run.machine.error_code is ErrorCode.ROW_LOST
        assert run.machine.stop_reason is None
        assert rover.calls == [("stop",)]

    def test_a_gap_mid_row_does_not_end_the_run(self, run, rover):
        """Two invalid frames, a valid one, two more: the count never reaches
        three, so the rover keeps driving.  crop_gap_midrow.yaml is this."""
        for est in (invalid(), invalid(), valid(), invalid(), invalid()):
            run.step(est)

        assert run.state is RoverState.DRIVING_ROW
        assert ("stop",) not in rover.calls


class TestFaultsFromOutsideTheLoop:
    """link_lost, camera_timeout, command_timeout and communication_lost are
    detected by the link monitor and the protocol codec, which are V3.  The
    handling exists; the detectors do not, and this is the seam between them."""

    def test_command_timeout_goes_to_error(self, run):
        assert run.report_fault(Event.COMMAND_TIMEOUT) is RoverState.ERROR
        assert run.machine.error_code is ErrorCode.COMMAND_TIMEOUT

    def test_command_timeout_commands_nothing_at_all(self, run, rover):
        """The ESP32 has already killed the motors.  Anything the controller
        sends now is a command to a rover that is not listening — and the one
        thing it must never send is a drive (§8.5)."""
        run.report_fault(Event.COMMAND_TIMEOUT)

        assert rover.calls == []

    def test_no_drive_is_issued_after_a_command_timeout(self, run, rover):
        """The failure this guards: a controller that sees command_timeout and
        carries on with the next loop iteration is a rover that starts moving
        with nobody having asked it to."""
        run.report_fault(Event.COMMAND_TIMEOUT)

        with pytest.raises(NotDriving):
            run.step(valid())

        assert rover.drive_calls == []
        assert run.state is RoverState.ERROR

    @pytest.mark.parametrize(
        "event, code",
        [
            (Event.CAMERA_TIMEOUT, ErrorCode.CAMERA_TIMEOUT),
            (Event.LINK_LOST, ErrorCode.LINK_LOST),
            (Event.COMMUNICATION_LOST, ErrorCode.COMMUNICATION_LOST),
        ],
    )
    def test_the_other_faults_halt_a_rover_that_may_still_be_moving(self, run, rover, event, code):
        """Unlike command_timeout, nothing has stopped the wheels yet."""
        assert run.report_fault(event) is RoverState.ERROR
        assert run.machine.error_code is code
        assert rover.calls == [("stop",)]

    def test_it_refuses_an_event_that_is_not_a_fault(self, run):
        with pytest.raises(ValueError, match="not a fault"):
            run.report_fault(Event.ROW_END)


class TestSteppingOutsideTheRow:
    def test_a_step_after_the_run_ended_is_refused(self, run, rover):
        for _ in range(3):
            run.step(invalid())
        assert run.state is RoverState.STOPPED

        with pytest.raises(NotDriving):
            run.step(valid())
        assert rover.drive_calls == []

    def test_the_refusal_names_the_state_it_was_in(self, run):
        run.report_fault(Event.LINK_LOST)
        with pytest.raises(NotDriving, match="ERROR"):
            run.step(valid())


class TestDrivingTheLoop:
    def test_it_runs_until_the_row_ends(self, run, rover):
        """Five valid frames then invalid ones: the loop stops itself three
        frames after the estimates go bad, not when the budget runs out."""
        frames = iter([valid()] * 5 + [invalid()] * 10)

        assert run.run(lambda: next(frames), max_frames=20) is RoverState.STOPPED
        assert len(rover.drive_calls) == 5

    def test_it_gives_up_at_the_frame_budget(self, run):
        """A row that never ends must not spin forever.  The budget is the
        caller's bound, not a timeout the rover knows about."""
        assert run.run(lambda: valid(), max_frames=7) is RoverState.DRIVING_ROW

    def test_the_budget_is_counted_in_frames_not_calls(self, run, rover):
        run.run(lambda: valid(), max_frames=7)
        assert len(rover.drive_calls) == 7


class TestTheV2Fence:
    def test_the_weed_log_is_not_here_yet(self, run):
        """§10.6: the MVP does not weed.  weed_log belongs in this loop at V2,
        off the control path at 1-2 Hz, and the loop leaves a place for it —
        but nothing half-built stands in for it now."""
        import controller.workflow.row_run as module

        assert not hasattr(module, "weed_log")
        assert not hasattr(run, "weed_log")

    def test_the_loop_reads_one_camera_per_frame(self, run):
        """The front camera is the control path.  A second read per frame would
        be the down camera creeping into it, at the wrong rate."""
        reads = []

        def estimator():
            reads.append(1)
            return valid()

        run.run(estimator, max_frames=4)
        assert len(reads) == 4


class TestItDoesNotOwnTheOtherSafetyLayers:
    def test_it_never_latches_the_estop(self, run, rover):
        """Cutting the motor rail is the hardware layer's job (§5.6).  A
        controller that latches it in software has taken over a layer it does
        not own, and the layer stops being independent of this process."""
        for _ in range(3):
            run.step(invalid())

        assert ("emergency_stop",) not in rover.calls
