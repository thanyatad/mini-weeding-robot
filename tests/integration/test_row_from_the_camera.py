"""A row driven from pixels: synthetic frames -> RowEstimate -> wheels.

    FakeCameraSet -> RowEstimator -> RowRun -> RowFollower -> Rover
                                        |
                                        +-> RowLossWatchdog -> StateMachine

``row_run.run()`` has taken a ``Callable[[], RowEstimate]`` since it was
written, and until Stage 1 nothing in the repo returned one.  Every other piece
of §5.5 was finished and separately tested; the loop could not close for want
of the one function that produces the estimate.  So the point of this file is
narrow and worth stating: it is the proof that the block is gone -- images in
at one end, drive commands out at the other, with nothing synthetic in between.

The complementary file is tests/integration/test_row_over_the_link.py, which
drives the *other* half of the same loop over a real codec and a real wire but
takes its estimates from ``FakeRowSensor`` and the harness's pose.  Between
them the whole of §5.5 is covered by something real at every position.

No Isaac and no GPU: the frames are generated, the furrow position is known
because it was chosen, and the interpreter is the repo's own.
"""

from __future__ import annotations

import pytest

from controller.config import load_config
from controller.motion import RowFollower
from controller.safety import RowLossWatchdog
from controller.workflow import ROW_END_SUSPECTED, ErrorCode, Event, RoverState, RowRun, StateMachine
from perception.cameras import FakeCameraSet
from perception.row_estimator import RowEstimator
from tests.harness import SHAPE, RecordingRover, bare_soil, crop_rows, receding_row


def drive(frames, max_frames: int | None = None):
    """Run the §5.5 loop against ``frames``, wired the way ``main.py`` will be.

    The lambda is the whole of the wiring: ``cameras.front()`` hands over a
    ``Frame``, the estimator reads its pixels, and ``RowRun`` never learns that
    a camera exists.
    """
    config = load_config()
    rover = RecordingRover()
    machine = StateMachine()

    run = RowRun(
        machine=machine,
        rover=rover,
        follower=RowFollower.from_config(config, backend="fake"),
        watchdog=RowLossWatchdog.from_config(rover, config),
    )
    machine.fire(Event.VALIDATE_OK)
    run.start()

    cameras = FakeCameraSet(front=frames)
    estimator = RowEstimator.from_config(config)

    run.run(
        lambda: estimator.estimate(cameras.front().image),
        max_frames if max_frames is not None else len(frames),
    )
    return run, rover


class TestTheLoopCloses:
    def test_pixels_reach_the_wheels(self):
        """The one thing Stage 1 was for."""
        run, rover = drive(crop_rows(), max_frames=20)

        assert len(rover.drive_calls) == 20
        assert run.state is RoverState.DRIVING_ROW

    def test_the_rover_drives_forward_the_whole_way(self):
        _, rover = drive(crop_rows(), max_frames=20)

        assert len(rover.drive_calls) == 20
        assert all(v_mm_s > 0 for _, v_mm_s, _ in rover.drive_calls)

    def test_a_centred_furrow_is_driven_straight(self):
        _, rover = drive(crop_rows(), max_frames=10)

        assert len(rover.drive_calls) == 10
        assert all(omega == pytest.approx(0.0) for _, _, omega in rover.drive_calls)

    def test_a_furrow_to_the_right_turns_the_rover_right(self):
        """The sign convention survives all four hops: + lateral_err out of the
        histogram, through `RowEstimate`, through `omega = -(k_lat * lateral +
        ...)`, to a negative omega, which `row_follower` documents as a right
        turn.  A sign lost anywhere in that chain is a rover that steers away
        from the furrow, and no unit test on either side would see it."""
        _, rover = drive(crop_rows(near_u=210), max_frames=10)

        assert len(rover.drive_calls) == 10
        assert all(omega < 0 for _, _, omega in rover.drive_calls)

    def test_a_furrow_to_the_left_turns_the_rover_left(self):
        _, rover = drive(crop_rows(near_u=110), max_frames=10)

        assert len(rover.drive_calls) == 10
        assert all(omega > 0 for _, _, omega in rover.drive_calls)

    def test_bare_soil_from_the_first_frame_never_moves_the_rover(self):
        """Invalid from frame one: §5.5 commands nothing, the watchdog counts
        to three and brakes.  A loop that drove on a frame it could not read
        would show up here as a drive call."""
        run, rover = drive([bare_soil(seed) for seed in range(5)])

        assert rover.drive_calls == []
        assert run.state is RoverState.STOPPED


class TestAGapMidRow:
    def test_driving_past_missing_plants_does_not_end_the_run(self):
        """The acceptance criterion, closed-loop: a stretch of row with no
        plants sweeps up the frame as the rover approaches and passes it, and
        the run neither stops nor faults."""
        frames = [
            crop_rows(missing_left_rows=(top, top + 30))
            for top in range(SHAPE[0] - 30, -1, -20)
        ]
        run, rover = drive(frames)

        assert run.state is RoverState.DRIVING_ROW
        assert len(rover.drive_calls) == len(frames)


class TestReachingTheEndOfARow:
    def test_a_row_that_runs_out_stops_normally(self):
        run, rover = drive(receding_row(step=0.05))

        assert run.state is RoverState.STOPPED
        assert run.machine.stop_reason == ROW_END_SUSPECTED
        assert run.machine.error_code is None
        assert rover.calls[-1] == ("stop",)

    def test_the_same_row_ending_is_a_fault_when_it_is_approached_slowly(self):
        """⚠️ A measurement, reported and not tuned away (V1-ISAAC rule 7).

        Identical frames, identical ending, one fifth of the closing speed —
        and §5.4 reads it the other way round.  ``green_fraction_row_end`` is
        0.10 while the estimate goes invalid at 0.113, so the watchdog's three
        frames are what has to carry green_fraction across the remaining 0.013.
        Driven at 0.05 of a frame height per frame they do; driven at 0.01 they
        do not, and a normal end of a row is filed as ROW_LOST, an error state
        a person has to clear.

        The verdict therefore depends on rover speed, which is the one thing it
        must not depend on.  Whether 0.10 is simply too low, or the split wants
        something other than a bare threshold, needs rendered foliage to settle
        — `row_end.yaml` against `crop_gap_midrow.yaml`, the opposed pair that
        has to pass both ways.  `runaway.py` says the same from its end: what
        V0 tested was the comparison and the counting, never the number.

        This test asserts today's behaviour so the question cannot go quiet.
        When the threshold is settled it will fail, and this is the reason why.
        """
        run, _ = drive(receding_row(step=0.01))

        assert run.state is RoverState.ERROR
        assert run.machine.error_code is ErrorCode.ROW_LOST

    @pytest.mark.parametrize(
        "step, expected",
        [(0.01, "row_lost"), (0.02, "row_lost"), (0.03, "row_end_suspected"), (0.05, "row_end_suspected")],
    )
    def test_where_the_verdict_flips(self, step, expected):
        """The boundary, measured: between 0.02 and 0.03 of a frame height per
        frame.  Recorded as a number so that whatever settles the threshold can
        be checked against it rather than against an impression."""
        run, _ = drive(receding_row(step=step))
        verdict = run.machine.stop_reason or run.machine.error_code.value

        assert verdict == expected
