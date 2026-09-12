"""The row-loss watchdog: the controller's layer of the three in §5.6.

Counting is the safety-critical half.  Startup invariant 2 is already enforced
against ``row_loss_frames``::

    v_max x row_loss_frames / loop_hz <= runaway_budget
    100  x 3                / 10      = 30 <= 60

so a watchdog that trips one frame late turns that invariant into a description
of a runaway distance the code does not honour — and nothing else in the system
is in a position to notice.  Hence a test on each of the three frames rather
than one test that it eventually trips.

The *direction* of the trip is tested against an explicit threshold passed in.
What is NOT tested here, and cannot be at V0, is whether the configured number
actually separates a gap mid-row from the end of a row: that needs rendered
images with crops in them, which is V1/V2.  The logic is proven; the value is
not.
"""

import pytest

from controller.config import get, load_config, with_overrides
from controller.motion import RowEstimate
from controller.safety import RowLossVerdict, RowLossWatchdog
from tests.harness import RecordingRover

#: A threshold chosen by this file, not read from config, so the comparison is
#: tested without asserting anything about the configured value.
TRIP_AT = 0.30


def invalid(green_fraction=0.0) -> RowEstimate:
    """An estimate the row estimator could not fit a line to.

    Constructed honestly: RowEstimate is frozen and range-validated, so the
    zeros here are real zeros and green_fraction is inside [0, 1].
    """
    return RowEstimate(
        valid=False,
        lateral_err=0.0,
        heading_err=0.0,
        confidence=0.0,
        green_fraction=green_fraction,
    )


def valid(green_fraction=0.4) -> RowEstimate:
    return RowEstimate(
        valid=True,
        lateral_err=0.1,
        heading_err=0.0,
        confidence=0.8,
        green_fraction=green_fraction,
    )


@pytest.fixture
def rover():
    return RecordingRover()


@pytest.fixture
def watchdog(rover):
    return RowLossWatchdog(rover, row_loss_frames=3, green_fraction_trip=TRIP_AT)


class TestCounting:
    def test_the_first_invalid_frame_does_not_trip(self, watchdog):
        assert watchdog.tick(invalid()) is None

    def test_the_second_invalid_frame_does_not_trip(self, watchdog):
        watchdog.tick(invalid())
        assert watchdog.tick(invalid()) is None

    def test_the_third_consecutive_invalid_frame_trips(self, watchdog):
        watchdog.tick(invalid())
        watchdog.tick(invalid())
        assert watchdog.tick(invalid()) is not None

    def test_a_valid_frame_in_between_resets_the_count_to_zero(self, watchdog):
        """Two invalid, one valid, two invalid: five invalid-ish frames and no
        trip, because they were never three in a row.  This is the whole
        difference between a gap mid-row and the end of a row."""
        watchdog.tick(invalid())
        watchdog.tick(invalid())
        watchdog.tick(valid())
        assert watchdog.consecutive_invalid == 0

        assert watchdog.tick(invalid()) is None
        assert watchdog.tick(invalid()) is None
        assert watchdog.tick(invalid()) is not None

    def test_reset_zeroes_the_count(self, watchdog):
        """§5.5 calls reset() on every valid estimate rather than tick()."""
        watchdog.tick(invalid())
        watchdog.tick(invalid())
        watchdog.reset()

        assert watchdog.consecutive_invalid == 0
        assert watchdog.tick(invalid()) is None

    def test_a_valid_estimate_handed_to_tick_never_trips(self, watchdog):
        for _ in range(10):
            assert watchdog.tick(valid()) is None

    def test_the_count_is_readable_while_it_climbs(self, watchdog):
        assert watchdog.consecutive_invalid == 0
        watchdog.tick(invalid())
        assert watchdog.consecutive_invalid == 1
        watchdog.tick(invalid())
        assert watchdog.consecutive_invalid == 2

    def test_a_watchdog_that_could_never_trip_is_rejected(self, rover):
        with pytest.raises(ValueError, match="row_loss_frames"):
            RowLossWatchdog(rover, row_loss_frames=0, green_fraction_trip=TRIP_AT)


class TestWhichWayItTrips:
    """§5.4: both outcomes are 'invalid for row_loss_frames in a row', and
    green_fraction is the only thing that tells them apart."""

    def test_green_gone_reads_as_the_end_of_the_row(self, watchdog):
        for _ in range(3):
            verdict = watchdog.tick(invalid(green_fraction=0.0))
        assert verdict is RowLossVerdict.ROW_END_SUSPECTED

    def test_green_still_there_reads_as_a_navigation_failure(self, watchdog):
        for _ in range(3):
            verdict = watchdog.tick(invalid(green_fraction=0.9))
        assert verdict is RowLossVerdict.ROW_LOST

    def test_exactly_at_the_threshold_is_treated_as_lost(self, watchdog):
        """The ambiguous case is reported as the fault.  A false 'end of row'
        is a navigation failure filed as a normal stop, which is the direction
        that hides the problem."""
        for _ in range(3):
            verdict = watchdog.tick(invalid(green_fraction=TRIP_AT))
        assert verdict is RowLossVerdict.ROW_LOST

    def test_the_frame_that_trips_is_the_one_that_decides(self, watchdog):
        """The verdict reads green_fraction from the frame that ran the counter
        out, which is the frame §5.4 describes."""
        watchdog.tick(invalid(green_fraction=0.9))
        watchdog.tick(invalid(green_fraction=0.9))
        assert watchdog.tick(invalid(green_fraction=0.0)) is RowLossVerdict.ROW_END_SUSPECTED

    def test_the_verdict_names_match_the_scenario_contract(self):
        assert RowLossVerdict.ROW_END_SUSPECTED.value == "row_end_suspected"
        assert RowLossVerdict.ROW_LOST.value == "row_lost"


class TestItHaltsTheRover:
    def test_it_brakes_rather_than_coasting(self, watchdog, rover):
        """stop() brakes and holds; drive(0, 0) coasts.  Both refresh the
        firmware watchdog, but on a slope they are not the same thing
        (docs/architecture.md).  Asserted on a real rover, not a mock."""
        for _ in range(3):
            watchdog.tick(invalid())

        assert rover.calls == [("stop",)]

    def test_it_commands_nothing_before_it_trips(self, watchdog, rover):
        """The two frames before the trip are the runaway budget being spent on
        purpose.  Commanding anything there would be the watchdog second-
        guessing the number invariant 2 is enforced against."""
        watchdog.tick(invalid())
        watchdog.tick(invalid())

        assert rover.calls == []

    def test_it_stops_once_not_once_per_frame(self, watchdog, rover):
        for _ in range(6):
            watchdog.tick(invalid())

        assert rover.calls == [("stop",)]

    def test_a_reset_arms_it_again(self, watchdog, rover):
        for _ in range(3):
            watchdog.tick(invalid())
        watchdog.reset()
        for _ in range(3):
            watchdog.tick(invalid())

        assert rover.calls == [("stop",), ("stop",)]


class TestItReadsTheConfiguredNumbers:
    def test_row_loss_frames_comes_from_config(self, rover):
        config = load_config()
        watchdog = RowLossWatchdog.from_config(rover, config)

        assert watchdog.row_loss_frames == get(config, "safety.row_loss_frames")
        assert watchdog.row_loss_frames == 3

    def test_changing_row_loss_frames_moves_the_trip(self, rover):
        """Proof that the number is read rather than pasted in: at 5 the third
        frame must no longer trip, and the fifth must."""
        config = with_overrides(load_config(), {"safety.row_loss_frames": 5})
        watchdog = RowLossWatchdog.from_config(rover, config)

        assert [watchdog.tick(invalid()) is not None for _ in range(5)] == [
            False,
            False,
            False,
            False,
            True,
        ]

    def test_the_green_fraction_threshold_comes_from_config(self, rover):
        config = load_config()
        watchdog = RowLossWatchdog.from_config(rover, config)

        assert watchdog.green_fraction_trip == get(
            config, "perception.row_estimator.green_fraction_row_end"
        )

    def test_changing_the_threshold_moves_the_verdict(self, rover):
        """The same 0.2 frame reads as the end of the row under one threshold
        and as a lost row under another.  This proves the comparison is wired
        to config; it proves nothing about whether either number is right."""
        low = RowLossWatchdog.from_config(
            rover,
            with_overrides(load_config(), {"perception.row_estimator.green_fraction_row_end": 0.1}),
        )
        high = RowLossWatchdog.from_config(
            rover,
            with_overrides(load_config(), {"perception.row_estimator.green_fraction_row_end": 0.5}),
        )

        for _ in range(3):
            low_verdict = low.tick(invalid(green_fraction=0.2))
            high_verdict = high.tick(invalid(green_fraction=0.2))

        assert low_verdict is RowLossVerdict.ROW_LOST
        assert high_verdict is RowLossVerdict.ROW_END_SUSPECTED


class TestItStandsAlone:
    """§5.6: 'if a layer depends on the layer above it, it is not a safety
    layer, it is a feature.'  The controller owns the middle layer only."""

    def test_it_does_not_reach_for_the_firmware_or_the_hardware_layer(self, watchdog, rover):
        """It never calls emergency_stop(): cutting the motor rail is the
        hardware layer's job, and a controller that latches it in software has
        taken over a layer it does not own."""
        for _ in range(6):
            watchdog.tick(invalid())

        assert ("emergency_stop",) not in rover.calls

    def test_it_needs_nothing_but_estimates_and_a_rover(self, rover):
        """No clock, no camera, no link state.  It counts what it is handed."""
        watchdog = RowLossWatchdog(rover, row_loss_frames=3, green_fraction_trip=TRIP_AT)
        for _ in range(3):
            watchdog.tick(invalid())

        assert rover.calls == [("stop",)]
