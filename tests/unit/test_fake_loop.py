"""The V0 closed loop: does this gain set converge, and does it swing out of
the furrow on the way?

Runs in milliseconds with no GPU, no Isaac, no images and no physics, which is
the whole reason FakeRover and FakeRowSensor are shaped the way they are.

The gains under test are the ones in config/control.yaml, loaded — not
constants pasted in here.  The day someone retunes them, this test is what says
whether the new pair still keeps the rover inside the furrow.
"""

import math

import pytest

from controller.config import get, load_config, with_overrides
from controller.motion import RowEstimate
from controller.rover import Pose
from controller.workflow import ROW_END_SUSPECTED, ErrorCode, RoverState
from tests.harness import FakeLoop, FakeRowSensor, straight_row


@pytest.fixture
def config():
    return load_config()


class TestConvergence:
    def test_converges_from_offset(self, config):
        """The acceptance case: 60 mm off-centre and 15 deg skewed *away* from
        the furrow, so the rover has to arrest a divergence first."""
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
        sim.run(seconds=3.0)

        assert abs(sim.history[-1].lateral_err) < 0.1
        assert sim.min_clearance_mm > 0

    def test_it_really_ran_for_three_seconds(self):
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
        sim.run(seconds=3.0)

        assert sim.history[0].t_s == 0.0
        assert sim.history[-1].t_s == pytest.approx(3.0)
        assert len(sim.history) == 31  # 10 Hz, plus the starting sample

    def test_the_estimate_stays_valid_throughout(self):
        """A run that converged because the furrow left the frame and the rover
        coasted to a stop has proved nothing."""
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
        sim.run(seconds=3.0)

        assert all(sample.valid for sample in sim.history)
        assert all(sample.v_mm_s == 100.0 for sample in sim.history[1:])

    def test_the_furrow_is_actually_reached_not_just_aimed_at(self):
        """lateral_err is a lookahead measurement: it goes to zero as soon as
        the rover *points* at the furrow, which happens well before it is in
        it.  Given longer, the offset itself has to go to zero too."""
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
        sim.run(seconds=10.0)

        assert abs(sim.history[-1].offset_mm) < 5.0
        assert abs(sim.history[-1].heading_deg) < 2.0

    def test_it_settles_instead_of_oscillating(self):
        """k_head earning its place: no sign-flipping hunt around the furrow."""
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
        sim.run(seconds=10.0)

        late = [sample.offset_mm for sample in sim.history if sample.t_s >= 6.0]
        assert max(abs(value) for value in late) < 10.0

    def test_the_mirror_image_converges_identically(self):
        """Nothing in the loop may prefer one side of the furrow."""
        left = FakeLoop(row=straight_row(), start_offset_mm=+60, start_heading_deg=+15)
        right = FakeLoop(row=straight_row(), start_offset_mm=-60, start_heading_deg=-15)
        left.run(seconds=3.0)
        right.run(seconds=3.0)

        assert right.history[-1].lateral_err == pytest.approx(-left.history[-1].lateral_err)
        assert right.history[-1].offset_mm == pytest.approx(-left.history[-1].offset_mm)
        assert right.min_clearance_mm == pytest.approx(left.min_clearance_mm)

    def test_a_pure_heading_error_is_corrected(self):
        sim = FakeLoop(row=straight_row(), start_offset_mm=0, start_heading_deg=15)
        sim.run(seconds=3.0)

        assert abs(sim.history[-1].lateral_err) < 0.1
        assert sim.min_clearance_mm > 0

    def test_starting_on_the_furrow_stays_on_it(self):
        sim = FakeLoop(row=straight_row(), start_offset_mm=0, start_heading_deg=0)
        sim.run(seconds=3.0)

        assert sim.max_lateral_offset_mm == pytest.approx(0.0, abs=1e-9)
        assert all(sample.omega_deg_s == 0.0 for sample in sim.history)


class TestClearance:
    def test_clearance_is_measured_against_foliage_not_row_spacing(self, config):
        """Leaves take 30 mm off each side of the furrow.  A clearance computed
        from row_spacing would report 30 mm of room the rover does not have."""
        sim = FakeLoop(row=straight_row(), start_offset_mm=0, start_heading_deg=0)
        sim.run(seconds=0.1)

        clear_furrow = get(config, "bed.row_spacing_mm") - 2 * get(
            config, "bed.crop_foliage_half_width_mm"
        )
        expected = (clear_furrow - get(config, "rover.body_width_mm")) / 2
        assert sim.history[0].clearance_mm == pytest.approx(expected)
        assert expected == pytest.approx(85.0)

    def test_a_run_that_leaves_the_furrow_reports_negative_clearance(self):
        """The check has to be able to fail, or min_clearance_mm > 0 proves
        nothing.  Gains of zero give a rover that drives straight out."""
        config = with_overrides(
            load_config(),
            {"row_follower.gains.fake.k_lat": 0.0, "row_follower.gains.fake.k_head": 0.0},
        )
        sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15, config=config)
        sim.run(seconds=3.0)

        assert sim.min_clearance_mm < 0
        assert sim.max_lateral_offset_mm > 72.0  # past the foliage, into the crops
        # It does not even end with a usable estimate: the furrow has left the frame.
        assert not sim.history[-1].valid


class TestFakeRowSensor:
    """The sensor is the part of the harness that can quietly flatter the
    controller, so its signs and its scale are pinned down here."""

    def test_a_rover_left_of_the_furrow_sees_it_to_the_right(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=+60.0, heading_deg=0.0))
        assert est.valid
        assert est.lateral_err > 0

    def test_a_rover_right_of_the_furrow_sees_it_to_the_left(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=-60.0, heading_deg=0.0))
        assert est.lateral_err < 0

    def test_a_rover_pointing_left_sees_the_furrow_tilt_right(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=0.0, heading_deg=+15.0))
        assert est.lateral_err > 0
        assert est.heading_err > 0

    def test_on_the_furrow_and_aligned_reads_zero(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=0.0, heading_deg=0.0))
        assert est.lateral_err == pytest.approx(0.0)
        assert est.heading_err == pytest.approx(0.0)

    def test_the_furrow_position_is_read_at_the_lookahead_not_underfoot(self):
        """The camera looks 180 mm ahead.  A sensor reading the offset under the
        rover would have no lookahead at all, and the gains would mean something
        else entirely."""
        sensor = FakeRowSensor(straight_row())
        offset = sensor._furrow_lateral_offset_mm(Pose(x_mm=0.0, y_mm=0.0, heading_deg=15.0), 180.0)
        assert offset == pytest.approx(-180.0 * math.tan(math.radians(15.0)))

    def test_the_estimate_goes_invalid_when_the_furrow_leaves_the_frame(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=0.0, heading_deg=80.0))
        assert not est.valid

    def test_it_reports_no_units_it_cannot_back_up(self):
        sensor = FakeRowSensor(straight_row())
        est = sensor.estimate(Pose(x_mm=0.0, y_mm=30.0, heading_deg=0.0))
        assert isinstance(est, RowEstimate)
        assert -1.0 <= est.lateral_err <= 1.0
        assert -1.0 <= est.heading_err <= 1.0


class TestHarnessUsesRealConfig:
    def test_the_loop_runs_at_the_configured_rate(self, config):
        sim = FakeLoop(row=straight_row())
        assert sim.loop_hz == get(config, "perception.loop_hz")

    def test_the_loop_uses_the_fake_backend_gains(self, config):
        sim = FakeLoop(row=straight_row())
        assert sim.follower.k_lat == get(config, "row_follower.gains.fake.k_lat")
        assert sim.follower.k_head == get(config, "row_follower.gains.fake.k_head")

    def test_the_loop_refuses_a_backend_with_untuned_gains(self):
        with pytest.raises(ValueError, match="esp32"):
            FakeLoop(row=straight_row(), backend="esp32")


class TestARowThatEnds:
    """A row with a length, so the closed loop can reach the end of one.

    `straight_row()` runs forever, which is right for a convergence test and
    useless for an ending.  Bounding it is the only thing the harness needs in
    order to produce the invalid estimates the watchdog counts: past the end
    there is simply no furrow to sample.
    """

    def test_an_unbounded_row_never_runs_out(self):
        sensor = FakeRowSensor(straight_row())
        assert sensor.estimate(Pose(x_mm=10_000.0, y_mm=0.0, heading_deg=0.0)).valid

    def test_the_estimate_is_valid_well_inside_a_bounded_row(self):
        sensor = FakeRowSensor(straight_row(length_mm=1000.0))
        assert sensor.estimate(Pose(x_mm=100.0, y_mm=0.0, heading_deg=0.0)).valid

    def test_the_estimate_goes_invalid_before_the_end_is_reached(self):
        """The camera looks ahead, so the furrow leaves the frame while the
        rover is still on it — FAR_LOOKAHEAD_MM short of the end."""
        sensor = FakeRowSensor(straight_row(length_mm=1000.0))
        assert not sensor.estimate(Pose(x_mm=900.0, y_mm=0.0, heading_deg=0.0)).valid

    def test_a_row_carries_its_length(self):
        assert straight_row().length_mm is None
        assert straight_row(length_mm=2000.0).length_mm == 2000.0


class TestGreenFractionIsDeclaredNotMeasured:
    """The harness has no pixels.  green_fraction is a number the caller states
    in order to stage one of the two endings of §5.4 — not something counted
    off an image, and it must not start looking like one."""

    def test_it_is_constant_wherever_the_rover_is(self):
        sensor = FakeRowSensor(straight_row())
        readings = {
            sensor.estimate(Pose(x_mm=x, y_mm=20.0, heading_deg=5.0)).green_fraction
            for x in (0.0, 100.0, 500.0)
        }
        assert readings == {0.4}

    def test_the_caller_states_what_an_invalid_frame_claims(self):
        sensor = FakeRowSensor(straight_row(length_mm=500.0), green_fraction_invalid=0.9)
        est = sensor.estimate(Pose(x_mm=450.0, y_mm=0.0, heading_deg=0.0))

        assert not est.valid
        assert est.green_fraction == 0.9

    def test_the_default_invalid_frame_claims_no_green(self):
        sensor = FakeRowSensor(straight_row(length_mm=500.0))
        est = sensor.estimate(Pose(x_mm=450.0, y_mm=0.0, heading_deg=0.0))

        assert not est.valid
        assert est.green_fraction == 0.0


class TestTheLoopRunsTheRealWorkflow:
    """FakeLoop drives the actual RowRun, watchdog and state machine — not a
    stand-in for them.  A harness that stops the rover its own way would be
    testing the harness."""

    def test_it_starts_in_the_row_having_passed_validation(self):
        sim = FakeLoop(row=straight_row())
        assert sim.state is RoverState.DRIVING_ROW

    def test_a_run_that_never_ends_is_still_in_the_row(self):
        sim = FakeLoop(row=straight_row())
        sim.run(seconds=3.0)

        assert sim.state is RoverState.DRIVING_ROW
        assert sim.stop_reason is None
        assert sim.error_code is None

    def test_the_end_of_the_row_stops_the_run_normally(self):
        """row_end.yaml, closed loop: green gone, so this is the end of the row
        and not a fault."""
        sim = FakeLoop(row=straight_row(length_mm=1000.0))
        sim.run(seconds=30.0)

        assert sim.state is RoverState.STOPPED
        assert sim.stop_reason == ROW_END_SUSPECTED
        assert sim.error_code is None

    def test_losing_the_row_while_green_remains_is_a_fault(self):
        """row_lost.yaml, closed loop: the same geometry and the same three
        invalid frames, and only green_fraction differs."""
        row = straight_row(length_mm=1000.0)
        sim = FakeLoop(row=row, sensor=FakeRowSensor(row, green_fraction_invalid=0.9))
        sim.run(seconds=30.0)

        assert sim.state is RoverState.ERROR
        assert sim.error_code is ErrorCode.ROW_LOST
        assert sim.stop_reason is None

    def test_it_stops_driving_once_the_run_has_ended(self):
        sim = FakeLoop(row=straight_row(length_mm=1000.0))
        sim.run(seconds=30.0)

        assert sim.history[-1].v_mm_s == 0.0
        assert sim.history[-1].omega_deg_s == 0.0

    def test_it_does_not_stop_at_the_first_invalid_frame(self):
        """row_loss_frames is 3.  A harness that stopped on the first one would
        make crop_gap_midrow untestable, because every gap would end the run."""
        sim = FakeLoop(row=straight_row(length_mm=1000.0))
        sim.run(seconds=30.0)

        invalid_samples = [sample for sample in sim.history if not sample.valid]
        assert len(invalid_samples) >= 3

    def test_it_ran_most_of_the_row_before_stopping(self):
        """The stop belongs at the end of the row, not somewhere in the middle."""
        sim = FakeLoop(row=straight_row(length_mm=1000.0))
        sim.run(seconds=30.0)

        assert sim.distance_travelled_mm > 500.0
