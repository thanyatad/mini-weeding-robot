"""A row driven through the whole stack: Controller + Bridge + ESP32 emulator.

    RowRun -> RowFollower -> Esp32Rover -> codec -> [wire] -> emulator
                                                                  |
       world pose <- what the BOARD decided to command  <----------+
                |
                +-> FakeRowSensor -> RowEstimate -> back into RowRun

The rover the controller holds has no idea where it is, which is the point: the
pose lives in the harness, the way it does in tests/harness/fake_loop.py, and
the world moves by whatever the *board* ended up commanding rather than by what
the controller asked for.  So anything the link does to a command -- dropping
it, overriding it, refusing to obey it after a timeout -- shows up as the rover
not moving, which is the only honest way to test a link.

Two endings, and the second is the one that matters:

* the row runs out and the run ends in STOPPED(row_end_suspected), with the
  discrete stop acked over the same wire
* the link is pulled mid-row and the run ends in ERROR(link_lost), with the
  board's motors already dead before the controller says a word -- that is the
  300 < 500 ordering, observed rather than asserted from the config file
"""

from __future__ import annotations

import re

import pytest

from controller.config import get
from controller.workflow import ROW_END_SUSPECTED, ErrorCode, RoverState
from tests.harness.bench import BED_LENGTH_MM, Bench


def test_a_row_driven_over_the_wire_ends_in_stopped_row_end_suspected():
    bench = Bench().drive_row()

    assert bench.state is RoverState.STOPPED
    assert bench.machine.stop_reason == ROW_END_SUSPECTED
    assert bench.machine.error_code is None


def test_the_rover_actually_drove_the_row():
    """The controller's commands reached the motors through the codec, the
    wire and latest-wins -- a loop that stopped at the first round would land
    in STOPPED too, at 0 mm."""
    bench = Bench().drive_row()

    # Not the full 2000 mm: FakeRowSensor samples the furrow 360 mm ahead, so
    # the estimate goes invalid a lookahead short of the end and the watchdog
    # trips three frames after that.  The harness ends a row early by
    # construction — what is under test here is that the rover got there.
    assert bench.distance_travelled_mm > 1500.0
    assert bench.board.drives_applied > 100
    assert bench.board.malformed_lines == 0


def test_the_watchdog_s_stop_is_acked_over_the_same_wire():
    """The discrete half of the protocol, end to end.

    The brake that ends the row is a command that must be answered, and it is
    answered inside the same round: the board reads the buffer after the
    controller has written to it, and the ack is waiting when the Pi next
    reads.  Nothing is left pending, and nothing ever reaches its deadline.
    """
    bench = Bench().drive_row()

    assert ("stop", 1) in bench.board.discrete_commands, "the run ended without braking"
    assert bench.rover.pending == ()
    assert bench.faults == [], "an answered command produces no fault"


def test_no_error_is_produced_by_a_healthy_row():
    bench = Bench().drive_row()

    assert bench.faults == []
    assert bench.link_lost_at_ms is None


def test_link_age_stays_far_below_the_ceiling_while_the_link_is_healthy():
    bench = Bench()
    for _ in range(50):
        bench.round()

    link_lost_ms = get(bench.config, "safety.link_lost_ms")
    assert bench.rover.get_drive_state()["link_age_ms"] < link_lost_ms / 2


# -- the link goes away mid-row ---------------------------------------------


def test_pulling_the_link_mid_row_ends_in_error_link_lost():
    bench = Bench()
    for _ in range(30):
        bench.round()
    assert bench.state is RoverState.DRIVING_ROW
    cut_at_mm = bench.distance_travelled_mm

    bench.wire.cut()
    bench.drive_row(max_rounds=50)

    assert bench.state is RoverState.ERROR
    assert bench.machine.error_code is ErrorCode.LINK_LOST
    assert bench.distance_travelled_mm > cut_at_mm, "it was still mid-row"
    assert bench.distance_travelled_mm < BED_LENGTH_MM, "the row did not end -- the link did"


def test_the_board_kills_the_motors_before_the_controller_declares_the_fault():
    """command_timeout_ms (300) < link_lost_ms (500), observed rather than read
    off the config file.  Reversed, the controller would enter ERROR while the
    wheels turned for another 200 ms, and the log would read backwards."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.wire.cut()
    bench.drive_row(max_rounds=50)

    assert bench.motors_died_at_ms is not None
    assert bench.link_lost_at_ms is not None
    assert bench.motors_died_at_ms < bench.link_lost_at_ms


def test_the_rover_stops_moving_once_the_board_gives_up():
    """The controller goes on sending drives into a dead wire the whole time.
    The rover stops because the board stopped obeying, which is the layer that
    does not depend on the Pi being alive (§5.6)."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.wire.cut()
    bench.drive_row(max_rounds=50)
    distance_at_error = bench.distance_travelled_mm

    for _ in range(20):
        bench.round()

    assert bench.board.commanded == {"v_mm_s": 0.0, "omega_deg_s": 0.0}
    assert bench.distance_travelled_mm == pytest.approx(distance_at_error)


def test_the_controller_tried_to_brake_and_the_command_went_unanswered():
    """report_fault() calls stop() for link_lost.  Down a cut wire it cannot be
    acked, so it is still pending -- which is the codec noticing what the
    monitor could not."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.wire.cut()
    bench.drive_row(max_rounds=50)

    assert bench.rover.pending, "nothing was sent to stop the rover"


def test_a_reconnected_board_that_rebooted_is_not_mistaken_for_a_healthy_one():
    """last_seq restarts at 0 after a reboot, so the link age is measured from
    the first frame of the session rather than from a seq the board no longer
    knows about.  A tracker that answered zero here would report a link that
    has been down for seconds as fresh."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.board.reboot(now_ms=bench.now_ms)
    bench.round()

    assert bench.rover.get_drive_state()["link_age_ms"] > 0


# -- the board reports a fault the Pi can still hear ------------------------


def test_a_broken_outbound_line_ends_in_error_command_timeout():
    """The board gives up at 300 ms and reports it, and the report is what ends
    the run -- no link monitor involved, because link_age_ms is only at 300 too
    and its ceiling is 500."""
    bench = Bench()
    for _ in range(30):
        bench.round()
    assert bench.state is RoverState.DRIVING_ROW

    bench.wire.cut_outbound()
    bench.drive_row(max_rounds=50)

    assert bench.state is RoverState.ERROR
    assert bench.machine.error_code is ErrorCode.COMMAND_TIMEOUT
    assert bench.link_lost_at_ms is None, "the board got there first"


def test_the_board_s_error_reaches_the_workflow_through_the_fault_manager():
    """The translation the bench used to do inline.  It is worth an assertion
    of its own because Event(fault.code) happens to work for this code -- and
    silently does nothing for emergency_stop, which is spelled differently."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.wire.cut_outbound()
    bench.drive_row(max_rounds=50)

    assert [fault.code for fault in bench.fault_manager.history] == ["command_timeout"]

    # The message is the half of a Fault that does not survive becoming an
    # Event, and protocol/messages.md asks it to carry the numbers that
    # identify the fault.  The board rounds up to its own loop period, so what
    # is pinned is that the elapsed is real and past the deadline -- not a
    # literal 300, which would be asserting the tick rate.
    message = bench.fault_manager.history[0].message
    elapsed_ms = float(re.search(r"(\d+) ms", message).group(1))
    assert elapsed_ms > get(bench.config, "safety.command_timeout_ms")


def test_nothing_is_commanded_on_the_way_into_command_timeout():
    """The motors are already dead, so there is nothing to stop and nothing to
    say to the board (§8.5).  report_fault() owns that rule; routing through
    the fault manager must not have quietly restored the brake."""
    bench = Bench()
    for _ in range(30):
        bench.round()

    bench.wire.cut_outbound()
    bench.drive_row(max_rounds=50)

    assert bench.board.motors_enabled is False
    assert ("stop", 1) not in bench.board.discrete_commands
    assert bench.rover.pending == ()
