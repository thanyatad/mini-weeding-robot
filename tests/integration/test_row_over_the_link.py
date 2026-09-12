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

import pytest

from bridge.protocol import Fault
from bridge.simulator import Esp32Emulator
from controller.config import get, load_config
from controller.motion import RowFollower
from controller.rover import Esp32Rover, FakeRover
from controller.safety import LinkMonitor, RowLossWatchdog
from controller.workflow import (
    ROW_END_SUSPECTED,
    ErrorCode,
    Event,
    RoverState,
    RowRun,
    StateMachine,
)
from controller.workflow.row_run import FAULT_EVENTS
from tests.harness import FakeRowSensor, straight_row

BED_LENGTH_MM = 2000.0


class CutWire:
    """The serial cable, and a hand to pull it out.

    Both directions go at once, because that is what unplugging does.  The
    board on the far side stays alive and keeps running its own loop -- which
    is the whole reason the ESP32 has a command timeout of its own.
    """

    def __init__(self, board: Esp32Emulator) -> None:
        self._board = board
        self.connected = True

    def cut(self) -> None:
        self.connected = False

    def write_line(self, line: str) -> None:
        if self.connected:
            self._board.write_line(line)

    def read_lines(self) -> list[str]:
        lines = self._board.read_lines()
        return lines if self.connected else []


class Bench:
    """One row, driven over the wire, with the harness playing the world."""

    def __init__(self, row_length_mm: float = BED_LENGTH_MM) -> None:
        self.config = load_config()
        self.board = Esp32Emulator.from_config(self.config)
        self.wire = CutWire(self.board)

        self.now_ms = 0.0
        self.rover = Esp32Rover.from_config(self.config, link=self.wire, clock=lambda: self.now_ms)

        # The world: a kinematic integrator, exactly as fake_loop.py uses it.
        # It is not the rover under test -- it is the ground the rover rolls on.
        self.world = FakeRover(x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        self.sensor = FakeRowSensor(straight_row(length_mm=row_length_mm))

        self.machine = StateMachine()
        self.run = RowRun(
            machine=self.machine,
            rover=self.rover,
            # The fake gains: these are the ones that are tuned.  control.yaml
            # keeps the esp32 gains null on purpose, and invariant 8 fails a
            # real run that tries to use them -- the emulator is not the real
            # motors, and pretending otherwise is how an untuned gain reaches a
            # field.
            follower=RowFollower.from_config(self.config, backend="fake"),
            watchdog=RowLossWatchdog.from_config(self.rover, self.config),
        )
        self.monitor = LinkMonitor.from_config(self.run, self.config)

        self.loop_hz = float(get(self.config, "perception.loop_hz"))
        self._dt_ms = 1000.0 / self.loop_hz

        self.faults: list[Fault] = []
        #: (now_ms, what) for the two events whose order is the point.
        self.motors_died_at_ms: float | None = None
        self.link_lost_at_ms: float | None = None

        self.machine.fire(Event.VALIDATE_OK)
        self.run.start()

    @property
    def state(self) -> RoverState:
        return self.machine.state

    @property
    def distance_travelled_mm(self) -> float:
        return self.world.pose.x_mm

    def round(self) -> None:
        """One turn of §5.5, with a board and a wire in the middle."""
        driving = self.state is RoverState.DRIVING_ROW
        if driving:
            self.run.step(self.sensor.estimate(self.world.pose))

        # The board reads everything the round put in its buffer, applies
        # latest-wins, and answers.
        self.board.tick(self.now_ms)
        self.faults.extend(self.rover.poll(self.now_ms))

        if self.motors_died_at_ms is None and not self.board.motors_enabled:
            self.motors_died_at_ms = self.now_ms

        if driving and self.state is RoverState.DRIVING_ROW:
            # Faults are routed only while the run is driving: report_fault()
            # would otherwise be handed a fault for a run the state machine has
            # already finished with, and it is the machine's job to refuse it.
            self._route_faults()
            if self.monitor.tick(self.rover.get_drive_state()["link_age_ms"]):
                self.link_lost_at_ms = self.now_ms

        # The world moves by what the board decided, not by what was asked.
        self.world.drive(**self.board.commanded)
        self.world.step(self._dt_ms / 1000.0)
        self.now_ms += self._dt_ms

    def _route_faults(self) -> None:
        for fault in self.faults:
            event = Event(fault.code)
            if event in FAULT_EVENTS and self.state is RoverState.DRIVING_ROW:
                self.run.report_fault(event)
        self.faults.clear()

    def drive_row(self, max_rounds: int = 400) -> Bench:
        for _ in range(max_rounds):
            if self.state is not RoverState.DRIVING_ROW:
                break
            self.round()
        return self


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
