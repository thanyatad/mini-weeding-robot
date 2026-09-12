"""One row driven through the whole stack, with a hand on the cable.

    RowRun -> RowFollower -> Esp32Rover -> codec -> [wire] -> emulator
                                                                  |
       world state <- what the BOARD decided to command  <---------+
                |
                +-> FakeRowSensor -> RowEstimate -> back into RowRun

The rover the controller holds has no idea where it is, which is the point: the
world lives in the harness, the way it does in fake_loop.py, and it moves by
whatever the *board* ended up commanding rather than by what the controller
asked for.  So anything the link does to a command -- dropping it, overriding
it, refusing to obey it after a timeout -- shows up as the rover not moving,
which is the only honest way to test a link.

This is the closest thing to a controller loop that exists, and it is here in
the harness rather than in a test module because two callers need it: the
integration tests that drive it directly, and the scenario runner in
scenario.py.  Nothing under controller/ may import it -- it reads the world's
state, which is exactly what tests/unit/test_layering.py forbids on the control
path.

What the bench wires together is every detector the controller owns, in the
order a real loop would run them:

    RowRun.step()           drive, or count an invalid estimate
    board.tick()            the far side obeys, or gives up
    rover.poll()            telemetry in, acks settled, faults out
    EmergencyStop.tick()    the sense line, unguarded by state -- the button
                            is legal from everywhere and does not wait its turn
    FaultManager.handle()   a wire code becomes a workflow event
    LinkMonitor.tick()      the link age against its ceiling
"""

from __future__ import annotations

from bridge.protocol import Fault
from bridge.simulator import Esp32Emulator
from controller.config import get, load_config
from controller.motion import RowFollower
from controller.rover import Esp32Rover, FakeRover
from controller.safety import EmergencyStop, FaultManager, LinkMonitor, RowLossWatchdog
from controller.workflow import Event, RoverState, RowRun, StateMachine
from tests.harness.fake_loop import FakeRowSensor
from tests.harness.row import straight_row

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
        self.outbound = True

    def cut(self) -> None:
        self.connected = False

    def cut_outbound(self) -> None:
        """Break Pi -> board only, leaving board -> Pi intact.

        A broken TX line rather than an unplugged cable, and the two fail
        differently: with both directions gone the Pi hears nothing and has to
        work the fault out from link_age_ms, but with only the outbound half
        gone the board notices first and *says so*.  That is the path where an
        error message off the wire has to be turned into a transition, which is
        the one the bench used to fake.
        """
        self.outbound = False

    def write_line(self, line: str) -> None:
        if self.connected and self.outbound:
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
        self.fault_manager = FaultManager(self.run)
        self.estop = EmergencyStop(self.machine)

        self.loop_hz = float(get(self.config, "perception.loop_hz"))
        self._dt_ms = 1000.0 / self.loop_hz

        self.faults: list[Fault] = []
        #: (now_ms, what) for the two events whose order is the point.
        self.motors_died_at_ms: float | None = None
        self.link_lost_at_ms: float | None = None
        self.estop_fired_at_ms: float | None = None

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

        # Before the faults and outside the driving guard, because the button
        # is legal from every state and does not wait its turn.  By the time
        # this reads the line the rail has already been cut electrically
        # (§11.4); what is left is for the workflow to find out.
        if self.estop.tick(self.rover.get_drive_state()["estop"]) is Event.ESTOP:
            self.estop_fired_at_ms = self.now_ms

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
        """The shipped router, not a copy of it.

        The state check stays here rather than moving into FaultManager: which
        faults a loop is still interested in is the loop's business, and a
        router that skipped faults on its own would be a second place deciding.
        What the bench no longer owns is the translation — Event(fault.code),
        which silently does nothing for the one code whose string and event are
        spelled differently.
        """
        for fault in self.faults:
            if self.state is RoverState.DRIVING_ROW:
                self.fault_manager.handle(fault)
        self.faults.clear()

    def drive_row(self, max_rounds: int = 400) -> Bench:
        for _ in range(max_rounds):
            if self.state is not RoverState.DRIVING_ROW:
                break
            self.round()
        return self
