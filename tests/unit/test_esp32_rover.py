"""The ESP32 backend, with and without a board attached.

Two things are being protected here.  One is that the class stays constructible
with nothing plugged in — ``tests/unit/test_rover_backends.py`` builds all three
backends to compare their drive-state key sets, and that has to run on a laptop
with no rover on the desk.  The other is that nothing about a missing or a lying
link is quietly rounded to zero: a backend that answered "link age 0" when it
could not tell would be reporting a healthy link at the moment it had least
right to.
"""

import pytest

from bridge.protocol import encode
from bridge.seq_tracker import UnknownSeq
from bridge.simulator import Esp32Emulator
from controller.config import load_config
from controller.rover import Esp32Rover, FakeRover, IsaacRover


class Clock:
    """A hand-wound clock, so a deadline is a number and not a sleep."""

    def __init__(self) -> None:
        self.now_ms = 0.0

    def __call__(self) -> float:
        return self.now_ms


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def board():
    return Esp32Emulator()


@pytest.fixture
def rover(board, clock):
    return Esp32Rover.from_config(load_config(), link=board, clock=clock)


# -- with no board attached -------------------------------------------------


def test_it_constructs_with_no_board_attached():
    """No port opened, no config file read, no board present."""
    assert Esp32Rover().get_drive_state()["link_age_ms"] == 0


def test_the_drive_state_shape_matches_the_other_backends():
    shapes = {frozenset(r.get_drive_state()) for r in (FakeRover(), IsaacRover(), Esp32Rover())}

    assert len(shapes) == 1


@pytest.mark.parametrize("call", ["drive", "stop", "emergency_stop", "poll"])
def test_everything_that_needs_a_board_refuses_without_one(call):
    rover = Esp32Rover()
    args = (100.0, 0.0) if call == "drive" else ()

    with pytest.raises(NotImplementedError):
        getattr(rover, call)(*args)


def test_a_link_without_a_codec_is_refused_at_construction():
    """from_config builds all three together.  Half a backend would fail later,
    somewhere less obvious."""
    with pytest.raises(ValueError, match="from_config"):
        Esp32Rover(link=Esp32Emulator())


# -- with the emulator on the other end -------------------------------------


def test_a_drive_goes_out_as_a_streaming_frame(rover, board, clock):
    rover.drive(100.0, -12.5)
    board.tick(clock.now_ms)

    assert board.last_seq == 1
    assert board.commanded == {"v_mm_s": 100.0, "omega_deg_s": -12.5}
    assert rover.pending == (), "a drive waits for nothing"


def test_seq_climbs_and_link_age_follows_the_echo(rover, board, clock):
    for _ in range(3):
        rover.drive(100.0, 0.0)
        board.tick(clock.now_ms)
        rover.poll(clock.now_ms)
        clock.now_ms += 100.0

    # The last echo was one round ago.
    assert rover.get_drive_state()["link_age_ms"] == 100
    assert rover.last_state["last_seq"] == 3


def test_link_age_climbs_while_the_board_says_nothing(rover, board, clock):
    rover.drive(100.0, 0.0)
    board.tick(clock.now_ms)
    rover.poll(clock.now_ms)

    clock.now_ms = 700.0

    assert rover.get_drive_state()["link_age_ms"] == 700


def test_link_age_is_zero_only_before_anything_was_sent(rover):
    assert rover.get_drive_state()["link_age_ms"] == 0


def test_a_board_echoing_a_seq_this_session_never_sent_is_rejected(clock):
    """Not smoothed into a link age of zero.  A board still running the
    previous session, or a line corrupted into a valid-looking one, is not a
    healthy link — and the next link age computed from it would be measuring a
    frame that does not exist.

    The emulator is not taught to lie for this; the line arrives from a
    transport that hands back exactly what a broken board would.
    """

    class Liar:
        """A link that echoes a seq nobody sent."""

        def write_line(self, line: str) -> None:
            pass

        def read_lines(self) -> list[str]:
            return [
                encode(
                    {
                        "type": "rover_state",
                        "commanded": {"v_mm_s": 0.0, "omega_deg_s": 0.0},
                        "last_seq": 999,
                        "estop": False,
                        "motors_enabled": True,
                        "uptime_ms": 10,
                    }
                )
            ]

    rover = Esp32Rover.from_config(load_config(), link=Liar(), clock=clock)
    rover.drive(100.0, 0.0)

    with pytest.raises(UnknownSeq):
        rover.poll(clock.now_ms)


def test_stop_is_discrete_and_waits_for_an_ack(rover, board, clock):
    rover.stop()
    assert rover.pending == (1,)

    board.tick(clock.now_ms)
    rover.poll(clock.now_ms)

    assert rover.pending == ()
    assert board.discrete_commands == [("stop", 1)]


def test_an_unanswered_stop_becomes_one_fault(rover, board, clock):
    """The board is not ticked, so nothing answers.  Rule 4, at the deadline
    config gives it."""
    rover.stop()

    assert rover.poll(clock.now_ms) == []

    (fault,) = rover.poll(200.0)
    assert fault.code == "communication_lost"
    assert fault.id == 1


def test_the_estop_flag_comes_from_the_sense_line_not_from_the_command(rover, board, clock):
    """The button is the truth.  An emergency_stop command latches the board's
    motor enable, but the sense line reads false until somebody presses it —
    and get_drive_state reports what the wire says."""
    rover.emergency_stop()
    board.tick(clock.now_ms)
    rover.poll(clock.now_ms)

    assert rover.get_drive_state()["estop"] is False
    assert rover.last_state["motors_enabled"] is False

    board.press_estop()
    board.tick(clock.now_ms + 100.0)
    rover.poll(clock.now_ms + 100.0)

    assert rover.get_drive_state()["estop"] is True


def test_an_error_from_the_board_comes_back_as_a_fault(rover, board, clock):
    rover.drive(100.0, 0.0)
    board.tick(clock.now_ms)
    rover.poll(clock.now_ms)

    board.tick(clock.now_ms + 400.0)
    (fault,) = rover.poll(clock.now_ms + 400.0)

    assert fault.code == "command_timeout"
    assert fault.id is None
