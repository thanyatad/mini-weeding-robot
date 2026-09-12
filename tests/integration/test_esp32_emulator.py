"""The six behaviours bridge/README.md requires of the ESP32 emulator.

An emulator that politely acks everything makes every integration test pass and
then fails at V3, where an afternoon of debugging costs more than all of this.
So each item on that checklist gets a test that fails if the emulator stops
doing it:

    [ ] command timeout — no drive for command_timeout_ms -> motors_enabled: false
    [ ] latest-wins — several drives in the buffer, the highest seq is used
    [ ] a seq that goes backwards is discarded
    [ ] stop / emergency_stop override every drive in the same round
    [ ] reset is refused while the estop sense line is still active
    [ ] uptime_ms going backwards, to simulate a reboot

The buffer is the thing to keep in view.  ``write_line`` only fills it; nothing
happens until ``tick`` runs a round, which is how a real board behaves between
loop iterations and is what makes latest-wins a property that can be tested at
all.
"""

import json

import pytest

from bridge.protocol import decode, encode
from bridge.simulator import Esp32Emulator
from controller.config import load_config, with_overrides
from tests.harness.json_schema import is_valid, load_schema

COMMAND_TIMEOUT_MS = 300.0


@pytest.fixture
def board():
    return Esp32Emulator(command_timeout_ms=COMMAND_TIMEOUT_MS)


@pytest.fixture(scope="module")
def schema():
    return load_schema()


def drive(seq, v_mm_s=100.0, omega_deg_s=0.0):
    return encode({"type": "drive", "seq": seq, "v_mm_s": v_mm_s, "omega_deg_s": omega_deg_s})


def messages(board, message_type=None):
    out = [decode(line) for line in board.read_lines()]
    return [m for m in out if message_type is None or m["type"] == message_type]


def latest_state(board):
    states = messages(board, "rover_state")
    assert states, "the board publishes telemetry every round"
    return states[-1]


# -- [1] command timeout ----------------------------------------------------


def test_no_drive_for_command_timeout_ms_kills_the_motors(board):
    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    assert latest_state(board)["motors_enabled"] is True

    board.tick(now_ms=COMMAND_TIMEOUT_MS)
    assert board.motors_enabled is True, "within the timeout is not past it"

    board.tick(now_ms=COMMAND_TIMEOUT_MS + 50.0)
    state = latest_state(board)
    assert state["motors_enabled"] is False
    assert state["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}


def test_the_timeout_is_reported_as_an_error_not_as_silence(board):
    """The controller learns about it from a message; the motors are already
    dead when it arrives (§8.5)."""
    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    board.read_lines()

    board.tick(now_ms=312.0)

    (error,) = messages(board, "error")
    assert error["code"] == "command_timeout"
    assert "312" in error["message"]
    assert "id" not in error, "nothing asked for this — it is not an answer to a command"


def test_the_timeout_is_reported_once_not_every_round(board):
    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    board.tick(now_ms=400.0)
    board.read_lines()

    for now_ms in (500.0, 600.0, 700.0):
        board.tick(now_ms=now_ms)

    assert messages(board, "error") == []


def test_a_drive_alone_does_not_resume_after_a_timeout(board):
    """The latch is cleared by reset, and by nothing else.  A board that woke up
    on the next drive would have the rover moving again with nobody having
    decided it should."""
    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    board.tick(now_ms=400.0)

    board.write_line(drive(2))
    board.tick(now_ms=500.0)

    state = latest_state(board)
    assert state["motors_enabled"] is False
    assert state["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}
    assert state["last_seq"] == 2, "it was received — it just was not obeyed"


def test_the_timeout_comes_from_config():
    board = Esp32Emulator.from_config(
        with_overrides(load_config(), {"safety.command_timeout_ms": 80})
    )
    assert board.command_timeout_ms == 80

    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    board.tick(now_ms=100.0)

    assert latest_state(board)["motors_enabled"] is False


# -- [2] latest-wins --------------------------------------------------------


def test_fifty_drives_in_one_round_leave_only_the_highest_seq(board):
    """§8.4: the serial buffer accumulates, so the board reads every line
    available each round and uses the highest seq.  A board that worked through
    the queue in order would steer by a command the Pi changed its mind about
    five rounds ago."""
    for seq in range(1, 51):
        board.write_line(drive(seq, v_mm_s=float(seq), omega_deg_s=-float(seq)))

    board.tick(now_ms=0.0)

    state = latest_state(board)
    assert state["commanded"] == {"v_mm_s": 50.0, "omega_deg_s": -50.0}
    assert state["last_seq"] == 50
    assert board.drives_applied == 1
    assert board.drives_discarded == 49


def test_an_out_of_order_burst_still_uses_the_highest_seq(board):
    """Arrival order is not seq order when a buffer has been sitting."""
    for seq in (7, 3, 9, 1, 8):
        board.write_line(drive(seq, v_mm_s=float(seq)))

    board.tick(now_ms=0.0)

    assert latest_state(board)["commanded"]["v_mm_s"] == 9.0
    assert board.drives_applied == 1
    assert board.drives_discarded == 4


# -- [3] a seq that goes backwards ------------------------------------------


def test_a_seq_that_goes_backwards_is_discarded(board):
    board.write_line(drive(10, v_mm_s=100.0))
    board.tick(now_ms=0.0)

    board.write_line(drive(9, v_mm_s=20.0))
    board.tick(now_ms=100.0)

    state = latest_state(board)
    assert state["commanded"]["v_mm_s"] == 100.0
    assert state["last_seq"] == 10
    assert board.drives_discarded == 1


def test_a_repeated_seq_is_discarded(board):
    """seq <= last_seq, not seq < last_seq: a line delivered twice is the same
    frame, not a fresh one, and re-applying it would refresh the command
    timeout on a link that has gone quiet."""
    board.write_line(drive(10))
    board.tick(now_ms=0.0)

    board.write_line(drive(10))
    board.tick(now_ms=100.0)

    assert board.drives_applied == 1
    assert board.drives_discarded == 1


def test_a_stale_frame_does_not_refresh_the_command_timeout(board):
    board.write_line(drive(10))
    board.tick(now_ms=0.0)

    for now_ms in (100.0, 200.0, 300.0):
        board.write_line(drive(5))
        board.tick(now_ms=now_ms)

    board.tick(now_ms=400.0)

    assert latest_state(board)["motors_enabled"] is False


# -- [4] stop / emergency_stop override every drive in the round ------------


@pytest.mark.parametrize("halt, motors_after", [("stop", True), ("emergency_stop", False)])
@pytest.mark.parametrize("arrives", ["before", "after"])
def test_a_halt_overrides_every_drive_in_the_same_round(board, halt, motors_after, arrives):
    """Whatever order they came off the wire in.  A board that worked through
    the buffer in sequence would apply the stop and then drive away again."""
    halt_line = encode({"type": halt, "id": 77})
    lines = [halt_line, drive(1)] if arrives == "before" else [drive(1), halt_line]
    for line in lines:
        board.write_line(line)

    board.tick(now_ms=0.0)

    sent = messages(board)
    assert {"type": "ack", "id": 77} in sent

    state = [m for m in sent if m["type"] == "rover_state"][-1]
    assert state["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}
    assert state["motors_enabled"] is motors_after
    assert state["last_seq"] == 1, "the drive was received; it was overridden, not dropped"


@pytest.mark.parametrize("command_type", ["stop", "emergency_stop", "reset"])
def test_every_discrete_command_is_acked(board, command_type):
    board.write_line(encode({"type": command_type, "id": 77}))

    board.tick(now_ms=0.0)

    assert [m for m in messages(board, "ack")] == [{"type": "ack", "id": 77}]


def test_a_drive_is_never_acked(board):
    board.write_line(drive(1))

    board.tick(now_ms=0.0)

    assert messages(board, "ack") == []


def test_stop_holds_the_brake_and_drive_may_follow(board):
    """A controlled stop is not a latch: §Discrete Commands says drive follows
    it without a reset."""
    board.write_line(encode({"type": "stop", "id": 1}))
    board.tick(now_ms=0.0)
    assert latest_state(board)["motors_enabled"] is True

    board.write_line(drive(5, v_mm_s=80.0))
    board.tick(now_ms=100.0)

    assert latest_state(board)["commanded"]["v_mm_s"] == 80.0


def test_emergency_stop_latches_and_drive_does_not_follow(board):
    board.write_line(encode({"type": "emergency_stop", "id": 1}))
    board.tick(now_ms=0.0)

    board.write_line(drive(5, v_mm_s=80.0))
    board.tick(now_ms=100.0)

    state = latest_state(board)
    assert state["motors_enabled"] is False
    assert state["commanded"]["v_mm_s"] == 0.0


# -- [5] reset while the estop sense line is active -------------------------


def test_reset_is_refused_while_the_estop_sense_line_is_active(board):
    """Answered with an error code emergency_stop, not an ack
    (protocol/messages.md).  The sense line is read from the wire; software
    cannot clear a button somebody is still holding down."""
    board.press_estop()
    board.write_line(encode({"type": "reset", "id": 79}))

    board.tick(now_ms=0.0)

    sent = messages(board)
    assert [m for m in sent if m["type"] == "ack"] == []
    (error,) = [m for m in sent if m["type"] == "error"]
    assert error["code"] == "emergency_stop"
    assert error["id"] == 79
    assert [m for m in sent if m["type"] == "rover_state"][-1]["estop"] is True


def test_reset_is_accepted_once_the_button_is_released(board):
    board.press_estop()
    board.tick(now_ms=0.0)
    assert latest_state(board)["motors_enabled"] is False

    board.release_estop()
    board.write_line(encode({"type": "reset", "id": 80}))
    board.tick(now_ms=100.0)

    assert messages(board, "ack") == [{"type": "ack", "id": 80}]

    board.write_line(drive(1, v_mm_s=60.0))
    board.tick(now_ms=200.0)

    state = latest_state(board)
    assert state["estop"] is False
    assert state["motors_enabled"] is True
    assert state["commanded"]["v_mm_s"] == 60.0


def test_reset_clears_a_command_timeout_latch(board):
    board.write_line(drive(1))
    board.tick(now_ms=0.0)
    board.tick(now_ms=400.0)

    board.write_line(encode({"type": "reset", "id": 2}))
    board.tick(now_ms=500.0)
    board.write_line(drive(2, v_mm_s=70.0))
    board.tick(now_ms=600.0)

    state = latest_state(board)
    assert state["motors_enabled"] is True
    assert state["commanded"]["v_mm_s"] == 70.0


def test_pressing_the_button_cuts_the_motors_without_a_command(board):
    """The sense line is hardware.  Nothing had to be sent for this to happen."""
    board.write_line(drive(1))
    board.tick(now_ms=0.0)

    board.press_estop()
    board.tick(now_ms=50.0)

    state = latest_state(board)
    assert state["estop"] is True
    assert state["motors_enabled"] is False
    assert state["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}


# -- [6] uptime_ms going backwards ------------------------------------------


def test_uptime_goes_backwards_across_a_reboot(board):
    board.write_line(drive(9))
    board.tick(now_ms=48_210.0)
    before = latest_state(board)
    assert before["uptime_ms"] == 48_210
    assert before["last_seq"] == 9

    board.reboot(now_ms=48_260.0)
    board.tick(now_ms=48_300.0)

    after = latest_state(board)
    assert after["uptime_ms"] < before["uptime_ms"]
    assert after["last_seq"] == 0, "the board that came back has never seen a seq"
    assert after["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}, (
        "a board that came back still holding the last command would drive away on its own"
    )


def test_a_seq_from_before_the_reboot_is_not_stale_to_the_new_board(board):
    """last_seq restarts at 0, so the Pi's seq counter is ahead — which is
    exactly what a Pi that kept counting through the reboot will send."""
    board.write_line(drive(500))
    board.tick(now_ms=0.0)
    board.reboot(now_ms=100.0)

    board.write_line(drive(501, v_mm_s=40.0))
    board.tick(now_ms=200.0)

    state = latest_state(board)
    assert state["last_seq"] == 501
    assert state["commanded"]["v_mm_s"] == 40.0


# -- the wire itself --------------------------------------------------------


def test_everything_the_board_says_is_a_valid_message(board, schema):
    board.press_estop()
    board.write_line(drive(1))
    board.write_line(encode({"type": "reset", "id": 1}))
    board.tick(now_ms=0.0)
    board.release_estop()
    board.write_line(encode({"type": "stop", "id": 2}))
    board.tick(now_ms=100.0)
    board.tick(now_ms=600.0)

    lines = board.read_lines()
    assert lines, "the board said nothing at all"
    for line in lines:
        assert is_valid(json.loads(line), schema), line


def test_a_line_that_is_not_a_message_is_ignored(board):
    """Serial delivers noise on connect and half a line after a reset.  A board
    that stopped on it would be a board that stops when a cable is plugged in."""
    board.write_line("not json")
    board.write_line(drive(1, v_mm_s=55.0))

    board.tick(now_ms=0.0)

    assert board.malformed_lines == 1
    assert latest_state(board)["commanded"]["v_mm_s"] == 55.0


def test_nothing_is_processed_until_a_round_runs(board):
    """The buffer accumulates; the board acts on loop iterations.  This is what
    makes latest-wins meaningful rather than an accident of timing."""
    board.write_line(drive(1))

    assert board.read_lines() == []
    assert board.drives_applied == 0
