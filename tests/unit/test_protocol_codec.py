"""encode / decode, and the pending table that only discrete commands enter.

The split under test is the whole point of §8.1.  ``drive`` is streaming: 10 Hz
forever, no ack, latest-wins, and a frame may be lost.  ``stop``,
``emergency_stop`` and ``reset`` are discrete: every one must be executed, every
one is acked, and one that is not acked in time is Rule 4's ERROR.

If ``drive`` were correlated the same way, one dropped frame in a hundred would
become an ERROR and the pending table would grow by ten entries a second for the
length of the run.  That is not a detail of the implementation; it is the rule
change spec §8.1 makes, and these tests are where it is held.
"""

import json

import pytest

from bridge.protocol import (
    DISCRETE_TYPES,
    Codec,
    ProtocolError,
    UnknownAck,
    decode,
    encode,
)
from controller.config import load_config, with_overrides
from controller.workflow import ErrorCode
from tests.harness.json_schema import is_valid, load_schema

ACK_TIMEOUT_MS = 200.0

CONTRACT_MESSAGES = (
    {"type": "drive", "seq": 1234, "v_mm_s": 100.0, "omega_deg_s": -12.5},
    {"type": "stop", "id": 77},
    {"type": "emergency_stop", "id": 78},
    {"type": "reset", "id": 79},
    {"type": "ack", "id": 77},
    {"type": "error", "code": "command_timeout", "message": "no drive command for 312 ms"},
    {
        "type": "rover_state",
        "commanded": {"v_mm_s": 100.0, "omega_deg_s": -12.5},
        "last_seq": 1234,
        "estop": False,
        "motors_enabled": True,
        "uptime_ms": 48210,
    },
)


@pytest.fixture
def codec():
    return Codec(ack_timeout_ms=ACK_TIMEOUT_MS)


@pytest.fixture(scope="module")
def schema():
    return load_schema()


# -- encoding ---------------------------------------------------------------


def test_a_drive_encodes_to_the_contract_line(codec):
    line = codec.drive(seq=1234, v_mm_s=100.0, omega_deg_s=-12.5)

    assert json.loads(line) == {
        "type": "drive",
        "seq": 1234,
        "v_mm_s": 100.0,
        "omega_deg_s": -12.5,
    }
    assert "\n" not in line, "one message per line — the framing is the newline"


@pytest.mark.parametrize("message", CONTRACT_MESSAGES, ids=lambda m: m["type"])
def test_every_contract_payload_survives_a_round_trip(message, schema):
    assert decode(encode(message)) == message
    assert is_valid(message, schema)


@pytest.mark.parametrize(
    "bad",
    [
        pytest.param(
            {"type": "drive", "seq": 1, "id": 7, "v_mm_s": 0.0, "omega_deg_s": 0.0},
            id="drive-with-an-id",
        ),
        pytest.param({"type": "stop"}, id="discrete-with-no-id"),
        pytest.param({"type": "home", "id": 80}, id="unknown-type"),
        pytest.param(
            {
                "type": "rover_state",
                "v_mm_s": 100.0,
                "omega_deg_s": -12.5,
                "last_seq": 1,
                "estop": False,
                "motors_enabled": True,
                "uptime_ms": 1,
            },
            id="flattened-rover_state",
        ),
    ],
)
def test_the_codec_rejects_exactly_what_the_schema_rejects(bad, schema):
    """One catalog, two enforcers.  A codec that accepts what schema.json
    refuses means the schema has stopped describing the wire."""
    assert not is_valid(bad, schema)
    with pytest.raises(ProtocolError):
        encode(bad)
    with pytest.raises(ProtocolError):
        decode(json.dumps(bad))


@pytest.mark.parametrize("line", ["", "   ", "not json", "[]", '"drive"', "{}"])
def test_a_line_that_is_not_a_message_is_rejected(line):
    """Serial delivers noise on connect, and half a line after a reset."""
    with pytest.raises(ProtocolError):
        decode(line)


# -- the pending table ------------------------------------------------------


def test_a_thousand_drive_frames_leave_the_pending_table_empty(codec):
    """The trap this module exists to avoid: at 10 Hz this is 100 seconds of
    driving, and nothing about it is pending on anything."""
    for seq in range(1, 1001):
        line = codec.drive(seq=seq, v_mm_s=100.0, omega_deg_s=0.0)
        assert decode(line)["seq"] == seq

    assert codec.pending == ()
    assert codec.tick(now_ms=10_000_000.0) == []


def test_a_dropped_drive_frame_produces_no_error(codec):
    """seq 103 in the correlation example never arrives.  Nobody has to do
    anything about it, at any time afterwards."""
    for seq, at in ((101, 0.0), (102, 100.0), (103, 200.0), (104, 300.0)):
        codec.drive(seq=seq, v_mm_s=100.0, omega_deg_s=0.0)
        assert codec.tick(now_ms=at) == []

    # The board only ever echoes 104; there is no ack for a drive to be missing.
    assert codec.tick(now_ms=100_000.0) == []


@pytest.mark.parametrize("command_type", sorted(DISCRETE_TYPES))
def test_a_discrete_command_is_pending_until_it_is_acked(codec, command_type):
    command_id, line = codec.discrete(command_type, now_ms=0.0)

    assert json.loads(line) == {"type": command_type, "id": command_id}
    assert codec.pending == (command_id,)

    codec.receive(encode({"type": "ack", "id": command_id}), now_ms=50.0)
    assert codec.pending == ()
    assert codec.tick(now_ms=10_000.0) == []


def test_ids_are_monotonic_across_command_types(codec):
    first, _ = codec.discrete("stop", now_ms=0.0)
    second, _ = codec.discrete("emergency_stop", now_ms=1.0)
    third, _ = codec.discrete("reset", now_ms=2.0)

    assert first < second < third


def test_an_unacked_discrete_command_faults_exactly_once(codec):
    command_id, _ = codec.discrete("stop", now_ms=0.0)

    assert codec.tick(now_ms=ACK_TIMEOUT_MS - 1) == []

    faults = codec.tick(now_ms=ACK_TIMEOUT_MS)
    assert len(faults) == 1
    assert faults[0].code == "communication_lost"
    assert faults[0].id == command_id
    assert "stop" in faults[0].message
    assert str(command_id) in faults[0].message

    assert codec.tick(now_ms=10_000.0) == [], "a fault reported twice is two faults"
    assert codec.pending == ()


def test_the_deadline_comes_from_config():
    config = load_config()
    assert Codec.from_config(config).ack_timeout_ms == 200

    stretched = Codec.from_config(with_overrides(config, {"safety.ack_timeout_ms": 900}))
    stretched.discrete("stop", now_ms=0.0)
    assert stretched.tick(now_ms=500.0) == []
    assert len(stretched.tick(now_ms=900.0)) == 1


def test_every_code_the_codec_emits_is_a_known_error_code(codec):
    """§8.5 fixes the seven.  A code outside them reaches the controller as a
    string nothing handles."""
    codec.discrete("stop", now_ms=0.0)
    (fault,) = codec.tick(now_ms=ACK_TIMEOUT_MS)

    assert ErrorCode(fault.code) is ErrorCode.COMMUNICATION_LOST


def test_an_ack_for_an_id_that_was_never_sent_is_rejected(codec):
    """Rejected rather than ignored: an ack nobody asked for means the board is
    answering a different session, or a line was corrupted into a valid one."""
    codec.discrete("stop", now_ms=0.0)

    with pytest.raises(UnknownAck):
        codec.receive(encode({"type": "ack", "id": 999}), now_ms=10.0)


def test_an_ack_arriving_twice_is_rejected(codec):
    command_id, _ = codec.discrete("stop", now_ms=0.0)
    codec.receive(encode({"type": "ack", "id": command_id}), now_ms=10.0)

    with pytest.raises(UnknownAck):
        codec.receive(encode({"type": "ack", "id": command_id}), now_ms=20.0)


def test_an_error_carrying_an_id_settles_the_command_it_answers(codec):
    """A reset refused while the E-stop sense line is still active is answered
    with an error, not an ack (protocol/messages.md).  The command has been
    answered; leaving it pending would raise a second, invented fault at the
    deadline."""
    command_id, _ = codec.discrete("reset", now_ms=0.0)

    refusal = encode(
        {
            "type": "error",
            "code": "emergency_stop",
            "message": "estop sense still active",
            "id": command_id,
        }
    )
    message = codec.receive(refusal, now_ms=10.0)

    assert message["code"] == "emergency_stop"
    assert codec.pending == ()
    assert codec.tick(now_ms=10_000.0) == []


def test_an_error_without_an_id_settles_nothing(codec):
    """command_timeout arrives on its own account, not as an answer to
    anything: the board reports it when the link comes back."""
    command_id, _ = codec.discrete("stop", now_ms=0.0)

    codec.receive(
        encode({"type": "error", "code": "command_timeout", "message": "no drive for 312 ms"}),
        now_ms=10.0,
    )

    assert codec.pending == (command_id,)


def test_receive_returns_the_decoded_message(codec):
    state = {
        "type": "rover_state",
        "commanded": {"v_mm_s": 100.0, "omega_deg_s": -12.5},
        "last_seq": 1234,
        "estop": False,
        "motors_enabled": True,
        "uptime_ms": 48210,
    }

    assert codec.receive(encode(state), now_ms=0.0) == state


def test_drive_is_not_a_discrete_command(codec):
    """The scoping fix of Rule 4, asserted where somebody would otherwise
    'fix' it by accepting a type the pending table can hold."""
    assert "drive" not in DISCRETE_TYPES

    with pytest.raises(ProtocolError):
        codec.discrete("drive", now_ms=0.0)
