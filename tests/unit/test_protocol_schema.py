"""protocol/schema.json against protocol/examples/, and against four wrong messages.

``protocol/messages.md`` has cited both files since it was written.  The catalog
is prose; the schema is the form of it another language's client can be held to,
and the examples are the payloads a test fixture can start from.

The four rejections below are the ones that matter, because each is a mistake
somebody would otherwise make and the wire would accept:

    a drive carrying an id          -- streaming treated as discrete (Rule 4)
    a discrete command with no id   -- discrete treated as streaming (Rule 2)
    an unknown type                 -- a message the catalog never defined
    a flattened rover_state         -- an echo read as feedback (there is no encoder)
"""

import json
from pathlib import Path

import pytest

from tests.harness.json_schema import (
    EXAMPLES_DIR,
    SCHEMA_PATH,
    assert_supported_keywords,
    is_valid,
    load_schema,
)

#: Every message type in the catalog.  One example file each, named for the type.
MESSAGE_TYPES = (
    "drive",
    "stop",
    "emergency_stop",
    "reset",
    "ack",
    "rover_state",
    "error",
)

#: The payloads exactly as spec §8.2 and protocol/messages.md write them.
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


@pytest.fixture(scope="module")
def schema():
    return load_schema()


def test_the_schema_only_uses_keywords_the_validator_checks(schema):
    assert_supported_keywords(schema)


def test_schema_exists_where_messages_md_says_it_does():
    assert SCHEMA_PATH.is_file(), f"{SCHEMA_PATH} is cited by protocol/messages.md"


@pytest.mark.parametrize("message_type", MESSAGE_TYPES)
def test_one_example_per_message_type(message_type):
    path = EXAMPLES_DIR / f"{message_type}.json"
    assert path.is_file(), f"protocol/examples/ has no example for {message_type!r}"
    assert json.loads(path.read_text(encoding="utf-8"))["type"] == message_type


@pytest.mark.parametrize("path", sorted(Path(EXAMPLES_DIR).glob("*.json")), ids=lambda p: p.name)
def test_every_example_validates(path, schema):
    assert is_valid(json.loads(path.read_text(encoding="utf-8")), schema)


@pytest.mark.parametrize("message", CONTRACT_MESSAGES, ids=lambda m: m["type"])
def test_the_contract_payloads_validate(message, schema):
    assert is_valid(message, schema)


def test_rejects_a_drive_carrying_an_id(schema):
    """Rule 4 is scoped to discrete commands.  A drive with an id invites the
    pending table to hold ten entries a second and turn a dropped frame into an
    ERROR."""
    assert not is_valid(
        {"type": "drive", "seq": 1234, "id": 77, "v_mm_s": 100.0, "omega_deg_s": -12.5}, schema
    )


@pytest.mark.parametrize("message_type", ["stop", "emergency_stop", "reset"])
def test_rejects_a_discrete_command_with_no_id(message_type, schema):
    """Without an id there is nothing for the ack to correlate to (Rule 2)."""
    assert not is_valid({"type": message_type}, schema)


def test_rejects_an_unknown_type(schema):
    assert not is_valid({"type": "home", "id": 80}, schema)


def test_rejects_a_rover_state_with_the_command_flattened(schema):
    """commanded is nested on purpose: there is no encoder, so a top-level
    v_mm_s would read like feedback and code would come to depend on it."""
    assert not is_valid(
        {
            "type": "rover_state",
            "v_mm_s": 100.0,
            "omega_deg_s": -12.5,
            "last_seq": 1234,
            "estop": False,
            "motors_enabled": True,
            "uptime_ms": 48210,
        },
        schema,
    )


def test_rejects_an_error_code_outside_the_table(schema):
    """§8.5 fixes the seven codes.  A code nobody handles is a fault that
    reaches the controller as an unknown string."""
    assert not is_valid({"type": "error", "code": "motor_timeout", "message": "..."}, schema)


def test_rejects_a_rover_state_missing_a_field(schema):
    assert not is_valid(
        {
            "type": "rover_state",
            "commanded": {"v_mm_s": 0.0, "omega_deg_s": 0.0},
            "last_seq": 1,
            "estop": False,
            "motors_enabled": True,
        },
        schema,
    )
