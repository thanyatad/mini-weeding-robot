"""encode / decode, and the pending table only discrete commands enter.

Two classes of command, two sets of rules (§8.1):

===============  ===================  ==============================
                 Streaming            Discrete
===============  ===================  ==============================
message          ``drive``            ``stop`` ``emergency_stop`` ``reset``
correlation      ``seq``              ``id``
ack              none                 required
may be dropped   yes                  no
timeout -> ERROR no                   yes
semantics        latest-wins          every one must be executed
===============  ===================  ==============================

``drive`` never enters the pending table.  It is sent at 10 Hz for the length of
a row: one dropped frame would become an ERROR, and the table would grow by ten
entries a second with nobody ever reading it.  Rule 4 in
``protocol/messages.md`` is scoped to discrete commands, and §8.1 is explicit
that this is a change to the protocol rule rather than an oversight in it.

A discrete command that goes unanswered past ``safety.ack_timeout_ms`` produces
exactly one :class:`Fault`, carrying the code and the id that identify it.  The
code is ``communication_lost`` — §8.5 gives that code to the Pi for a serial
link that dropped, which is what a command going out with nothing coming back
is.  It is not ``link_lost``, which §8.5 defines as ``link_age_ms`` exceeding
its ceiling and which belongs to the link monitor; two detectors sharing one
code cannot be told apart in a log.

What settles a pending command is an answer, of either kind.  A ``reset``
refused while the E-stop sense line is still active is answered with an
``error``, not an ``ack``, and that error closes the command — leaving it
pending would produce a second fault at the deadline for a command the board
has already replied to.

This module holds no clock.  ``now_ms`` arrives from the caller, as it does in
the state machine and the row-loss watchdog, which is what lets a deadline be
tested against a synthetic timeline rather than a sleep.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

#: Streaming: correlated by ``seq``, never acked, may be dropped.
STREAMING_TYPES = frozenset({"drive"})

#: Discrete: correlated by ``id``, acked, and Rule 4 applies to these only.
DISCRETE_TYPES = frozenset({"stop", "emergency_stop", "reset"})

#: Rover -> Controller.
RESPONSE_TYPES = frozenset({"ack", "rover_state", "error"})

#: The seven codes of §8.5.  Mirrors ``controller.workflow.ErrorCode``, which is
#: the enum the controller reasons with; this is the wire's spelling of it and
#: the two are held together by a test rather than by an import, because
#: ``bridge`` must stay usable without the controller.
ERROR_CODES = frozenset(
    {
        "row_lost",
        "link_lost",
        "camera_timeout",
        "command_timeout",
        "config_invalid",
        "communication_lost",
        "emergency_stop",
    }
)

#: type -> (required fields, optional fields).  The same catalog
#: ``protocol/schema.json`` states in JSON Schema; a test holds the two to the
#: same four rejections so that neither can drift into accepting what the other
#: refuses.
_FIELDS: dict[str, tuple[frozenset[str], frozenset[str]]] = {
    "drive": (frozenset({"seq", "v_mm_s", "omega_deg_s"}), frozenset()),
    "stop": (frozenset({"id"}), frozenset()),
    "emergency_stop": (frozenset({"id"}), frozenset()),
    "reset": (frozenset({"id"}), frozenset()),
    "ack": (frozenset({"id"}), frozenset()),
    "rover_state": (
        frozenset({"commanded", "last_seq", "estop", "motors_enabled", "uptime_ms"}),
        frozenset(),
    ),
    "error": (frozenset({"code", "message"}), frozenset({"id"})),
}


class ProtocolError(Exception):
    """A line that is not a message of this catalog."""


class UnknownAck(ProtocolError):
    """An ack or an id-carrying error for a command this session never sent.

    Raised rather than ignored.  An ack nobody is waiting for means the board is
    answering a previous session, or a line was corrupted into a valid-looking
    one — and a codec that shrugs at it will shrug at the ack that was supposed
    to settle a command still counting down to an ERROR.
    """


@dataclass(frozen=True)
class Fault:
    """A fault the codec detected, in the wire's own vocabulary.

    ``code`` is a §8.5 string rather than a ``controller.workflow.ErrorCode``:
    this package carries and does not decide, and the controller maps the code
    to the event it takes.
    """

    code: str
    message: str
    id: int | None = None


def _validate(message: Any) -> dict[str, Any]:
    """Check ``message`` against the catalog.  Returns it, or raises."""
    if not isinstance(message, dict):
        raise ProtocolError(f"a message is a JSON object, got {type(message).__name__}")

    message_type = message.get("type")
    if message_type not in _FIELDS:
        raise ProtocolError(f"unknown message type: {message_type!r}")

    required, optional = _FIELDS[message_type]
    present = set(message) - {"type"}

    missing = required - present
    if missing:
        raise ProtocolError(f"{message_type} is missing {sorted(missing)}")

    extra = present - required - optional
    if extra:
        # The one that matters: a drive carrying an id, or a rover_state with
        # the command flattened out of its nesting.
        raise ProtocolError(f"{message_type} does not carry {sorted(extra)}")

    if message_type == "error" and message["code"] not in ERROR_CODES:
        raise ProtocolError(f"unknown error code: {message['code']!r}")

    if message_type == "rover_state":
        commanded = message["commanded"]
        if not isinstance(commanded, dict) or set(commanded) != {"v_mm_s", "omega_deg_s"}:
            raise ProtocolError(
                "rover_state.commanded holds exactly v_mm_s and omega_deg_s — it is an echo "
                "of the command, and flattening it would read like feedback the rover has no "
                "encoder to produce"
            )

    return message


def encode(message: dict[str, Any]) -> str:
    """One message, one line, validated before it reaches the wire."""
    return json.dumps(_validate(message), separators=(",", ":"))


def decode(line: str) -> dict[str, Any]:
    """Parse one line into a message, or raise :class:`ProtocolError`.

    Serial delivers noise on connect and half a line after a board reset, so a
    line that does not parse is expected traffic rather than an internal error.
    """
    try:
        parsed = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"not JSON: {line!r}") from exc
    return _validate(parsed)


class Codec:
    """Encoding both ways, plus the ``id`` correlation of discrete commands."""

    def __init__(self, ack_timeout_ms: float) -> None:
        if ack_timeout_ms <= 0:
            raise ValueError(f"ack_timeout_ms must be positive, got {ack_timeout_ms}")
        self.ack_timeout_ms = float(ack_timeout_ms)
        self._next_id = 1
        #: id -> (type, sent_at).  Discrete commands only, and emptied by an
        #: answer or by the deadline — never by a drive frame.
        self._pending: dict[int, tuple[str, float]] = {}

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> Codec:
        from controller.config import get

        return cls(ack_timeout_ms=get(config, "safety.ack_timeout_ms"))

    @property
    def pending(self) -> tuple[int, ...]:
        """Ids awaiting an answer, oldest first."""
        return tuple(sorted(self._pending))

    def drive(self, seq: int, v_mm_s: float, omega_deg_s: float) -> str:
        """Encode a streaming command.  Touches nothing else, deliberately."""
        return encode(
            {
                "type": "drive",
                "seq": int(seq),
                "v_mm_s": float(v_mm_s),
                "omega_deg_s": float(omega_deg_s),
            }
        )

    def discrete(self, command_type: str, now_ms: float) -> tuple[int, str]:
        """Encode a discrete command and start its ack deadline.

        Returns ``(id, line)``.  The id is allocated here because the pending
        table is what it is for.
        """
        if command_type not in DISCRETE_TYPES:
            raise ProtocolError(
                f"{command_type!r} is not a discrete command — Rule 4 applies to "
                f"{sorted(DISCRETE_TYPES)} only"
            )

        command_id = self._next_id
        self._next_id += 1
        line = encode({"type": command_type, "id": command_id})
        self._pending[command_id] = (command_type, float(now_ms))
        return command_id, line

    def receive(self, line: str, now_ms: float) -> dict[str, Any]:
        """Decode one line from the board and settle what it answers."""
        message = decode(line)

        if message["type"] == "ack":
            self._settle(message["id"], "ack")
        elif message["type"] == "error" and "id" in message:
            # An answer, just not the happy one: a refused reset comes back as
            # an error carrying the id it refuses.
            self._settle(message["id"], f"error {message['code']!r}")

        return message

    def tick(self, now_ms: float) -> list[Fault]:
        """Faults for every discrete command whose deadline has passed.

        Each one is reported once: the entry is dropped as the fault is made,
        because a fault repeated every loop reads like a stream of new ones.
        """
        expired = [
            (command_id, command_type, sent_at)
            for command_id, (command_type, sent_at) in sorted(self._pending.items())
            if now_ms - sent_at >= self.ack_timeout_ms
        ]

        faults = []
        for command_id, command_type, sent_at in expired:
            del self._pending[command_id]
            faults.append(
                Fault(
                    code="communication_lost",
                    message=(
                        f"{command_type} id={command_id} was not acked within "
                        f"{self.ack_timeout_ms:.0f} ms ({now_ms - sent_at:.0f} ms elapsed)"
                    ),
                    id=command_id,
                )
            )
        return faults

    def _settle(self, command_id: int, answer: str) -> None:
        if command_id not in self._pending:
            raise UnknownAck(
                f"{answer} for id={command_id}, which is not pending "
                f"(pending: {list(self.pending)})"
            )
        del self._pending[command_id]
