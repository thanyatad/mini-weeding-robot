"""Rover backend for the ESP32, speaking through ``bridge/``.

Construction stays side-effect free: it opens nothing, reads no config file and
needs no board.  ``Esp32Rover()`` with no arguments is a backend with no link,
which is what ``tests/unit/test_rover_backends.py`` builds to check that the
drive-state shape matches the other two.  Anything that needs a board raises
rather than pretending.

With a link attached it is the controller's end of the protocol:

    drive()             seq allocated, encoded, written, stamped in the tracker
    stop()              a discrete command, pending until the board acks it
    emergency_stop()    the same, at the highest priority the wire has
    poll()              telemetry in, acks settled, faults out
    link_age_ms         now - sent_at[last_seq], the Pi's clock at both ends

Its `link` is anything with ``write_line`` and ``read_lines``: the emulator in
``bridge/simulator.py`` today, a pyserial port at V3.  Neither this class nor
anything above it changes when that swap happens, which is the reason the
transport was kept behind two methods.

What it does not do
-------------------
It does not decide what a fault means.  ``poll()`` returns what the board said
and what the codec noticed; ``RowRun.report_fault()`` owns the transition and
the braking, and a backend that also stopped the rover would be the second
place that rule lives.

It has no measured velocity to offer and never will: the ESP32 mixes and drives
open-loop, so ``get_drive_state()`` here carries exactly what it carries
everywhere else.  ``link_age_ms`` is the one value in it that is measured
rather than echoed.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any, Protocol

from bridge.protocol import Codec, Fault
from bridge.seq_tracker import SeqTracker, UnknownSeq
from controller.rover.base import DriveStateOwner

_NO_LINK = "Esp32Rover has no link attached — build it with Esp32Rover.from_config(config, link)"


class LineTransport(Protocol):
    """One JSON message per line, both directions.

    The whole of what this backend needs from a transport.  ``bridge.simulator``
    satisfies it in process; ``serial_link.py`` will satisfy it over a port.
    """

    def write_line(self, line: str) -> None: ...

    def read_lines(self) -> list[str]: ...


def _monotonic_ms() -> float:
    return time.monotonic() * 1000.0


class Esp32Rover(DriveStateOwner):
    """Velocity commands out, telemetry and faults back."""

    def __init__(
        self,
        link: LineTransport | None = None,
        *,
        codec: Codec | None = None,
        tracker: SeqTracker | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        super().__init__()
        if link is not None and (codec is None or tracker is None):
            raise ValueError(
                "a link needs a codec and a seq tracker — use Esp32Rover.from_config(config, link)"
            )
        self._link = link
        self._codec = codec
        self._tracker = tracker
        #: The one place a real clock enters this stack.  Everything below —
        #: the codec, the tracker, the state machine — is handed ``now_ms``.
        self._clock = clock if clock is not None else _monotonic_ms

        self._last_seq = 0
        self._motors_enabled = False
        self._uptime_ms = 0
        self._published_command = {"v_mm_s": 0.0, "omega_deg_s": 0.0}

    @classmethod
    def from_config(
        cls,
        config: dict[str, Any],
        link: LineTransport,
        clock: Callable[[], float] | None = None,
    ) -> Esp32Rover:
        return cls(
            link,
            codec=Codec.from_config(config),
            tracker=SeqTracker.from_config(config),
            clock=clock,
        )

    # -- Rover interface ---------------------------------------------------

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Send one streaming frame.  Nothing waits on it (§8.1)."""
        codec, tracker = self._require_link()

        seq = tracker.next_seq()
        self._link.write_line(codec.drive(seq, v_mm_s, omega_deg_s))
        tracker.sent(seq, self._clock())
        self._record_command(v_mm_s, omega_deg_s)

    def stop(self) -> None:
        """Controlled stop, brake held.  Discrete: it has to be acked."""
        self._send_discrete("stop")
        self._record_command(0.0, 0.0)

    def emergency_stop(self) -> None:
        """Cut motor enable and latch.  Cleared by a person, never by software."""
        self._send_discrete("emergency_stop")
        self._record_command(0.0, 0.0)

    # -- the link ----------------------------------------------------------

    @property
    def pending(self) -> tuple[int, ...]:
        """Discrete commands still waiting for an answer."""
        return () if self._codec is None else self._codec.pending

    @property
    def last_state(self) -> dict[str, Any]:
        """The last telemetry seen, in the fields the board publishes.

        ``commanded`` here is the board's account of what it is doing, which is
        not the same thing as ``get_drive_state()["commanded"]`` — that is this
        process's echo of what it asked for.  They part company exactly where
        it matters: a latched board reports zero while the controller's echo
        still holds the last drive it sent into a link that is no longer being
        obeyed.
        """
        return {
            "commanded": dict(self._published_command),
            "last_seq": self._last_seq,
            "estop": self._estop,
            "motors_enabled": self._motors_enabled,
            "uptime_ms": self._uptime_ms,
        }

    def poll(self, now_ms: float | None = None) -> list[Fault]:
        """Read everything the board has said, and report what went wrong.

        Faults come from two places and both are the caller's to route: an
        ``error`` message the board sent, and a discrete command the codec
        watched miss its deadline.
        """
        codec, _ = self._require_link()
        now_ms = self._clock() if now_ms is None else now_ms

        faults: list[Fault] = []
        for line in self._link.read_lines():
            message = codec.receive(line, now_ms)
            if message["type"] == "rover_state":
                self._absorb_state(message)
            elif message["type"] == "error":
                faults.append(
                    Fault(code=message["code"], message=message["message"], id=message.get("id"))
                )

        faults.extend(codec.tick(now_ms))
        return faults

    def _absorb_state(self, state: dict[str, Any]) -> None:
        """Take the telemetry at face value, except for what it cannot know.

        ``estop`` is the sense line as the board reads it from the wire, which
        is why it is taken from here rather than latched locally when an
        ``emergency_stop`` is sent: the button, not the command, is the truth.

        ``last_seq`` is the exception: a board echoing a seq this session never
        sent is a board still running the previous one, or a line corrupted into
        a valid-looking message.  Taking it would make the next link age a
        measurement of a frame that does not exist.
        """
        last_seq = int(state["last_seq"])
        highest_sent = self._tracker.highest_sent if self._tracker is not None else None
        if last_seq > (highest_sent if highest_sent is not None else 0):
            raise UnknownSeq(
                f"the board echoed last_seq={last_seq}, which this session never sent "
                f"(highest sent: {highest_sent})"
            )

        self._last_seq = last_seq
        self._published_command = {
            "v_mm_s": float(state["commanded"]["v_mm_s"]),
            "omega_deg_s": float(state["commanded"]["omega_deg_s"]),
        }
        self._estop = bool(state["estop"])
        self._motors_enabled = bool(state["motors_enabled"])
        self._uptime_ms = int(state["uptime_ms"])

    def _link_age_ms(self) -> int:
        """Milliseconds since the frame the board last echoed was sent (§8.3).

        Zero while nothing has been sent: there is no link age before there is
        a link.  It cannot raise: :meth:`_absorb_state` refuses a ``last_seq``
        this session never sent, so what is stored here is always a frame the
        tracker can answer for.
        """
        if self._tracker is None or self._tracker.highest_sent is None:
            return 0
        return int(self._tracker.age(self._last_seq, self._clock()))

    def _send_discrete(self, command_type: str) -> int:
        codec, _ = self._require_link()
        command_id, line = codec.discrete(command_type, self._clock())
        self._link.write_line(line)
        return command_id

    def _require_link(self) -> tuple[Codec, SeqTracker]:
        if self._link is None or self._codec is None or self._tracker is None:
            raise NotImplementedError(_NO_LINK)
        return self._codec, self._tracker
