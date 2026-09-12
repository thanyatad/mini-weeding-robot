"""An ESP32 emulator that behaves like the firmware, not like a polite server.

Used by ``tests/integration/`` to close the loop

    Controller + Bridge + ESP32 emulator

with no hardware and no Isaac.  It is the transport as well as the board: lines
go in with :meth:`write_line`, come back out with :meth:`read_lines`, and
nothing happens in between until :meth:`tick` runs a round.

That buffer is the point.  §8.4 says the serial buffer accumulates and the board
reads *every* line available each round, uses the highest ``seq`` and throws the
rest away.  A board that worked through the queue in order would steer by a
command the Pi changed its mind about five rounds ago, and a Pi that merely
sends carefully does not make that go away — latest-wins is enforced here,
which is the behaviour V3 will be tested against on real hardware.

The six behaviours from bridge/README.md, and why each one would otherwise be
missed:

``command timeout``
    No ``drive`` for ``command_timeout_ms`` and the motors die, on the board's
    own account, whether or not the Pi is alive.  It reports
    ``command_timeout`` once, as a message — the controller finds out from an
    error, not from silence, and the motors are already dead when it arrives.

``latest-wins``
    The highest ``seq`` in the round is applied and the rest are discarded, and
    :attr:`drives_discarded` counts them so a test can tell "the last one won"
    from "they were all applied in order".

``a seq that goes backwards``
    ``seq <= last_seq`` is discarded, repeats included.  A stale frame must not
    refresh the command timeout, or a link that has gone quiet would look alive.

``stop / emergency_stop override every drive in the same round``
    Discrete commands are applied after the round's winning drive, whatever
    order they arrived in, so the halt wins without the board having to sort
    the buffer.

``reset refused while the E-stop sense line is active``
    Answered with ``error`` code ``emergency_stop`` rather than ``ack``.  The
    sense line is hardware; software does not get to clear a button somebody is
    still holding down.

``uptime_ms going backwards``
    :meth:`reboot` restarts the board's clock and its ``last_seq``, which is how
    the Pi is supposed to notice an MCU that reset mid-row.

Two latches, cleared by ``reset`` and by nothing else: ``emergency_stop`` and
``command_timeout``.  A board that resumed on the next ``drive`` would have the
rover moving again with nobody having decided that it should (§8.5).

This is test scaffolding, so it holds no clock: ``now_ms`` comes from the caller,
the way it does everywhere else in this repo.
"""

from __future__ import annotations

from typing import Any

from bridge.protocol import DISCRETE_TYPES, ProtocolError, decode, encode

#: Telemetry rate from §8.2 — 20 Hz, tied to no command id.
TELEMETRY_PERIOD_MS = 50.0


class Esp32Emulator:
    """The firmware's observable behaviour, in process."""

    def __init__(
        self,
        command_timeout_ms: float = 300.0,
        telemetry_period_ms: float = TELEMETRY_PERIOD_MS,
        estop_sense: bool = False,
    ) -> None:
        self.command_timeout_ms = float(command_timeout_ms)
        self.telemetry_period_ms = float(telemetry_period_ms)

        self._inbox: list[str] = []
        self._outbox: list[str] = []

        self._boot_ms = 0.0
        self._now_ms = 0.0
        self._last_telemetry_ms: float | None = None

        self._estop_sense = bool(estop_sense)
        self._estop_latched = bool(estop_sense)
        self._timed_out = False

        self._last_seq = 0
        self._last_drive_ms: float | None = None
        self._v_mm_s = 0.0
        self._omega_deg_s = 0.0

        #: Counters, so a test can tell latest-wins from "applied in order".
        self.drives_applied = 0
        self.drives_discarded = 0
        self.malformed_lines = 0
        #: ``(type, id)`` of every discrete command obeyed, in order.  Once the
        #: codec has settled the ack there is nothing left for a test to look
        #: at, and "was the brake actually commanded" is worth being able to ask.
        self.discrete_commands: list[tuple[str, int]] = []

    @classmethod
    def from_config(cls, config: dict[str, Any], **kwargs: Any) -> Esp32Emulator:
        from controller.config import get

        return cls(command_timeout_ms=get(config, "safety.command_timeout_ms"), **kwargs)

    # -- transport ---------------------------------------------------------

    def write_line(self, line: str) -> None:
        """Pi -> board.  Buffers only; the board acts on loop rounds."""
        self._inbox.append(line)

    def read_lines(self) -> list[str]:
        """Board -> Pi.  Drains everything written since the last read."""
        out, self._outbox = self._outbox, []
        return out

    # -- the bench ---------------------------------------------------------

    @property
    def motors_enabled(self) -> bool:
        return not (self._estop_latched or self._timed_out)

    @property
    def last_seq(self) -> int:
        return self._last_seq

    @property
    def commanded(self) -> dict[str, float]:
        """The standing command, as rover_state carries it.

        Telemetry is throttled to 20 Hz, so a round that lands less than
        ``telemetry_period_ms`` after the last one publishes nothing.  This is
        the same state, readable on the bench, for assertions at a finer
        boundary than the wire reports.
        """
        return {"v_mm_s": self._v_mm_s, "omega_deg_s": self._omega_deg_s}

    @property
    def uptime_ms(self) -> int:
        return int(self._now_ms - self._boot_ms)

    @property
    def estop(self) -> bool:
        return self._estop_sense

    def press_estop(self) -> None:
        """Drive the sense line active.  Hardware: nothing was sent for this."""
        self._estop_sense = True

    def release_estop(self) -> None:
        """Release the button.  The latch stays until a ``reset`` clears it."""
        self._estop_sense = False

    def reboot(self, now_ms: float) -> None:
        """Restart the MCU: the clock goes back to zero and so does last_seq."""
        self._boot_ms = float(now_ms)
        self._now_ms = float(now_ms)
        self._last_telemetry_ms = None
        self._inbox.clear()
        self._last_seq = 0
        self._last_drive_ms = None
        self._v_mm_s = 0.0
        self._omega_deg_s = 0.0
        self._timed_out = False
        self._estop_latched = self._estop_sense
        self.discrete_commands.clear()

    # -- one loop round ----------------------------------------------------

    def tick(self, now_ms: float) -> None:
        """Read the whole buffer, obey it, check the timeout, publish state."""
        self._now_ms = float(now_ms)

        drives, discretes = self._drain()
        self._apply_latest_drive(drives)
        # After the drive, whatever order they arrived in: a halt found anywhere
        # in the buffer overrides every drive in the round.
        for command in discretes:
            self._apply_discrete(command)

        self._check_estop_sense()
        self._check_command_timeout()
        self._publish_state()

    def _drain(self) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Every line waiting, split into the two classes of command."""
        drives: list[dict[str, Any]] = []
        discretes: list[dict[str, Any]] = []

        for line in self._inbox:
            try:
                message = decode(line)
            except ProtocolError:
                # Serial delivers noise on connect and half a line after a
                # reset.  A board that stopped on it would stop when somebody
                # plugged in a cable.
                self.malformed_lines += 1
                continue

            if message["type"] == "drive":
                drives.append(message)
            elif message["type"] in DISCRETE_TYPES:
                discretes.append(message)
            # Anything else is a message the board sends, not one it receives.

        self._inbox.clear()
        return drives, discretes

    def _apply_latest_drive(self, drives: list[dict[str, Any]]) -> None:
        """Highest seq wins; everything else in the round is thrown away."""
        if not drives:
            return

        winner = max(drives, key=lambda message: message["seq"])
        self.drives_discarded += len(drives) - 1

        if winner["seq"] <= self._last_seq:
            # Out of order, or the same frame twice.  Discarded without
            # refreshing the command timeout: a stale frame is not evidence
            # that the link is alive.
            self.drives_discarded += 1
            return

        self._last_seq = winner["seq"]
        self.drives_applied += 1
        self._last_drive_ms = self._now_ms

        if not self.motors_enabled:
            # Received and echoed in last_seq, but not obeyed: the latch is
            # cleared by reset and by nothing else.
            return

        self._v_mm_s = float(winner["v_mm_s"])
        self._omega_deg_s = float(winner["omega_deg_s"])

    def _apply_discrete(self, command: dict[str, Any]) -> None:
        command_type = command["type"]
        command_id = command["id"]
        self.discrete_commands.append((command_type, command_id))

        if command_type == "stop":
            # Controlled stop, brake held.  Not a latch: drive may follow.
            self._v_mm_s = 0.0
            self._omega_deg_s = 0.0
            self._send({"type": "ack", "id": command_id})
            return

        if command_type == "emergency_stop":
            self._estop_latched = True
            self._v_mm_s = 0.0
            self._omega_deg_s = 0.0
            self._send({"type": "ack", "id": command_id})
            return

        if self._estop_sense:
            self._send(
                {
                    "type": "error",
                    "code": "emergency_stop",
                    "message": "reset refused: the E-stop sense line is still active",
                    "id": command_id,
                }
            )
            return

        self._estop_latched = False
        self._timed_out = False
        # The deadline runs from the reset, not from the last drive before it:
        # otherwise a reset would be followed instantly by the timeout it just
        # cleared.
        self._last_drive_ms = self._now_ms
        self._send({"type": "ack", "id": command_id})

    def _check_estop_sense(self) -> None:
        """The button cuts the motor rail electrically; this is the firmware
        noticing, not the firmware doing it."""
        if self._estop_sense:
            self._estop_latched = True
            self._v_mm_s = 0.0
            self._omega_deg_s = 0.0

    def _check_command_timeout(self) -> None:
        """No drive within ``command_timeout_ms`` and the motors die here.

        This does not depend on the Pi being alive, which is the whole of its
        job (§5.6): it trips at 300 ms, while the Pi's link_lost is 500 ms, so
        the motors are already dead when the controller declares the fault.
        """
        if self._timed_out or self._last_drive_ms is None:
            return

        elapsed = self._now_ms - self._last_drive_ms
        if elapsed <= self.command_timeout_ms:
            return

        self._timed_out = True
        self._v_mm_s = 0.0
        self._omega_deg_s = 0.0
        self._send(
            {
                "type": "error",
                "code": "command_timeout",
                "message": f"no drive command for {elapsed:.0f} ms",
            }
        )

    def _publish_state(self) -> None:
        """Telemetry at 20 Hz, tied to no command id."""
        if (
            self._last_telemetry_ms is not None
            and self._now_ms - self._last_telemetry_ms < self.telemetry_period_ms
        ):
            return

        self._last_telemetry_ms = self._now_ms
        self._send(
            {
                "type": "rover_state",
                "commanded": {"v_mm_s": self._v_mm_s, "omega_deg_s": self._omega_deg_s},
                "last_seq": self._last_seq,
                "estop": self._estop_sense,
                "motors_enabled": self.motors_enabled,
                "uptime_ms": self.uptime_ms,
            }
        )

    def _send(self, message: dict[str, Any]) -> None:
        self._outbox.append(encode(message))
