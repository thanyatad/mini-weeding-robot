"""The rover's states and the transitions between them (spec §5.3).

This module owns transitions and nothing else.  It does not drive, does not
read a camera, does not sleep and holds no clock — every transition is a call
from outside.  That is what lets the whole table be tested as arithmetic,
without a rover and without an image.

Two distinctions are load-bearing:

``STOPPED`` is not a fault.  It is the normal end of a run waiting for a human
to acknowledge it, and it carries a ``stop_reason`` with no error code.  The
scenario contract is ``final_state: STOPPED``, ``stop_reason:
row_end_suspected``, ``error_code: null`` — modelling the end of a row as an
error would make ``row_end`` and ``crop_gap_midrow`` indistinguishable at the
one point where they have to differ.

``ESTOP`` is reachable from every state, ``BOOT`` and ``ERROR`` included: the
button cuts the motor rail electrically, wherever the software happens to be.
Leaving it takes two separate things — the button released *and* an ack — so
that software cannot ack its way past a button somebody is still holding down.

Where the events come from
--------------------------
``ROW_END`` and ``ROW_LOST`` are raised by ``controller.safety.runaway``, the
controller's own safety layer.  ``LINK_LOST``, ``CAMERA_TIMEOUT``,
``COMMAND_TIMEOUT`` and ``COMMUNICATION_LOST`` are drawn by §5.3 and so the
edges exist here, but **nothing in V0 raises them**: their detectors are the
link monitor and the protocol codec, which are V3 work.  The edges are the
contract those detectors will be written against; a half-built detector left
in this module would read like a working one.
"""

from __future__ import annotations

from enum import Enum

#: The reason recorded on a normal stop.  Named as a suspicion on purpose:
#: green_fraction is a heuristic, not a measurement, and nothing before V1 has
#: rendered images to check it against (§5.4).
ROW_END_SUSPECTED = "row_end_suspected"


class RoverState(Enum):
    BOOT = 1
    READY = 2
    DRIVING_ROW = 3
    STOPPED = 4  # normal end of a run, waiting for a human ack
    ERROR = 5
    ESTOP = 6


class ErrorCode(Enum):
    """The codes of spec §8.5 that this machine can land in ERROR or ESTOP with."""

    ROW_LOST = "row_lost"
    LINK_LOST = "link_lost"
    CAMERA_TIMEOUT = "camera_timeout"
    COMMAND_TIMEOUT = "command_timeout"
    CONFIG_INVALID = "config_invalid"
    COMMUNICATION_LOST = "communication_lost"
    EMERGENCY_STOP = "emergency_stop"


class Event(Enum):
    """Everything that can move the machine.  All of it arrives from outside."""

    VALIDATE_OK = "validate_ok"
    VALIDATE_FAILED = "validate_failed"
    START = "start"
    ROW_END = "row_end"
    ROW_LOST = "row_lost"
    LINK_LOST = "link_lost"
    CAMERA_TIMEOUT = "camera_timeout"
    COMMAND_TIMEOUT = "command_timeout"
    COMMUNICATION_LOST = "communication_lost"
    ESTOP = "estop"
    ESTOP_RELEASED = "estop_released"
    ACK = "ack"


class IllegalTransition(Exception):
    """An event the current state has no edge for.

    Raised rather than ignored: a machine that silently drops a transition ends
    up somewhere nobody asked for, and the call site that fired the event goes
    on believing it landed.
    """


#: Faults out of DRIVING_ROW, and the code each one lands in ERROR with.
_FAULTS: dict[Event, ErrorCode] = {
    Event.ROW_LOST: ErrorCode.ROW_LOST,
    Event.LINK_LOST: ErrorCode.LINK_LOST,
    Event.CAMERA_TIMEOUT: ErrorCode.CAMERA_TIMEOUT,
    Event.COMMAND_TIMEOUT: ErrorCode.COMMAND_TIMEOUT,
    Event.COMMUNICATION_LOST: ErrorCode.COMMUNICATION_LOST,
}

#: Every edge §5.3 draws, minus the two that carry a condition or a payload:
#: ``ESTOP`` (legal from everywhere) and ``ACK`` out of ``ESTOP`` (needs the
#: button released first).  Both are handled explicitly in :meth:`fire`.
_TRANSITIONS: dict[tuple[RoverState, Event], RoverState] = {
    (RoverState.BOOT, Event.VALIDATE_OK): RoverState.READY,
    (RoverState.BOOT, Event.VALIDATE_FAILED): RoverState.ERROR,
    (RoverState.READY, Event.START): RoverState.DRIVING_ROW,
    (RoverState.DRIVING_ROW, Event.ROW_END): RoverState.STOPPED,
    **{(RoverState.DRIVING_ROW, event): RoverState.ERROR for event in _FAULTS},
    (RoverState.STOPPED, Event.ACK): RoverState.READY,
    (RoverState.ERROR, Event.ACK): RoverState.READY,
}


class StateMachine:
    """The transition table of §5.3, and the reason for the current state.

    ``stop_reason`` and ``error_code`` are separate fields rather than one
    ``reason``, because the scenario contract asserts them separately and a
    normal stop must report ``error_code: null``.
    """

    def __init__(self) -> None:
        self._state = RoverState.BOOT
        self._stop_reason: str | None = None
        self._error_code: ErrorCode | None = None
        self._estop_released = False

    @property
    def state(self) -> RoverState:
        return self._state

    @property
    def stop_reason(self) -> str | None:
        """Why the run ended normally.  ``None`` unless the state is STOPPED."""
        return self._stop_reason

    @property
    def error_code(self) -> ErrorCode | None:
        """Which §8.5 code put the machine in ERROR or ESTOP."""
        return self._error_code

    @property
    def estop_released(self) -> bool:
        """Whether the button has been reported released since it was pressed."""
        return self._estop_released

    def fire(self, event: Event) -> RoverState:
        """Apply ``event`` and return the resulting state.

        Raises :class:`IllegalTransition` — without changing anything — for any
        pair the diagram does not draw.
        """
        if event is Event.ESTOP:
            # Reachable from every state, BOOT and ERROR included, and it
            # re-latches: a button pressed again must not ride on the release
            # that came before it.
            self._state = RoverState.ESTOP
            self._stop_reason = None
            self._error_code = ErrorCode.EMERGENCY_STOP
            self._estop_released = False
            return self._state

        if self._state is RoverState.ESTOP:
            return self._fire_in_estop(event)

        destination = _TRANSITIONS.get((self._state, event))
        if destination is None:
            raise IllegalTransition(f"{self._state.name} has no edge for {event.value!r}")

        self._state = destination
        self._stop_reason = ROW_END_SUSPECTED if event is Event.ROW_END else None
        if event is Event.VALIDATE_FAILED:
            self._error_code = ErrorCode.CONFIG_INVALID
        else:
            self._error_code = _FAULTS.get(event)
        return self._state

    def _fire_in_estop(self, event: Event) -> RoverState:
        """Leaving ESTOP needs the button released *and* an ack, in that order.

        Two steps because they are two different facts: one is about the
        hardware, the other about a person deciding it is safe to carry on.
        Collapsing them into one event would let software clear a latched
        E-stop on its own.
        """
        if event is Event.ESTOP_RELEASED:
            self._estop_released = True
            return self._state

        if event is Event.ACK:
            if not self._estop_released:
                raise IllegalTransition(
                    "ESTOP has no edge for 'ack' while the button is still held — "
                    "it needs 'estop_released' first"
                )
            self._state = RoverState.READY
            self._stop_reason = None
            self._error_code = None
            self._estop_released = False
            return self._state

        raise IllegalTransition(f"ESTOP has no edge for {event.value!r}")
