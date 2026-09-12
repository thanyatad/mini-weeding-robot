"""The E-stop sense line: an edge on the wire becomes an event in the machine.

    press_estop()  ->  rover_state["estop"]  ->  get_drive_state()["estop"]  ->  here

By the time this runs, the rover has already stopped.  The button cuts the
motor rail electrically (§11.4) and the ESP32 latches on its own account; the
sense line is a *report* of that, not a request for it.  So this module records
what happened and does nothing about it — which is the whole of its job.

It detects and hands over; it does not handle
---------------------------------------------
Nothing here touches the rover.  A detector that called ``emergency_stop()``
would have the software layer doing the hardware layer's work, and §5.6 is
explicit that each of the three layers has to work without the one above it: a
layer that leans on the layer above is not a safety layer, it is a feature.
The failure that makes this concrete is a button that was never wired — a
controller that brakes on its own hides it until somebody presses the thing.

Nor does it own a latch.  ``StateMachine`` already holds one, ``ESTOP``
re-latches there, and ``estop_released`` is read from there.  Two answers to
"has the button come up" is one too many, and the copy that is not the state
machine is the one that will be stale.

Two facts, not one
------------------
Leaving ESTOP takes ``ESTOP_RELEASED`` *then* ``ACK``.  They are different
kinds of fact: one is about a wire, the other about a person deciding it is
safe to carry on.  This detector can see the wire and can never see the person,
so it fires the first and never the second.  A detector that fired both would
let software clear a latched E-stop on its own.

Why an edge and not a level
---------------------------
The line stays active for as long as somebody holds the button down.  Firing
every tick would re-latch a latch that never came up, and would bury the one
press worth reading under a hundred repeats of it.  The only state kept here is
the previous reading, which is what makes an edge an edge.

Why it is handed a bool
-----------------------
Like ``link_monitor``, it takes the reading rather than fetching it: the caller
does ``detector.tick(rover.get_drive_state()["estop"])``.  One value in means
the whole of this module can be tested against a synthetic sequence of presses,
with no rover, no board and no wire.

It fires on the ``StateMachine`` and not on the ``RowRun``, because ``ESTOP``
is legal from ``BOOT`` and ``ERROR`` — states in which there is no run to hand
it to.  ``RowRun.report_fault()`` handles the faults that end a row; the button
is not one of them, and ``FAULT_EVENTS`` deliberately leaves it out.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - typing only
    from controller.workflow import Event, StateMachine


class EmergencyStop:
    """Turns the sense line into ``ESTOP`` on the way down and
    ``ESTOP_RELEASED`` on the way up."""

    def __init__(self, machine: StateMachine) -> None:
        self._machine = machine
        #: The line as last read.  Starts inactive so that a rover booting with
        #: the button already held reads as a press rather than as the steady
        #: state — landing in ESTOP, which is where it belongs.
        self._active = False

    def tick(self, estop: bool) -> Event | None:
        """Account for one reading of the sense line.

        Returns the event fired, or ``None`` when the line has not changed.
        The state it leaves the machine in is the machine's to decide: an event
        the current state has no edge for raises :class:`IllegalTransition`
        from there, and no guard is kept here for the same reason the link
        monitor keeps none — that rule already has exactly one home.
        """
        active = bool(estop)
        if active == self._active:
            return None

        self._active = active
        # Imported here rather than at module scope: controller.workflow imports
        # controller.safety for the row-loss watchdog, and at call time both
        # packages are fully built.
        from controller.workflow import Event

        event = Event.ESTOP if active else Event.ESTOP_RELEASED
        self._machine.fire(event)
        return event
