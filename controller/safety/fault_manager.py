"""Where a §8.5 wire code becomes a §5.3 workflow event, and gets written down.

``bridge/`` carries faults and does not decide what they mean: ``Fault.code`` is
a string off the wire, deliberately not a ``controller.workflow.ErrorCode``.
``RowRun.report_fault()`` decides what a fault *does* — the transition first,
then the ``stop()`` for everything except ``command_timeout``.  Between the two
there was a translation nobody owned, and it was inlined in an integration
bench: the shipped path was the one thing the tests were not exercising.

This module is that translation, and nothing more.

Routes, records, decides nothing
--------------------------------
It calls no rover method at all.  A manager that braked would be a second copy
of ``report_fault()``'s rule, and the copy that is not ``report_fault()`` is the
one that keeps doing what it always did after the other changes.  Same lesson
as ``link_monitor``, and asserted the same way: on a ``RecordingRover``, where
a second ``stop()`` in the journal is visible.

It does not check the run's state first either.  ``report_fault()`` takes the
transition before it commands anything, and the state machine refuses an event
the current state has no edge for — with nothing sent to the rover on the way
out.  A guard here would be a second copy of that rule.

Two destinations, because there are two kinds of fault
------------------------------------------------------
The four faults detected outside the control loop end a row, and ``RowRun``
owns what that means.  ``emergency_stop`` is not one of them: ``FAULT_EVENTS``
leaves the button out because ``ESTOP`` is legal from every state, including
``BOOT`` and ``ERROR``, where there is no run to hand anything to — so it goes
straight to the machine, which is the only thing that owns the ESTOP edge.
There is no braking to skip: the rail was cut electrically before this code ran
(§11.4).

The trap this table exists for
------------------------------
``Event(fault.code)`` looks like it would do the job, and does for four of the
seven codes.  ``ErrorCode.EMERGENCY_STOP`` is ``"emergency_stop"`` while the
event is ``Event.ESTOP`` — ``"estop"`` — so the shortcut raises on the one
fault that matters most, and a router that swallowed the failure would do
nothing at all for the button.  Hence a table written out by hand.

The two codes with no wire path
-------------------------------
``row_lost`` and ``config_invalid`` are in the table, because the table is the
§8.5 ↔ §5.3 dictionary and a partial dictionary is a worse thing to read than
none.  They are refused at dispatch:

``row_lost``
    the row-loss watchdog's verdict.  It brakes and *then* fires, so routing it
    from here would be a second, brakeless path to a transition that already
    has an owner.

``config_invalid``
    startup validation, before a run exists.  A board has no opinion about this
    controller's config file.

Neither has ever arrived from the wire.  Refusing them loudly means the day one
does, somebody finds out — which is the whole difference between a router and a
place faults go to be forgotten.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import TYPE_CHECKING

from controller.workflow import Event
from controller.workflow.row_run import FAULT_EVENTS

if TYPE_CHECKING:  # pragma: no cover - typing only
    from bridge.protocol import Fault
    from controller.workflow import RowRun


class UnroutableFault(Exception):
    """A fault code this controller has no route for.

    Raised rather than ignored, for the same reason ``IllegalTransition`` is: a
    router that swallowed a code it did not recognise would leave the caller
    believing the fault had been handled, while whatever produced it carries on
    going wrong.
    """


#: Every code ``protocol/schema.json`` defines, and the event §5.3 draws for it.
FAULT_CODE_EVENTS: dict[str, Event] = {
    "row_lost": Event.ROW_LOST,
    "link_lost": Event.LINK_LOST,
    "camera_timeout": Event.CAMERA_TIMEOUT,
    "command_timeout": Event.COMMAND_TIMEOUT,
    "config_invalid": Event.VALIDATE_FAILED,
    "communication_lost": Event.COMMUNICATION_LOST,
    # Not ``Event("emergency_stop")``, which does not exist.  The code and the
    # event are spelled differently, and this line is the only place that knows.
    "emergency_stop": Event.ESTOP,
}

#: Codes that exist in §8.5 but never arrive from the board, and the owner each
#: one actually belongs to.  See the module docstring.
_NO_WIRE_PATH: dict[str, str] = {
    "row_lost": (
        "the row-loss watchdog's verdict, not a fault the board reports — it brakes "
        "and then fires ROW_LOST itself (controller/safety/runaway.py)"
    ),
    "config_invalid": (
        "raised by startup validation before a run exists, not by the board "
        "(controller/startup_checks.py)"
    ),
}


class FaultManager:
    """Turns the faults ``Esp32Rover.poll()`` returns into workflow events."""

    def __init__(self, run: RowRun) -> None:
        self._run = run
        #: Every fault that arrived, in order, carrying the ``message`` and
        #: ``id`` that are dropped the moment a code becomes an ``Event`` — and
        #: which are the whole of what makes a log entry worth reading later.
        self.history: list[Fault] = []

    def handle(self, fault: Fault) -> Event:
        """Record ``fault``, route it, and return the event it was routed as.

        Recorded on arrival rather than on success: a fault that could not be
        routed is the one most worth finding in the log afterwards.
        """
        self.history.append(fault)

        reason = _NO_WIRE_PATH.get(fault.code)
        if reason is not None:
            raise UnroutableFault(f"{fault.code!r} is {reason}")

        try:
            event = FAULT_CODE_EVENTS[fault.code]
        except KeyError:
            raise UnroutableFault(
                f"{fault.code!r} is not one of the seven error codes of §8.5 — "
                f"expected one of {sorted(FAULT_CODE_EVENTS)}"
            ) from None

        if event in FAULT_EVENTS:
            self._run.report_fault(event)
        else:
            # ESTOP, and only ESTOP.  Straight to the machine: it is legal from
            # every state, report_fault() rejects it, and there is nothing left
            # to brake.
            self._run.machine.fire(event)

        return event

    def handle_all(self, faults: Iterable[Fault]) -> list[Event]:
        """Route a whole ``poll()`` in order, and return what each became."""
        return [self.handle(fault) for fault in faults]
