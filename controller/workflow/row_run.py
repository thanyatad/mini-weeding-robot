"""One row, start to stop — the control loop of spec §5.5.

::

    while state is DRIVING_ROW:
        est = row_estimator.estimate(cameras.front())    # 10 Hz
        if not est.valid:
            watchdog.tick(est)
            continue
        watchdog.reset()
        rover.drive(*follower.step(est))
        weed_log.maybe_record(cameras.down())            # 1-2 Hz, off the control path

This module is the wiring and nothing more: the estimator produces, the
watchdog counts, the follower computes, the rover moves, and a watchdog trip
becomes a state transition.  Every piece it joins is separately testable, and
none of them knows about any of the others.

Three shapes worth reading twice:

**An invalid estimate commands nothing.**  §5.5 `continue`s past the rover, so
the standing command carries on for up to ``row_loss_frames`` — that is the
runaway budget invariant 2 reserves, spent on purpose, and it is what lets a
gap mid-row be ridden out instead of stopped on.

**``command_timeout`` never resumes.**  When the ESP32 reports it, the motors
are already dead; the controller treats the rover as genuinely stopped, goes to
ERROR and sends nothing — not even a ``stop()``.  Issuing a ``drive`` on the
next iteration would be a rover that starts moving with nobody having asked it
to (§8.5).

**The E-stop is not handled here.**  Cutting the motor rail belongs to the
hardware layer (§5.6) and the ESTOP transition belongs to the state machine.  A
controller that latched it in software would have taken over a layer that is
meant to work whether or not this process is alive.

Not here yet: ``weed_log``, which is V2.  The place it goes is marked in
:meth:`RowRun.step`, at the end of the iteration and off the control path — a
slow or missed detection must never make the rover steer wrong, which is why
the front camera runs at 10 Hz and the down camera at 1-2 Hz.
"""

from __future__ import annotations

from collections.abc import Callable

from controller.motion import RowEstimate, RowFollower
from controller.rover import Rover
from controller.safety import RowLossVerdict, RowLossWatchdog
from controller.workflow.state_machine import Event, RoverState, StateMachine


class NotDriving(Exception):
    """A control-loop iteration was attempted outside DRIVING_ROW.

    Raised rather than ignored so that a loop which has already ended cannot go
    on calling ``step()`` and believing it is running.  Nothing is commanded on
    the way out.
    """


#: A watchdog verdict, as the event the state machine takes.  Two names for the
#: same two outcomes is one name too many; this is where they meet.
_VERDICT_EVENTS: dict[RowLossVerdict, Event] = {
    RowLossVerdict.ROW_END_SUSPECTED: Event.ROW_END,
    RowLossVerdict.ROW_LOST: Event.ROW_LOST,
}

#: Faults detected outside the control loop.  Their detectors are the link
#: monitor and the protocol codec (V3); only the handling lives here.
FAULT_EVENTS = frozenset(
    {
        Event.LINK_LOST,
        Event.CAMERA_TIMEOUT,
        Event.COMMAND_TIMEOUT,
        Event.COMMUNICATION_LOST,
    }
)


class RowRun:
    """Drives one row and stops, which is the whole of the MVP (§10.6).

    Turning into the next row would mean turning towards something the camera
    cannot see, with no odometry to do it blind — that is M4, not a knob
    missing here.
    """

    def __init__(
        self,
        machine: StateMachine,
        rover: Rover,
        follower: RowFollower,
        watchdog: RowLossWatchdog,
    ) -> None:
        self.machine = machine
        self.rover = rover
        self.follower = follower
        self.watchdog = watchdog

    @property
    def state(self) -> RoverState:
        return self.machine.state

    def start(self) -> RoverState:
        """Begin the run.  Only legal from READY, i.e. after validation passed."""
        return self.machine.fire(Event.START)

    def run(
        self,
        estimator: Callable[[], RowEstimate],
        max_frames: int,
    ) -> RoverState:
        """Step until the run leaves DRIVING_ROW, or ``max_frames`` are spent.

        No sleeping and no clock: the loop runs at the rate ``estimator``
        returns frames at, which on the real rover is the front camera at 10 Hz.
        ``max_frames`` is the caller's bound on a row that never ends — the
        rover knows nothing about it.
        """
        for _ in range(max_frames):
            if self.machine.state is not RoverState.DRIVING_ROW:
                break
            self.step(estimator())
        return self.machine.state

    def step(self, est: RowEstimate) -> RoverState:
        """One iteration of §5.5.  Returns the state it leaves the run in."""
        if self.machine.state is not RoverState.DRIVING_ROW:
            raise NotDriving(
                f"the run is in {self.machine.state.name}, not DRIVING_ROW — nothing was commanded"
            )

        if not est.valid:
            verdict = self.watchdog.tick(est)
            if verdict is not None:
                # The watchdog has already braked; this records why.
                self.machine.fire(_VERDICT_EVENTS[verdict])
            return self.machine.state

        self.watchdog.reset()
        self.rover.drive(*self.follower.step(est))
        # V2: weed_log.maybe_record(cameras.down()) belongs here — after the
        # command, at 1-2 Hz, off the control path.
        return self.machine.state

    def report_fault(self, event: Event) -> RoverState:
        """Hand the run a fault that was detected outside the control loop.

        The transition is taken first: if the run is not in DRIVING_ROW the
        state machine rejects it and nothing has been commanded.
        """
        if event not in FAULT_EVENTS:
            raise ValueError(
                f"{event.value!r} is not a fault detected outside the loop — "
                f"expected one of {sorted(e.value for e in FAULT_EVENTS)}"
            )

        state = self.machine.fire(event)

        if event is not Event.COMMAND_TIMEOUT:
            # command_timeout is the exception: the ESP32 has already killed the
            # motors, so there is nothing left to stop and nothing to say to it
            # (§8.5).  For the rest the wheels may still be turning.
            self.rover.stop()

        return state
