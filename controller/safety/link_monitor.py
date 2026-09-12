"""The link-age detector: ``link_age_ms`` past its ceiling becomes LINK_LOST.

This is the Pi's half of the pair §8.5 insists on having both of.  The ESP32
kills the motors when no ``drive`` has arrived for ``command_timeout_ms``; this
notices that the board has stopped confirming what it receives.  Both are
needed: with only the Pi's half, a Pi that crashes leaves nobody to order a
stop.

    command_timeout_ms  300    the board gives up on the Pi
    link_lost_ms        500    the Pi gives up on the board

That ordering is deliberate and must stay (config/README.md).  The motors are
dead before the controller declares the fault, so the log reads in causal
order.  Reversed, the controller enters ERROR while the wheels keep turning for
another 200 ms — which is why this module never reaches for the smaller number
to make a test agree.

It detects and hands over; it does not handle
---------------------------------------------
``RowRun.report_fault()`` already owns what a fault does — the transition, and
the ``stop()`` that goes with everything except ``command_timeout``.  Nothing
here touches the rover.  Two places deciding the same thing is two places to
keep in step, and the one that is not the state machine will lose.

It does not check the run's state first, either.  ``report_fault()`` takes the
transition before it commands anything, so a fault arriving after the run has
ended is refused by the state machine with nothing sent to the rover.  A guard
here would be a second copy of that rule.

Where the number comes from
---------------------------
``link_age_ms`` is measured by ``bridge.seq_tracker`` from the ``last_seq`` the
board echoes, using the Pi's own clock at both ends.  This module is handed the
result: it needs one number, and keeping it that way is what makes it testable
against a synthetic timeline instead of a link.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover - typing only
    from controller.workflow import RowRun


class LinkMonitor:
    """Trips once when ``link_age_ms`` reaches ``safety.link_lost_ms``."""

    def __init__(self, run: RowRun, link_lost_ms: float) -> None:
        if link_lost_ms <= 0:
            raise ValueError(
                f"link_lost_ms must be positive, got {link_lost_ms} — a monitor that "
                f"trips at zero declares every link lost"
            )
        self._run = run
        self.link_lost_ms = float(link_lost_ms)
        self._tripped = False

    @classmethod
    def from_config(cls, run: RowRun, config: dict[str, Any]) -> LinkMonitor:
        from controller.config import get

        return cls(run, link_lost_ms=get(config, "safety.link_lost_ms"))

    @property
    def tripped(self) -> bool:
        return self._tripped

    def tick(self, link_age_ms: float) -> bool:
        """Account for one reading of the link age.  ``True`` on the trip only.

        Once only: a lost link does not come back on its own, so the age goes on
        climbing, and a detector that fired every loop would ask the state
        machine to leave ERROR by the same edge it just took.
        """
        if self._tripped or link_age_ms < self.link_lost_ms:
            return False

        self._tripped = True
        # Imported here rather than at module scope: controller.workflow imports
        # controller.safety for the row-loss watchdog, and at call time both
        # packages are fully built.
        from controller.workflow import Event

        self._run.report_fault(Event.LINK_LOST)
        return True

    def reset(self) -> None:
        """Arm it again for another run."""
        self._tripped = False
