"""The row-loss watchdog — the controller's layer of the three in §5.6.

    Command timeout    ESP32 firmware    motors die on their own
    Row-loss watchdog  controller        this module
    Latching E-stop    hardware          cuts the motor rail electrically

Each layer works without the one above it: the ESP32 cuts the motors whether or
not this process is alive, and the E-stop cuts the rail whether or not the
firmware is.  So this module assumes the other two exist rather than trying to
do their jobs — it counts estimates and it brakes, and it never reaches for
``emergency_stop()``, which belongs to a relay and a person.

It trips at exactly ``safety.row_loss_frames`` consecutive invalid estimates.
Exactly, because startup invariant 2 is enforced against that number::

    v_max x row_loss_frames / loop_hz <= runaway_budget
    100  x 3                / 10      = 30 mm <= 60 mm

A watchdog that trips one frame late makes that invariant a description of a
runaway distance the code does not honour, and nothing else in the system is
placed to notice.  The frames before the trip cost that budget deliberately:
nothing is commanded there, so a gap mid-row is ridden out rather than stopped
on.

It brakes with ``stop()``, not ``drive(0, 0)``.  Both refresh the firmware's
command timeout, but ``stop()`` holds the brake while ``drive(0, 0)`` coasts,
and on a slope those are different outcomes (docs/architecture.md).

Which way it trips (§5.4)
-------------------------
Both endings arrive as the same thing — invalid estimates, ``row_loss_frames``
of them in a row — and ``green_fraction`` is the only signal that separates
them:

===================================  =================  ==========================
``green_fraction`` on the trip       reading            verdict
===================================  =================  ==========================
below ``green_fraction_row_end``     the green is gone  ``ROW_END_SUSPECTED``
at or above it                       green, but no line ``ROW_LOST``
===================================  =================  ==========================

**The threshold is unvalidated.**  ``perception.row_estimator.green_fraction_
row_end`` is an assumption, not a measurement: telling a gap mid-row apart from
the end of a row needs rendered frames with crops in them, which is V1/V2 work
(``crop_gap_midrow.yaml`` and ``row_end.yaml``, the opposed pair that has to
pass both ways).  What is tested at V0 is the comparison, the counting and the
braking — not whether 0.10 is the right place to put the line.

The equal case is reported as ``ROW_LOST``.  A navigation failure filed as a
normal end of run is the direction that hides the problem; the reverse merely
asks a human to look.
"""

from __future__ import annotations

from enum import Enum
from typing import Any

from controller.motion import RowEstimate
from controller.rover import Rover


class RowLossVerdict(Enum):
    """What the watchdog concluded when it ran out of frames.

    The values are the strings the scenario contract asserts, so a verdict can
    be read straight into ``stop_reason`` / ``error_code`` without a second
    naming of the same two outcomes.
    """

    ROW_END_SUSPECTED = "row_end_suspected"
    ROW_LOST = "row_lost"


class RowLossWatchdog:
    """Counts consecutive invalid estimates and halts the rover at the limit."""

    def __init__(
        self,
        rover: Rover,
        row_loss_frames: int,
        green_fraction_trip: float,
    ) -> None:
        if row_loss_frames < 1:
            raise ValueError(
                f"row_loss_frames must be at least 1, got {row_loss_frames} — "
                f"a watchdog that cannot trip is not a safety layer"
            )
        self._rover = rover
        self.row_loss_frames = int(row_loss_frames)
        self.green_fraction_trip = float(green_fraction_trip)
        self._consecutive_invalid = 0
        self._tripped = False

    @classmethod
    def from_config(cls, rover: Rover, config: dict[str, Any]) -> RowLossWatchdog:
        from controller.config import get

        return cls(
            rover,
            row_loss_frames=get(config, "safety.row_loss_frames"),
            green_fraction_trip=get(config, "perception.row_estimator.green_fraction_row_end"),
        )

    @property
    def consecutive_invalid(self) -> int:
        return self._consecutive_invalid

    def tick(self, est: RowEstimate) -> RowLossVerdict | None:
        """Account for one estimate; return a verdict on the frame that trips.

        ``None`` on every other frame, including every frame after the trip:
        the rover is already stopped, and a verdict repeated each loop would
        read like a fresh one.  §5.5 hands only invalid estimates here and calls
        :meth:`reset` on valid ones, but a valid estimate arriving here resets
        too — the count means *consecutive*, and there is no reading of the
        control loop under which it should not.
        """
        if est.valid:
            self.reset()
            return None

        self._consecutive_invalid += 1
        if self._consecutive_invalid < self.row_loss_frames or self._tripped:
            return None

        self._tripped = True
        self._rover.stop()
        return self._verdict(est)

    def reset(self) -> None:
        """Clear the count.  Called on every valid estimate (§5.5)."""
        self._consecutive_invalid = 0
        self._tripped = False

    def _verdict(self, est: RowEstimate) -> RowLossVerdict:
        """Read §5.4's table on the frame that ran the counter out.

        That frame, and not an average over the window: it is the one §5.4
        describes, and averaging would be a filter nobody has specified.
        """
        if est.green_fraction < self.green_fraction_trip:
            return RowLossVerdict.ROW_END_SUSPECTED
        return RowLossVerdict.ROW_LOST
