"""A rover that keeps a journal of what was asked of it.

``stop()`` and ``drive(0, 0)`` leave ``FakeRover`` in exactly the same place —
it is kinematic, and models neither braking nor coasting — so the only way to
assert that the watchdog brakes rather than coasts is to look at which call was
made.  On a slope that difference is real (docs/architecture.md), which is why
it is worth asserting at all.

This is a subclass, not a mock: it integrates a pose, it honours the ``Rover``
protocol, and a method that quietly stopped existing would fail here rather
than pass.
"""

from __future__ import annotations

from controller.rover import FakeRover


class RecordingRover(FakeRover):
    """``FakeRover`` plus ``calls``, the sequence of commands it received."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.calls: list[tuple] = []

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        self.calls.append(("drive", v_mm_s, omega_deg_s))
        super().drive(v_mm_s, omega_deg_s)

    def stop(self) -> None:
        self.calls.append(("stop",))
        super().stop()

    def emergency_stop(self) -> None:
        self.calls.append(("emergency_stop",))
        super().emergency_stop()

    @property
    def drive_calls(self) -> list[tuple]:
        return [call for call in self.calls if call[0] == "drive"]
