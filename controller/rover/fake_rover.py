"""Kinematic-only rover: integrate (v, omega) into an internal pose.

No physics, no images, no graphics — the whole control loop can be closed in
milliseconds inside tests/unit/, which is what makes "do these gains converge,
and do they swing out of the furrow" a question answerable before Isaac exists.

The pose is deliberately NOT part of the ``Rover`` interface.  ``FakeRowSensor``
reads it because it is the test harness; if anything in ``workflow/``,
``motion/`` or ``safety/`` grew to depend on it, that code would break the day
the backend became ``esp32`` — which has no pose to give — and it would break
in the field, where debugging is most expensive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from controller.rover.base import DriveStateOwner


@dataclass(frozen=True)
class Pose:
    """Rover pose in the bed frame.  Harness-only: nothing in the real system
    knows this, because the ``bed -> rover`` transform does not exist at
    runtime (hardware/mechanical/coordinate-frames.md)."""

    x_mm: float
    y_mm: float
    heading_deg: float


class FakeRover(DriveStateOwner):
    """A rover that moves exactly as commanded.

    No slip, no deadband, no saturation — a real skid-steer has all three.  The
    point is not fidelity; it is that the control loop's geometry and signs can
    be tested without any of them in the way.
    """

    def __init__(self, x_mm: float = 0.0, y_mm: float = 0.0, heading_deg: float = 0.0) -> None:
        super().__init__()
        self._x_mm = float(x_mm)
        self._y_mm = float(y_mm)
        self._heading_deg = float(heading_deg)

    # -- Rover interface ---------------------------------------------------

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        self._record_command(v_mm_s, omega_deg_s)

    def stop(self) -> None:
        self._record_command(0.0, 0.0)

    def emergency_stop(self) -> None:
        self._estop = True
        self._commanded_v_mm_s = 0.0
        self._commanded_omega_deg_s = 0.0

    # -- harness only ------------------------------------------------------

    @property
    def pose(self) -> Pose:
        """Test-harness access only — see the module docstring."""
        return Pose(self._x_mm, self._y_mm, self._heading_deg)

    def step(self, dt_s: float) -> None:
        """Advance the pose by ``dt_s`` under the standing command.

        Unicycle integration at the midpoint heading, which keeps a constant-
        omega arc exact rather than accumulating the error a forward Euler step
        would leave behind at 10 Hz.
        """
        v_mm_s = self._commanded_v_mm_s
        omega_deg_s = self._commanded_omega_deg_s

        heading_mid_rad = math.radians(self._heading_deg + 0.5 * omega_deg_s * dt_s)
        self._x_mm += v_mm_s * math.cos(heading_mid_rad) * dt_s
        self._y_mm += v_mm_s * math.sin(heading_mid_rad) * dt_s
        self._heading_deg += omega_deg_s * dt_s
