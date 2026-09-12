"""Rover backend for Isaac Sim.  Skeleton — the adapter lands at V1.

Construction stays side-effect free on purpose: tests/unit/ must be able to ask
this class for its drive-state shape without a GPU or a running simulator.  The
moment the constructor reaches for Isaac, the cross-backend shape test stops
being runnable in CI.

At V1 this talks to sim/isaac/adapters/, which owns the unit conversion at the
boundary (mm -> m, deg/s -> rad/s) and applies controller.rover.mixing to get
wheel joint velocities.
"""

from __future__ import annotations

from controller.rover.base import DriveStateOwner

_NOT_YET = "IsaacRover is a V1 skeleton — sim/isaac/adapters/ does not exist yet"


class IsaacRover(DriveStateOwner):
    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        raise NotImplementedError(_NOT_YET)

    def stop(self) -> None:
        raise NotImplementedError(_NOT_YET)

    def emergency_stop(self) -> None:
        raise NotImplementedError(_NOT_YET)
