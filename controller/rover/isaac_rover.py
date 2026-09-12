"""Rover backend for Isaac Sim.

Construction stays side-effect free on purpose: ``IsaacRover()`` opens nothing,
reads no config and needs no simulator, which is what lets
tests/unit/test_rover_backends.py build one to check that its drive-state shape
matches the other two backends.  The moment the constructor reaches for Isaac,
that test stops being runnable without a GPU.

``sim/isaac/adapters/`` is the only thing this talks to.  The adapter owns the
unit conversion at the boundary (mm/s and deg/s in, rad/s out) and applies
``controller.rover.mixing`` to get wheel speeds, so the skid-steer formula
exists in exactly the two places it already did - here it is only called.

What this class deliberately cannot do
--------------------------------------
It cannot tell anyone where the rover is.  Isaac knows, and
hardware/mechanical/coordinate-frames.md is explicit that the controller must
never ask: the ``bed -> rover`` transform does not exist at runtime, there is no
localization, and code written against a pose works in simulation and fails in
the field.  tests/unit/test_coordinate_frames.py enforces that as a static
check, because by the time it fails at runtime it is failing in a vegetable bed.

It also has no measured velocity.  The rover has no encoders in simulation for
the same reason it has none on the bench: ``get_drive_state()`` carries an echo
of the command and nothing that reads like feedback.
"""

from __future__ import annotations

from typing import Any

from controller.rover.base import DriveStateOwner

_NO_ADAPTER = (
    "IsaacRover has no wheel adapter - build it with "
    "IsaacRover.from_config(config, wheels) inside a running Isaac stage"
)


class IsaacRover(DriveStateOwner):
    """Velocity commands into a live Isaac articulation."""

    def __init__(self, adapter: Any | None = None) -> None:
        super().__init__()
        self._adapter = adapter

    @classmethod
    def from_config(cls, config: dict[str, Any], joints: Any) -> IsaacRover:
        """Build the adapter from config and hand it the joint handles.

        Imported here rather than at module scope: ``sim/`` is the simulator's
        half of the system and importing it from ``controller/`` unconditionally
        would make the whole package need Isaac on the path.
        """
        from sim.isaac.adapters.wheel_adapter import WheelAdapter

        return cls(WheelAdapter.from_config(config, joints))

    @property
    def adapter(self) -> Any:
        return self._adapter

    # -- Rover interface ---------------------------------------------------

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Command a body velocity.  One value per side reaches two joints."""
        adapter = self._require_adapter()
        if self._estop:
            # Latched: the motor rail is open and nothing this process sends
            # can close it.  Commanding anyway would make the simulation safer
            # than the rover, which is the wrong direction for a difference.
            self._record_command(v_mm_s, omega_deg_s)
            return
        adapter.drive(v_mm_s, omega_deg_s)
        self._record_command(v_mm_s, omega_deg_s)

    def stop(self) -> None:
        """Controlled stop, brake held.  ``drive()`` may follow."""
        adapter = self._require_adapter()
        if not self._estop:
            adapter.brake()
        self._record_command(0.0, 0.0)

    def emergency_stop(self) -> None:
        """Cut motor enable and latch.  Cleared by hand, never by software."""
        self._require_adapter().cut()
        self._estop = True
        self._commanded_v_mm_s = 0.0
        self._commanded_omega_deg_s = 0.0

    # -- reporting ---------------------------------------------------------

    def wheel_velocities(self, v_mm_s: float, omega_deg_s: float) -> Any:
        """What ``drive()`` would send, without sending it.

        sim/isaac/app.py prints the target beside the measured result; asking
        for the number must not move the rover.
        """
        return self._require_adapter().wheel_velocities(v_mm_s, omega_deg_s)

    def _require_adapter(self) -> Any:
        if self._adapter is None:
            raise NotImplementedError(_NO_ADAPTER)
        return self._adapter
