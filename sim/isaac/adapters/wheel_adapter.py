"""(v, omega) from the ``Rover`` interface -> wheel joint velocity in Isaac.

This is one of the two places in the system where units change
(hardware/mechanical/coordinate-frames.md#unit-convention), and it is the only
thing ``controller/rover/isaac_rover.py`` is allowed to talk to.

What it does NOT do is mix.  The skid-steer formula is implemented twice
already - ``controller.rover.mixing`` and the C++ mirror in ``firmware/esp32``
- and both are pinned to ``config/drive_mixing_vectors.csv``.  A third copy
would drift, and the drift would show up as a rover that turns slightly wrong
in simulation and correctly on hardware, or the reverse.  So this module calls
``mix()`` and then divides by the wheel radius.  That is all.

    mm/s    -> rad/s    v_side / wheel_radius
    deg/s   -> rad/s    inside mix(), which takes degrees
    mm      -> m        the wheel radius: config says 70 mm diameter, the URDF
                        collision cylinder says 0.035 m.  They must agree.

Both wheels on a side get one value.  The real rover wires its two motors per
side in parallel, so front and rear cannot be commanded apart; emitting a
per-side value rather than a per-wheel one makes that a property of the type
instead of a rule someone has to remember.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from controller.config import get
from controller.rover.mixing import mix


@dataclass(frozen=True)
class SideVelocities:
    """Commanded wheel joint velocity per side, rad/s about the joint's +Y axis.

    Positive drives that side forward: the URDF gives every wheel the axis
    ``(0, 1, 0)``, so both sides share a sign and neither is mirrored.
    """

    left_rad_s: float
    right_rad_s: float


class WheelJointHandles(Protocol):
    """What the adapter needs from ``sim.isaac.robots.rover.wheels``.

    Kept as a Protocol so every test above ``tests/unit/`` can run the adapter
    without a GPU, an Isaac install, or a stage.
    """

    def set_side_velocities(self, left_rad_s: float, right_rad_s: float) -> None: ...

    def hold(self) -> None: ...

    def release(self) -> None: ...


class WheelAdapter:
    """Convert body velocity to wheel joint velocity, and push it at the joints.

    Constructed without handles it is a pure converter, which is how
    ``tests/unit/test_wheel_adapter.py`` exercises the arithmetic.
    """

    def __init__(
        self,
        joints: WheelJointHandles | None = None,
        *,
        track_width_mm: float,
        wheel_v_max_mm_s: float,
        wheel_radius_mm: float,
    ) -> None:
        if wheel_radius_mm <= 0:
            raise ValueError(f"wheel_radius_mm must be positive, got {wheel_radius_mm}")
        self._joints = joints
        self.track_width_mm = float(track_width_mm)
        self.wheel_v_max_mm_s = float(wheel_v_max_mm_s)
        self.wheel_radius_mm = float(wheel_radius_mm)

    @classmethod
    def from_config(
        cls, config: dict[str, Any], joints: WheelJointHandles | None = None
    ) -> WheelAdapter:
        return cls(
            joints,
            track_width_mm=get(config, "rover.track_width_mm"),
            wheel_v_max_mm_s=get(config, "rover.drive.wheel_v_max_mm_s"),
            wheel_radius_mm=get(config, "rover.wheel_diameter_mm") / 2.0,
        )

    # -- the conversion ----------------------------------------------------

    def wheel_velocities(self, v_mm_s: float, omega_deg_s: float) -> SideVelocities:
        """Mix, then divide by the wheel radius.  No formula lives here."""
        wheels = mix(
            v_mm_s,
            omega_deg_s,
            track_width_mm=self.track_width_mm,
            wheel_v_max_mm_s=self.wheel_v_max_mm_s,
        )
        return SideVelocities(
            left_rad_s=wheels.left_mm_s / self.wheel_radius_mm,
            right_rad_s=wheels.right_mm_s / self.wheel_radius_mm,
        )

    # -- the joints --------------------------------------------------------

    def drive(self, v_mm_s: float, omega_deg_s: float) -> SideVelocities:
        sides = self.wheel_velocities(v_mm_s, omega_deg_s)
        self._require_joints().set_side_velocities(sides.left_rad_s, sides.right_rad_s)
        return sides

    def brake(self) -> None:
        """Zero target with the drive still engaged - the damping holds the wheel."""
        self._require_joints().hold()

    def cut(self) -> None:
        """E-stop: drop the joint's maximum force to zero.

        The real E-stop opens the motor rail electrically (config/safety.yaml),
        so the motors produce no torque at all and the wheels freewheel.
        Commanding zero velocity instead would brake, which is a different and
        gentler thing than what the button does.
        """
        self._require_joints().release()

    def _require_joints(self) -> WheelJointHandles:
        if self._joints is None:
            raise NotImplementedError(
                "WheelAdapter has no joint handles - build it with "
                "WheelAdapter.from_config(config, wheels) inside a running stage"
            )
        return self._joints
