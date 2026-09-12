"""Joint handles for the four wheels: fl, fr, rl, rr.

Two things live here.  The names and the left/right grouping are plain data and
are checked against ``cad/urdf/weeding_rover.urdf`` in ``tests/unit`` without a
GPU - they are what rots when CAD changes, and a check that needs Isaac running
is a check nobody runs.  ``WheelJoints`` is the thin part that touches a live
articulation, and it imports Isaac lazily so that importing this module stays
free.

Nothing here sets a drive gain, a damping value or a friction coefficient.
Those belong in ``rover.usd``, the committed override layer, because
``rover_base.usd`` is regenerated from the URDF and anything written into it is
destroyed the next time CAD changes.  See cad/README.md.
"""

from __future__ import annotations

from typing import Any

#: Declaration order.  This is the order Isaac's DOF arrays are built in below,
#: so it is a contract, not a cosmetic choice.
WHEEL_JOINTS: tuple[str, ...] = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")

#: +Y is left (hardware/mechanical/coordinate-frames.md).  Swapping these two
#: tuples builds a rover that turns the wrong way and passes every test that
#: only looks at magnitudes, so tests/unit/test_wheels.py reads the sign of Y
#: straight out of the URDF and compares.
LEFT_JOINTS: tuple[str, ...] = ("wheel_fl", "wheel_rl")
RIGHT_JOINTS: tuple[str, ...] = ("wheel_fr", "wheel_rr")

_NO_ARTICULATION = (
    "WheelJoints has no articulation - build it with "
    "WheelJoints.from_stage(prim_path) after the world has been reset"
)


def joint_targets(left_rad_s: float, right_rad_s: float) -> dict[str, float]:
    """Fan one value per side out to two joints each, in :data:`WHEEL_JOINTS` order.

    The real rover wires its two motors per side in parallel, so front and rear
    physically cannot be commanded apart.  Taking two arguments rather than four
    makes that a property of the type instead of a rule to remember.
    """
    by_side = dict.fromkeys(LEFT_JOINTS, float(left_rad_s))
    by_side.update(dict.fromkeys(RIGHT_JOINTS, float(right_rad_s)))
    return {name: by_side[name] for name in WHEEL_JOINTS}


class WheelJoints:
    """Velocity targets onto a live articulation.

    Satisfies ``sim.isaac.adapters.wheel_adapter.WheelJointHandles``.
    """

    def __init__(self, articulation: Any | None) -> None:
        self._articulation = articulation
        self._dof_indices: list[int] | None = None
        self._max_efforts: Any | None = None

    @classmethod
    def from_stage(cls, prim_path: str) -> WheelJoints:
        """Wrap the articulation at ``prim_path``.

        Needs a world that has already been reset: the physics view does not
        exist before that, and ``get_dof_index`` on an uninitialised handle
        returns nothing useful rather than raising.
        """
        from isaacsim.core.prims import Articulation

        articulation = Articulation(prim_paths_expr=prim_path, name="weeding_rover")
        articulation.initialize()
        return cls(articulation)

    # -- WheelJointHandles -------------------------------------------------

    def set_side_velocities(self, left_rad_s: float, right_rad_s: float) -> None:
        """Targets are rad/s.  isaacsim.core converts to the degrees UsdPhysics
        authors angular drives in - see robots/rover/config.yaml."""
        import numpy as np

        articulation = self._require_articulation()
        targets = joint_targets(left_rad_s, right_rad_s)
        articulation.set_joint_velocity_targets(
            np.array([[targets[name] for name in WHEEL_JOINTS]], dtype=np.float32),
            joint_indices=self._indices(),
        )

    def hold(self) -> None:
        """Controlled stop: zero velocity target, drive still engaged.

        The damping set in ``rover.usd`` is what resists motion afterwards, so
        this holds the brake rather than coasting.
        """
        self.set_side_velocities(0.0, 0.0)

    def release(self) -> None:
        """E-stop: zero the joint's maximum effort so the motors make no torque.

        The real E-stop opens the motor rail electrically (config/safety.yaml
        is explicit that no config key can stand in for it), which leaves the
        wheels freewheeling rather than braked.
        """
        import numpy as np

        articulation = self._require_articulation()
        if self._max_efforts is None:
            self._max_efforts = articulation.get_max_efforts(joint_indices=self._indices())
        self.set_side_velocities(0.0, 0.0)
        articulation.set_max_efforts(
            np.zeros((1, len(WHEEL_JOINTS)), dtype=np.float32),
            joint_indices=self._indices(),
        )

    # -- internals ---------------------------------------------------------

    def _indices(self) -> list[int]:
        if self._dof_indices is None:
            articulation = self._require_articulation()
            self._dof_indices = [articulation.get_dof_index(name) for name in WHEEL_JOINTS]
        return self._dof_indices

    def _require_articulation(self) -> Any:
        if self._articulation is None:
            raise NotImplementedError(_NO_ARTICULATION)
        return self._articulation
