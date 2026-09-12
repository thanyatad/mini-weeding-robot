"""Row following: RowEstimate -> (v_mm_s, omega_deg_s).

Knows nothing about cameras and nothing about rovers.  It takes an estimate and
returns a tuple, which is what makes it testable with synthetic numbers — no
images, no Isaac.

Proportional, not PID.  ``heading_err`` is already the rate of change of
``lateral_err`` in geometry, so ``k_head`` acts as the damping term without
differentiating a noisy image signal.  No integral: a standing offset inside a
furrow is not worth the windup risk.  Speed is constant in the MVP — slowing
down when far off is a knob that can be added later.
"""

from __future__ import annotations

from typing import Any

from controller.motion.limits import clamp
from controller.motion.row_estimate import RowEstimate


class RowFollower:
    def __init__(
        self,
        k_lat: float,
        k_head: float,
        v_mm_s: float,
        omega_max_deg_s: float,
    ) -> None:
        self.k_lat = float(k_lat)
        self.k_head = float(k_head)
        self.v_mm_s = float(v_mm_s)
        self.omega_max_deg_s = float(omega_max_deg_s)

    @classmethod
    def from_config(cls, config: dict[str, Any], backend: str | None = None) -> RowFollower:
        """Build from the merged config, using the gains of the active backend.

        Gains are stored per backend and ``esp32`` is null until it has been
        tuned on the real thing.  A null here is a guard, not an oversight:
        skid-steer slip on soil is not skid-steer slip in sim, and the cheapest
        way to find that out is a startup failure rather than a field debug.
        """
        from controller.config import get

        backend = backend if backend is not None else get(config, "backend")
        gains = get(config, f"row_follower.gains.{backend}", None)
        if gains is None:
            raise ValueError(f"no row_follower gains configured for backend {backend!r}")

        k_lat, k_head = gains.get("k_lat"), gains.get("k_head")
        if k_lat is None or k_head is None:
            raise ValueError(
                f"row_follower gains for backend {backend!r} are not tuned "
                f"(k_lat={k_lat}, k_head={k_head}) — tune them on that backend and "
                f"write them into config/control.yaml; sim gains do not transfer"
            )

        return cls(
            k_lat=k_lat,
            k_head=k_head,
            v_mm_s=get(config, "row_follower.v_mm_s"),
            omega_max_deg_s=get(config, "row_follower.omega_max_deg_s"),
        )

    def step(self, est: RowEstimate) -> tuple[float, float]:
        """Return ``(v_mm_s, omega_deg_s)`` for one estimate.

        The caller checks ``est.valid`` first — an invalid estimate belongs to
        the row-loss watchdog, not here.  Keeping that decision out of this
        function is what keeps it a pure function of the numbers it is given.

        Negated because ``lateral_err > 0`` means the furrow lies to the right
        while ``omega > 0`` turns left.
        """
        omega_deg_s = -(self.k_lat * est.lateral_err + self.k_head * est.heading_err)
        omega_deg_s = clamp(omega_deg_s, -self.omega_max_deg_s, +self.omega_max_deg_s)
        return (self.v_mm_s, omega_deg_s)
