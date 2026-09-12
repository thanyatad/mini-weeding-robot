"""The ``Rover`` contract and the one place its drive state is built.

The controller must not know whether it is driving simulation or hardware — it
speaks only through this Protocol.  Backends: ``FakeRover``, ``IsaacRover``,
``Esp32Rover``.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class Rover(Protocol):
    """Velocity-commanded rover.  No position, no distance, no homing.

    Methods that would have to be faked are deliberately absent:
    ``drive_distance()`` could only be ``velocity x time`` without encoders,
    ``get_position()`` has nothing to answer with, ``home()`` has no absolute
    frame to home into.  An interface that can say what it cannot do is worth
    more than one with a full set of methods.
    """

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Command a body velocity.

        Must be repeated within ``safety.command_timeout_ms`` or the motors cut
        out on their own — that timeout belongs to the ESP32 and does not
        depend on this process being alive.
        """
        ...

    def stop(self) -> None:
        """Controlled stop, brake held.  ``drive()`` may follow.

        Different from ``drive(0, 0)``, which coasts: on a slope that is a real
        difference, not a stylistic one.
        """
        ...

    def get_drive_state(self) -> dict:
        """``{"commanded": {"v_mm_s", "omega_deg_s"}, "estop", "link_age_ms"}``."""
        ...

    def emergency_stop(self) -> None:
        """Cut motor enable and latch.  Cleared by hand, never by software."""
        ...


class DriveStateOwner:
    """Shared bookkeeping so the three backends cannot drift apart.

    ``get_drive_state()`` is implemented exactly once, here.  Every backend
    inherits it, which is what makes "all backends return an identical key set"
    a property of the code rather than a promise in a README.

    There is deliberately no ``measured`` field.  The rover has no encoders, so
    no one knows how fast the wheels actually turned; a bare ``v_mm_s`` would
    read like feedback.  Nesting under ``commanded`` tells the call site it is
    holding an echo of its own command.  ``link_age_ms`` is the only value in
    the struct that is genuinely measured.

    When encoders arrive at M2, add ``"measured": {...}`` here — additively,
    without touching the existing keys.
    """

    def __init__(self) -> None:
        self._commanded_v_mm_s = 0.0
        self._commanded_omega_deg_s = 0.0
        self._estop = False

    def get_drive_state(self) -> dict[str, Any]:
        return {
            "commanded": {
                "v_mm_s": float(self._commanded_v_mm_s),
                "omega_deg_s": float(self._commanded_omega_deg_s),
            },
            "estop": bool(self._estop),
            "link_age_ms": int(self._link_age_ms()),
        }

    def _record_command(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Latch what was commanded.  A latched E-stop forces it to zero: the
        motor rail is cut, so reporting anything else would be a lie."""
        if self._estop:
            self._commanded_v_mm_s = 0.0
            self._commanded_omega_deg_s = 0.0
            return
        self._commanded_v_mm_s = float(v_mm_s)
        self._commanded_omega_deg_s = float(omega_deg_s)

    def _link_age_ms(self) -> int:
        """Milliseconds since the last acknowledged exchange with the motors.

        Zero where there is no link to age (fake, in-process sim).  Backends
        with a real link override this.
        """
        return 0
