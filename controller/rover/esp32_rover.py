"""Rover backend for the real ESP32 over serial.  Skeleton — lands at V3.

Construction stays side-effect free on purpose: it must not open a port.
tests/unit/ constructs this class to check that its drive-state shape matches
the other backends, and that test has to run with no board attached.

At V3 this talks to bridge/, which owns the serial framing, the ``seq`` /
``last_seq`` exchange that ``link_age_ms`` is derived from, and the latest-wins
rule that the firmware enforces on its side.

Note what this backend will NOT be able to add: a measured wheel speed.  The
ESP32 mixes and drives open-loop, so ``get_drive_state()`` here carries exactly
what it carries everywhere else.
"""

from __future__ import annotations

from controller.rover.base import DriveStateOwner

_NOT_YET = "Esp32Rover is a V3 skeleton — bridge/ does not exist yet"


class Esp32Rover(DriveStateOwner):
    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        raise NotImplementedError(_NOT_YET)

    def stop(self) -> None:
        raise NotImplementedError(_NOT_YET)

    def emergency_stop(self) -> None:
        raise NotImplementedError(_NOT_YET)

    def _link_age_ms(self) -> int:
        """Age of the last acknowledged exchange with the board.

        Zero while there is no link at all.  At V3 this comes from bridge/ via
        the ``seq`` / ``last_seq`` echo, which measures the round trip without
        either side needing a synchronised clock.
        """
        return 0
