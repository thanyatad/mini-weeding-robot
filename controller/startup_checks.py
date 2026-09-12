"""The nine startup invariants.  Any one of them failing means the rover does
not start.

These are not warnings.  The MVP has no bumper switch, no ToF and no endstops;
arithmetic that agrees with itself is the substitute, and the only moment it
can be enforced cheaply is before anything moves.

Every message names the numbers that contradict each other and points at the
value to change.  An invariant that reports only its own name sends whoever is
reading back to the spreadsheet the invariant was meant to replace.

Full derivation of every number: config/README.md#startup-invariants
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from controller.config import get


class ConfigInvalid(Exception):
    """The config contradicts itself.  Startup stops here."""


@dataclass(frozen=True)
class Invariant:
    """One check.  ``check`` returns a message when violated, ``None`` when not."""

    name: str
    scope: str
    check: Callable[[dict[str, Any]], str | None]


def _fmt(value: float) -> str:
    """Print a number the way the documentation prints it: 30, not 30.0; 141.9,
    not 141.88799999999998."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return str(value)
    rounded = round(float(value), 1)
    return str(int(rounded)) if rounded == int(rounded) else str(rounded)


def _clear_furrow_mm(config: dict[str, Any]) -> float:
    """Width the rover can actually use.

    Not ``row_spacing``: foliage takes ``crop_foliage_half_width`` off each
    side, and every side-clearance invariant is written against what is left.
    """
    return get(config, "bed.row_spacing_mm") - 2 * get(config, "bed.crop_foliage_half_width_mm")


def _wheel_differential_mm_s(config: dict[str, Any]) -> float:
    """How much faster the outer wheel runs than the body at full turn rate."""
    omega_max_rad_s = math.radians(get(config, "rover.drive.omega_max_deg_s"))
    return omega_max_rad_s * get(config, "rover.track_width_mm") / 2.0


# -- the nine ---------------------------------------------------------------


def _runaway_on_command_timeout(config: dict[str, Any]) -> str | None:
    v_max = get(config, "rover.drive.v_max_mm_s")
    timeout_ms = get(config, "safety.command_timeout_ms")
    budget = get(config, "safety.runaway_budget_mm")

    distance = v_max * timeout_ms / 1000.0
    if distance <= budget:
        return None
    return (
        f"v_max ({_fmt(v_max)} mm/s) x command_timeout ({_fmt(timeout_ms)} ms) "
        f"= {_fmt(distance)} mm of travel after the link drops, which exceeds "
        f"runaway_budget ({_fmt(budget)} mm) "
        f"-> lower v_max_mm_s or command_timeout_ms, or raise runaway_budget_mm"
    )


def _runaway_on_row_loss(config: dict[str, Any]) -> str | None:
    v_max = get(config, "rover.drive.v_max_mm_s")
    frames = get(config, "safety.row_loss_frames")
    loop_hz = get(config, "perception.loop_hz")
    budget = get(config, "safety.runaway_budget_mm")

    distance = v_max * frames / loop_hz
    if distance <= budget:
        return None
    return (
        f"v_max ({_fmt(v_max)} mm/s) x row_loss_frames ({_fmt(frames)}) "
        f"/ loop_hz ({_fmt(loop_hz)} Hz) = {_fmt(distance)} mm of travel before the "
        f"watchdog trips, which exceeds runaway_budget ({_fmt(budget)} mm) "
        f"-> lower row_loss_frames, raise loop_hz, or lower v_max_mm_s"
    )


def _wheel_clears_soil(config: dict[str, Any]) -> str | None:
    diameter = get(config, "rover.wheel_diameter_mm")
    soil = get(config, "bed.soil_variation_mm")

    if diameter >= 4 * soil:
        return None
    return (
        f"wheel_diameter ({_fmt(diameter)} mm) must be at least "
        f"4 x soil_variation ({_fmt(soil)} mm) = {_fmt(4 * soil)} mm, or the wheels "
        f"climb the soil instead of rolling over it "
        f"-> raise wheel_diameter_mm in CAD, or lower soil_variation_mm"
    )


def _chassis_clears_soil(config: dict[str, Any]) -> str | None:
    clearance = get(config, "rover.chassis_clearance_mm")
    soil = get(config, "bed.soil_variation_mm")

    if clearance > soil:
        return None
    return (
        f"chassis_clearance ({_fmt(clearance)} mm) must exceed "
        f"soil_variation ({_fmt(soil)} mm), or the belly drags on every high spot "
        f"-> raise chassis_clearance_mm in CAD, or lower soil_variation_mm"
    )


def _furrow_fits_the_rover(config: dict[str, Any]) -> str | None:
    clear_furrow = _clear_furrow_mm(config)
    body_width = get(config, "rover.body_width_mm")
    budget = get(config, "safety.runaway_budget_mm")
    row_spacing = get(config, "bed.row_spacing_mm")
    foliage = get(config, "bed.crop_foliage_half_width_mm")

    needed = body_width + 2 * budget
    if clear_furrow > needed:
        return None
    return (
        f"clear_furrow ({_fmt(clear_furrow)} mm) must exceed "
        f"body_width ({_fmt(body_width)}) + 2 x runaway_budget ({_fmt(budget)}) "
        f"= {_fmt(needed)} mm, where clear_furrow = row_spacing ({_fmt(row_spacing)}) "
        f"- 2 x crop_foliage_half_width ({_fmt(foliage)}) "
        f"-> raise row_spacing_mm, or lower runaway_budget_mm"
    )


def _outer_wheel_within_motor_top_speed(config: dict[str, Any]) -> str | None:
    v_max = get(config, "rover.drive.v_max_mm_s")
    omega_max = get(config, "rover.drive.omega_max_deg_s")
    track = get(config, "rover.track_width_mm")
    wheel_v_max = get(config, "rover.drive.wheel_v_max_mm_s")

    outer = v_max + _wheel_differential_mm_s(config)
    if outer <= wheel_v_max:
        return None
    return (
        f"outer wheel at full turn: v_max ({_fmt(v_max)}) + omega_max ({_fmt(omega_max)} deg/s) "
        f"x track_width ({_fmt(track)}) / 2 = {_fmt(outer)} mm/s, which exceeds "
        f"wheel_v_max ({_fmt(wheel_v_max)} mm/s) "
        f"-> lower v_max_mm_s or omega_max_deg_s"
    )


def _inner_wheel_above_motor_deadband(config: dict[str, Any]) -> str | None:
    v_max = get(config, "rover.drive.v_max_mm_s")
    omega_max = get(config, "rover.drive.omega_max_deg_s")
    track = get(config, "rover.track_width_mm")
    wheel_v_min = get(config, "rover.drive.wheel_v_min_mm_s")

    inner = v_max - _wheel_differential_mm_s(config)
    if inner >= wheel_v_min:
        return None
    return (
        f"inner wheel at full turn: v_max ({_fmt(v_max)}) - omega_max ({_fmt(omega_max)} deg/s) "
        f"x track_width ({_fmt(track)}) / 2 = {_fmt(inner)} mm/s, which is below the "
        f"deadband wheel_v_min ({_fmt(wheel_v_min)} mm/s) -- the inner wheel stops "
        f"turning while the ESP32 still believes it is driving, and the rover turns "
        f"harder than commanded with nothing to signal it "
        f"-> lower omega_max_deg_s. Never raise wheel_v_min_mm_s to pass: it is a "
        f"property of the motor, measured at V3, not a knob"
    )


def _gains_are_tuned_for_the_active_backend(config: dict[str, Any]) -> str | None:
    backend = get(config, "backend")
    gains = get(config, f"row_follower.gains.{backend}", None)

    if gains is None:
        return (
            f"backend is {backend!r} but row_follower.gains.{backend} does not exist "
            f"-> add gains for that backend in config/control.yaml, or pick a backend "
            f"that has them"
        )

    k_lat, k_head = gains.get("k_lat"), gains.get("k_head")
    if k_lat is not None and k_head is not None:
        return None
    return (
        f"backend is {backend!r} but its row follower gains are not tuned "
        f"(k_lat={k_lat}, k_head={k_head}) -- skid-steer slip on soil is not slip in "
        f"sim, so the sim gains must not stand in "
        f"-> tune on that backend and write the pair into config/control.yaml "
        f"(row_follower.gains.{backend})"
    )


def _simulation_soil_matches_the_bed(config: dict[str, Any]) -> str | None:
    sim_soil = get(config, "simulation.soil.variation_mm")
    bed_soil = get(config, "bed.soil_variation_mm")

    if sim_soil == bed_soil:
        return None
    return (
        f"simulation.soil.variation_mm ({_fmt(sim_soil)}) must equal "
        f"bed.soil_variation_mm ({_fmt(bed_soil)}), or the simulator builds soil that "
        f"contradicts the geometry invariants while those invariants still report ok "
        f"-> make the two match"
    )


INVARIANTS: tuple[Invariant, ...] = (
    Invariant("runaway_on_command_timeout", "safety", _runaway_on_command_timeout),
    Invariant("runaway_on_row_loss", "safety", _runaway_on_row_loss),
    Invariant("wheel_clears_soil", "geometry", _wheel_clears_soil),
    Invariant("chassis_clears_soil", "geometry", _chassis_clears_soil),
    Invariant("furrow_fits_the_rover", "geometry", _furrow_fits_the_rover),
    Invariant("outer_wheel_within_motor_top_speed", "drive", _outer_wheel_within_motor_top_speed),
    Invariant("inner_wheel_above_motor_deadband", "drive", _inner_wheel_above_motor_deadband),
    Invariant("gains_are_tuned", "config", _gains_are_tuned_for_the_active_backend),
    Invariant("simulation_soil_matches_bed", "consistency", _simulation_soil_matches_the_bed),
)


def run(config: dict[str, Any]) -> None:
    """Check every invariant.  Raise ``ConfigInvalid`` if any fails.

    All nine run before anything is raised, so one startup shows everything
    that is wrong rather than one problem per attempt.
    """
    violations = [
        f"{invariant.scope:<11} {message}"
        for invariant in INVARIANTS
        if (message := invariant.check(config)) is not None
    ]

    if violations:
        raise ConfigInvalid("config_invalid:\n  " + "\n  ".join(violations))


def margins(config: dict[str, Any]) -> dict[str, float]:
    """How much room each numeric invariant has left, in its own units.

    A config can satisfy every invariant and still be a bad config: the reason
    ``track_width`` went from 140 to 120 was a clear-furrow margin of 4 mm, not
    a failure.  Margins make that visible instead of leaving it in a commit
    message.
    """
    return {
        "runaway_command_timeout": (
            get(config, "safety.runaway_budget_mm")
            - get(config, "rover.drive.v_max_mm_s")
            * get(config, "safety.command_timeout_ms")
            / 1000.0
        ),
        "runaway_row_loss": (
            get(config, "safety.runaway_budget_mm")
            - get(config, "rover.drive.v_max_mm_s")
            * get(config, "safety.row_loss_frames")
            / get(config, "perception.loop_hz")
        ),
        "wheel_diameter": (
            get(config, "rover.wheel_diameter_mm") - 4 * get(config, "bed.soil_variation_mm")
        ),
        "chassis_clearance": (
            get(config, "rover.chassis_clearance_mm") - get(config, "bed.soil_variation_mm")
        ),
        "clear_furrow": (
            _clear_furrow_mm(config)
            - get(config, "rover.body_width_mm")
            - 2 * get(config, "safety.runaway_budget_mm")
        ),
        "wheel_v_max": (
            get(config, "rover.drive.wheel_v_max_mm_s")
            - get(config, "rover.drive.v_max_mm_s")
            - _wheel_differential_mm_s(config)
        ),
        "wheel_v_min": (
            get(config, "rover.drive.v_max_mm_s")
            - _wheel_differential_mm_s(config)
            - get(config, "rover.drive.wheel_v_min_mm_s")
        ),
    }
