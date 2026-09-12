"""Entry point: bring up Isaac, drive the rover, report what it actually did.

    ~/isaac/python.bat sim/isaac/app.py --seconds 3

This is Stage 2's demonstrator, not the controller.  ``controller/main.py``
closes the perception loop at Stage 6; what runs here is a fixed list of drive
commands, so that "the wheels turn on command" has numbers attached to it
rather than an adjective.

The measured forward speed and yaw rate come from Isaac's ground truth, which
is legitimate HERE and nowhere else: this file is the simulator's own harness.
Nothing under ``controller/`` or ``perception/`` may read it - the rover has no
localization (hardware/mechanical/coordinate-frames.md), so code that does
works in simulation and fails in the field.
"""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser(description="Drive the rover in Isaac and measure it.")
parser.add_argument("--seconds", type=float, default=3.0, help="seconds per command")
parser.add_argument("--settle", type=float, default=0.5, help="seconds to settle before driving")
parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
parser.add_argument(
    "--command",
    action="append",
    metavar="V,OMEGA",
    help="a drive command as mm/s,deg/s; repeatable.  Defaults to the Stage 2 cases.",
)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import math  # noqa: E402
import sys  # noqa: E402
from pathlib import Path  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from controller.config import load_config  # noqa: E402
from controller.rover.isaac_rover import IsaacRover  # noqa: E402
from sim.isaac.world import (  # noqa: E402
    ROVER_BASE_PRIM_PATH,
    ROVER_PRIM_PATH,
    build_world,
)

#: Whether world.step() renders.  It also changes how much simulated time one
#: step covers, which is the reason step_seconds() below exists.
RENDER = not args.headless

#: The Stage 2 acceptance cases, in the order V1-ISAAC.md lists them.  The last
#: entry is stop(), which is why it carries a label rather than a command.
DEFAULT_CASES = (
    ("drive(100, 0)", 100.0, 0.0),
    ("drive(0, 40)", 0.0, 40.0),
    ("drive(320, 40)", 320.0, 40.0),
    ("stop()", None, None),
)


def parse_cases():
    if not args.command:
        return DEFAULT_CASES
    cases = []
    for text in args.command:
        v, omega = (float(part) for part in text.split(","))
        cases.append((f"drive({v:g}, {omega:g})", v, omega))
    return tuple(cases)


def step_seconds(world) -> float:
    """How much simulated time a single ``world.step()`` covers.

    NOT always the physics step.  ``step(render=True)`` advances by the
    RENDERING period, running as many physics substeps as that takes: at 60 Hz
    physics and 30 Hz render, 60 steps advance 1.0 s headless and 2.0 s with a
    window open.  Assuming physics_dt in both reports every speed in the GUI at
    exactly twice its real value - which is a very convincing bug, because the
    rover looks fine and only the numbers lie.
    """
    return world.get_rendering_dt() if RENDER else world.get_physics_dt()


def _yaw(quaternion) -> float:
    """Yaw from a (w, x, y, z) quaternion, the order Isaac returns."""
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def run_case(world, rover, body, label, v_mm_s, omega_deg_s):
    """Settle, command, run, and report what the body actually did."""
    world.reset()
    step_s = step_seconds(world)

    for _ in range(max(0, round(args.settle / step_s))):
        world.step(render=RENDER)

    if v_mm_s is None:
        rover.drive(100.0, 0.0)
        for _ in range(max(1, round(1.0 / step_s))):
            world.step(render=RENDER)
        rover.stop()
        sides = rover.wheel_velocities(0.0, 0.0)
    else:
        rover.drive(v_mm_s, omega_deg_s)
        sides = rover.wheel_velocities(v_mm_s, omega_deg_s)

    start_position, start_orientation = body.get_world_pose()
    start_time_s = world.current_time
    for _ in range(max(1, round(args.seconds / step_s))):
        world.step(render=RENDER)
    end_position, end_orientation = body.get_world_pose()

    # Taken from the clock rather than counted, so the number cannot be wrong
    # even if the relationship between a step and a second changes again.
    elapsed_s = world.current_time - start_time_s
    travelled_mm = 1000.0 * math.dist(start_position[:2], end_position[:2])
    yaw_deg = math.degrees(_yaw(end_orientation) - _yaw(start_orientation))
    yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0

    print(
        f"  {label:<16}"
        f"  target L={sides.left_rad_s:8.4f} R={sides.right_rad_s:8.4f} rad/s"
        f"  ->  v={travelled_mm / elapsed_s:8.1f} mm/s"
        f"  omega={yaw_deg / elapsed_s:7.2f} deg/s"
        f"  over {elapsed_s:.2f} s"
    )


def main() -> int:
    config = load_config()
    world = build_world(config)
    world.reset()

    from isaacsim.core.prims import SingleXFormPrim

    from sim.isaac.robots.rover.wheels import WheelJoints

    rover = IsaacRover.from_config(config, WheelJoints.from_stage(ROVER_PRIM_PATH))
    body = SingleXFormPrim(ROVER_BASE_PRIM_PATH)

    print(f"physics {1.0 / world.get_physics_dt():.0f} Hz, {args.seconds:g} s per case")
    for label, v_mm_s, omega_deg_s in parse_cases():
        run_case(world, rover, body, label, v_mm_s, omega_deg_s)
    return 0


if __name__ == "__main__":
    status = main()
    simulation_app.close()
    sys.exit(status)
