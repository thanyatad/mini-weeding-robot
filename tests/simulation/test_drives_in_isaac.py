"""Stage 2 acceptance, measured in Isaac.

Skips cleanly wherever Isaac is not installed, which is everywhere except a
machine with a GPU and a working Isaac Sim.  ``pytest -q`` at the repo root
stays fast and GPU-free; run this one deliberately:

    ~/isaac/python.bat -m pytest tests/simulation -q

The numbers checked here are loose on purpose.  This is not physics tuning -
that is Stage 3, and the mass the friction is tuned against is itself
incomplete.  What these assert is that the sign, the scale and the saturation
behaviour are right, which are the three things that are cheap to get wrong and
expensive to notice later.

Ground truth is read here because this is the simulator's own harness.  Nothing
under controller/ or perception/ may do the same - see
tests/unit/test_coordinate_frames.py.
"""

import math

import pytest

pytest.importorskip("isaacsim", reason="Isaac Sim is not installed in this interpreter")

pytestmark = pytest.mark.isaac

SETTLE_S = 0.5
RUN_S = 2.0


@pytest.fixture(scope="module")
def sim():
    """The world, the rover and the body to measure.

    The SimulationApp itself belongs to conftest.py, which closes it after the
    reporter has run - see the note there about fastShutdown.
    """
    from conftest import isaac_app

    isaac_app()

    from isaacsim.core.prims import SingleXFormPrim

    from controller.config import load_config
    from controller.rover.isaac_rover import IsaacRover
    from sim.isaac.robots.rover.wheels import WheelJoints
    from sim.isaac.world import ROVER_BASE_PRIM_PATH, ROVER_PRIM_PATH, build_world

    config = load_config()
    world = build_world(config)
    world.reset()

    rover = IsaacRover.from_config(config, WheelJoints.from_stage(ROVER_PRIM_PATH))
    body = SingleXFormPrim(ROVER_BASE_PRIM_PATH)

    return world, rover, body


def _yaw(quaternion) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def run(sim, v_mm_s, omega_deg_s, *, seconds=RUN_S):
    """Settle, command, run, and report displacement and yaw rate."""
    world, rover, body = sim
    world.reset()
    dt_s = world.get_physics_dt()

    for _ in range(round(SETTLE_S / dt_s)):
        world.step(render=False)

    if v_mm_s is None:
        rover.stop()
    else:
        rover.drive(v_mm_s, omega_deg_s)

    start_position, start_orientation = body.get_world_pose()
    start_time_s = world.current_time
    for _ in range(round(seconds / dt_s)):
        world.step(render=False)
    end_position, end_orientation = body.get_world_pose()

    # From the clock, not counted.  These tests step with render=False, where a
    # step is one physics step - but step(render=True) advances by the RENDERING
    # period instead, and counting would then report every speed here at twice
    # its real value.  sim/isaac/app.py hit exactly that.
    elapsed_s = world.current_time - start_time_s
    yaw_deg = math.degrees(_yaw(end_orientation) - _yaw(start_orientation))
    yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0

    return {
        "forward_mm_s": 1000.0 * (end_position[0] - start_position[0]) / elapsed_s,
        "lateral_mm_s": 1000.0 * (end_position[1] - start_position[1]) / elapsed_s,
        "speed_mm_s": 1000.0 * math.dist(start_position[:2], end_position[:2]) / elapsed_s,
        "omega_deg_s": yaw_deg / elapsed_s,
    }


def test_drive_100_0_drives_straight_at_about_100_mm_s(sim):
    result = run(sim, 100.0, 0.0)
    assert result["forward_mm_s"] == pytest.approx(100.0, rel=0.10)
    assert abs(result["omega_deg_s"]) < 2.0


def test_drive_0_40_turns_on_the_spot(sim):
    """On the spot means it yaws in the commanded direction without travelling.

    The threshold is 5 deg/s against a commanded 40, well under the 15.9
    measured, because how MUCH of the commanded yaw arrives is the xfail
    below - this one only asks that the rover turns the way it was told and
    stays put while doing it.  It read 0.5 while the drive was too weak to
    break the wheels loose at all; a bar that low would pass a rover that
    barely twitches.
    """
    result = run(sim, 0.0, 40.0)
    assert result["omega_deg_s"] > 5.0, "commanded a left turn and it did not yaw left"
    assert result["speed_mm_s"] < 20.0, "turning on the spot should not travel"


@pytest.mark.xfail(
    strict=False,
    reason="Stage 3: skid-steer yaw reaches about 40% of nominal on flat ground - "
    "15.9 deg/s measured against a commanded 40.  Not a sign or unit fault, and no "
    "longer a torque fault either: the wheels hold their commanded speed to within "
    "1% and the drive peaks at ~1.1 of the 10 N.m it is allowed, so what is left is "
    "the contact model.  Friction is not the knob - that was tested on the previous "
    "rover and lowering it 45x moved the yaw barely at all.  See "
    "sim/isaac/robots/rover/config.yaml.",
)
def test_drive_0_40_should_reach_a_usable_fraction_of_the_commanded_yaw(sim):
    """The gap between what the mixing commands and what the contact delivers.

    Slip means the achieved omega is never the commanded one - the row follower
    closes its loop on the image precisely because of that.  40% is slip and a
    rover that steers; the few percent this used to read was a rover that could
    not steer at all, and it turned out to be the drive rather than the contact:
    damping was set for a 1 kg rover, so the drive saturated at 0.34 N.m against
    the ~0.92 N.m needed to scrub 35 kg round.  What remains is the contact
    model.  It is xfail rather than deleted so that the day Stage 3 closes the
    rest of the gap, this turns green and says so.
    """
    assert run(sim, 0.0, 40.0)["omega_deg_s"] > 20.0


def test_a_negative_omega_turns_the_other_way(sim):
    """The sign, checked against the simulator rather than against intuition.
    A flipped omega makes the row follower's feedback positive, and the rover
    leaves the furrow faster than it would with no steering at all."""
    left = run(sim, 0.0, 40.0)["omega_deg_s"]
    right = run(sim, 0.0, -40.0)["omega_deg_s"]
    assert left > 0.0 > right


def test_drive_320_40_saturates_without_losing_the_turn(sim):
    """Both sides scale, so the rover still turns - slower, and in the same
    direction.  Clipping one side instead would change the turn silently."""
    nominal = run(sim, 100.0, 40.0)
    saturated = run(sim, 320.0, 40.0)

    assert saturated["forward_mm_s"] > nominal["forward_mm_s"]
    assert saturated["omega_deg_s"] > 0.0, "saturation must not cancel the turn"


def test_stop_holds_the_rover_still(sim):
    result = run(sim, None, None)
    assert result["speed_mm_s"] < 5.0


def test_stop_holds_the_brake_rather_than_coasting(sim):
    """A coasting rover carries its speed for a while; a braked one does not."""
    world, rover, body = sim
    world.reset()
    dt_s = world.get_physics_dt()

    for _ in range(round(SETTLE_S / dt_s)):
        world.step(render=False)
    rover.drive(100.0, 0.0)
    for _ in range(round(1.5 / dt_s)):
        world.step(render=False)

    rover.stop()
    start_position, _ = body.get_world_pose()
    for _ in range(round(0.5 / dt_s)):
        world.step(render=False)
    end_position, _ = body.get_world_pose()

    coast_mm = 1000.0 * math.dist(start_position[:2], end_position[:2])
    assert coast_mm < 10.0, f"rolled {coast_mm:.1f} mm after stop() - the brake is not holding"
