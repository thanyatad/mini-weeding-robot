"""IsaacRover against a recording stand-in for the wheel joints.

No Isaac, no GPU, no stage.  What is checked here is the part that is the
controller's business: that a command reaches the wheels once, in the right
units, with the right sign, and that a latched E-stop stops it from getting
there at all.  Whether PhysX then turns the wheels is Isaac's business and is
measured by sim/isaac/app.py.

tests/unit/test_rover_backends.py already holds this class to the shape of
``get_drive_state()``; none of that is repeated here.
"""

import pytest

from controller.config import load_config
from controller.rover.isaac_rover import IsaacRover
from sim.isaac.adapters.wheel_adapter import WheelAdapter


class RecordingJoints:
    """Satisfies ``WheelJointHandles`` and writes down what it was told."""

    def __init__(self):
        self.velocities = []
        self.holds = 0
        self.releases = 0

    def set_side_velocities(self, left_rad_s, right_rad_s):
        self.velocities.append((left_rad_s, right_rad_s))

    def hold(self):
        self.holds += 1

    def release(self):
        self.releases += 1


def build():
    joints = RecordingJoints()
    rover = IsaacRover.from_config(load_config(), joints)
    return rover, joints


# -- construction ----------------------------------------------------------


def test_a_rover_with_no_joints_says_so_rather_than_pretending():
    """Construction must stay side-effect free so that the cross-backend shape
    test in test_rover_backends.py can build one without a simulator."""
    with pytest.raises(NotImplementedError):
        IsaacRover().drive(100.0, 0.0)


def test_from_config_wires_up_an_adapter_built_from_config():
    rover, _ = build()
    assert isinstance(rover.adapter, WheelAdapter)
    assert rover.adapter.track_width_mm == 430.0


# -- driving ---------------------------------------------------------------


def test_driving_forward_sends_one_command_to_both_sides():
    rover, joints = build()
    rover.drive(100.0, 0.0)

    assert len(joints.velocities) == 1
    left, right = joints.velocities[0]
    assert left == pytest.approx(100.0 / 125.0)
    assert right == pytest.approx(left)


def test_the_drive_state_echoes_what_was_commanded_not_what_reached_the_wheels():
    rover, _ = build()
    rover.drive(100.0, -12.5)
    assert rover.get_drive_state()["commanded"] == {"v_mm_s": 100.0, "omega_deg_s": -12.5}


def test_a_positive_omega_reaches_the_wheels_as_a_faster_right_side():
    """The sign check that matters: coordinate-frames.md says omega > 0 is CCW
    with Z up, which is a left turn, which is v_right > v_left.  Flipped, the
    row follower's feedback turns positive and the rover leaves the furrow
    faster than it would with no steering at all."""
    rover, joints = build()
    rover.drive(100.0, 40.0)
    left, right = joints.velocities[0]
    assert right > left


def test_saturation_is_the_adapters_and_reaches_the_wheels_scaled():
    rover, joints = build()
    rover.drive(320.0, 40.0)
    left, right = joints.velocities[0]
    assert max(abs(left), abs(right)) == pytest.approx(327.0 / 125.0)
    assert left > 0.0


# -- stopping --------------------------------------------------------------


def test_stop_holds_the_brake_rather_than_commanding_zero_velocity():
    """``Rover.stop()`` is a controlled stop with the brake held, which is a
    different thing from drive(0, 0) coasting.  The brake is the drive's own
    damping, so the distinction is that the drive stays engaged."""
    rover, joints = build()
    rover.drive(100.0, 0.0)
    rover.stop()

    assert joints.holds == 1
    assert joints.releases == 0
    assert rover.get_drive_state()["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}


def test_emergency_stop_cuts_the_motor_rail_instead_of_braking():
    """The real E-stop opens the motor rail electrically, so the motors make no
    torque at all.  Braking would be a gentler thing than what the button
    does, and the simulation would be easier than the rover."""
    rover, joints = build()
    rover.emergency_stop()

    assert joints.releases == 1
    assert joints.holds == 0
    assert rover.get_drive_state()["estop"] is True


def test_a_latched_estop_keeps_later_commands_off_the_wheels():
    """Cleared by hand, never by software.  A drive() that still reached the
    joints would make the simulation safer than the rover."""
    rover, joints = build()
    rover.emergency_stop()
    rover.drive(100.0, 0.0)

    assert joints.velocities == []
    assert rover.get_drive_state()["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}


def test_stop_after_an_estop_does_not_re_engage_the_drive():
    rover, joints = build()
    rover.emergency_stop()
    rover.stop()
    assert joints.holds == 0


# -- what must not leak ----------------------------------------------------


def test_the_rover_offers_no_pose():
    """Isaac knows where the rover is.  The controller must never be able to
    ask, because nothing on the hardware can answer."""
    rover, _ = build()
    for forbidden in ("pose", "get_world_pose", "position", "x_mm", "heading_deg"):
        assert not hasattr(rover, forbidden)


def test_wheel_velocities_are_readable_without_commanding_them():
    """sim/isaac/app.py prints the target beside the measured result.  Asking
    for the number must not move the rover."""
    rover, joints = build()
    sides = rover.wheel_velocities(100.0, 0.0)
    assert sides.left_rad_s == pytest.approx(100.0 / 125.0)
    assert joints.velocities == []
