"""The Rover contract, held identically by every backend.

Two traps live here:

1. ``get_drive_state()`` must have no ``measured`` field.  There are no
   encoders, so nothing may report wheel speed as if it were observed.
2. ``FakeRover`` holds an internal pose, and that pose must never reach the
   interface.  Code written at V0 against a leaked pose breaks the day the
   backend becomes esp32 — in the field, where debugging is most expensive.
"""

import pytest

from controller.rover import Esp32Rover, FakeRover, IsaacRover, Rover

BACKENDS = (FakeRover, IsaacRover, Esp32Rover)


@pytest.fixture(params=BACKENDS, ids=lambda c: c.__name__)
def rover(request):
    return request.param()


def test_drive_state_shape_matches_across_backends():
    keys = {frozenset(r().get_drive_state()) for r in BACKENDS}
    assert len(keys) == 1
    assert keys.pop() == {"commanded", "estop", "link_age_ms"}


def test_commanded_shape_matches_across_backends():
    keys = {frozenset(r().get_drive_state()["commanded"]) for r in BACKENDS}
    assert len(keys) == 1
    assert keys.pop() == {"v_mm_s", "omega_deg_s"}


def test_no_backend_reports_anything_it_cannot_measure(rover):
    """No encoders exist, so no key may read like feedback.  The nesting under
    `commanded` is what tells a call site this is an echo of the command."""
    state = rover.get_drive_state()
    flat = {*state, *state["commanded"]}
    for forbidden in ("measured", "pose", "x_mm", "y_mm", "heading_deg", "odometry"):
        assert forbidden not in flat


def test_drive_state_types(rover):
    state = rover.get_drive_state()
    assert isinstance(state["commanded"]["v_mm_s"], float)
    assert isinstance(state["commanded"]["omega_deg_s"], float)
    assert isinstance(state["estop"], bool)
    assert isinstance(state["link_age_ms"], int)


def test_every_backend_satisfies_the_protocol(rover):
    assert isinstance(rover, Rover)


def test_constructing_a_backend_touches_no_device():
    """Isaac and the serial link are not available in tests/unit — construction
    must stay side-effect free or the shape test above cannot exist."""
    IsaacRover()
    Esp32Rover()


@pytest.mark.parametrize("backend", (IsaacRover, Esp32Rover))
def test_unimplemented_backends_say_so_instead_of_pretending(backend):
    with pytest.raises(NotImplementedError):
        backend().drive(100.0, 0.0)


class TestFakeRover:
    def test_drive_echoes_the_command(self):
        rover = FakeRover()
        rover.drive(100.0, -12.5)
        assert rover.get_drive_state()["commanded"] == {"v_mm_s": 100.0, "omega_deg_s": -12.5}

    def test_stop_zeroes_the_command(self):
        rover = FakeRover()
        rover.drive(100.0, 20.0)
        rover.stop()
        assert rover.get_drive_state()["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}

    def test_pose_is_reachable_for_the_harness_but_not_through_the_interface(self):
        """FakeRowSensor reads .pose — it is the test harness.  Nothing in
        workflow/, motion/ or safety/ may, and get_drive_state() is the fence."""
        rover = FakeRover()
        assert rover.pose.x_mm == 0.0
        assert "pose" not in rover.get_drive_state()
        assert not hasattr(Rover, "pose")

    def test_driving_straight_advances_x_only(self):
        rover = FakeRover()
        rover.drive(100.0, 0.0)
        rover.step(1.0)
        assert rover.pose.x_mm == pytest.approx(100.0)
        assert rover.pose.y_mm == pytest.approx(0.0)
        assert rover.pose.heading_deg == pytest.approx(0.0)

    def test_positive_omega_turns_left(self):
        """coordinate-frames.md: omega > 0 is CCW, and +Y is left."""
        rover = FakeRover()
        rover.drive(100.0, 40.0)
        rover.step(1.0)
        assert rover.pose.heading_deg == pytest.approx(40.0)
        assert rover.pose.y_mm > 0.0

    def test_emergency_stop_latches_and_drive_stops_taking_effect(self):
        rover = FakeRover()
        rover.emergency_stop()
        assert rover.get_drive_state()["estop"] is True

        rover.drive(100.0, 0.0)
        rover.step(1.0)
        assert rover.get_drive_state()["commanded"] == {"v_mm_s": 0.0, "omega_deg_s": 0.0}
        assert rover.pose.x_mm == pytest.approx(0.0)

    def test_a_fake_rover_has_no_link_to_age(self):
        assert FakeRover().get_drive_state()["link_age_ms"] == 0
