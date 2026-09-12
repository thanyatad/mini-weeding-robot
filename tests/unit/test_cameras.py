"""The camera boundary: two named accessors, and a time that travels with the
pixels.

Why a ``Frame`` and not a bare array.  ``perception.camera_timeout_ms`` is 300
ms, which means somewhere something has to know how old a frame is.  Every
camera stack that has a real sensor behind it carries that with the frame --
ROS 2 puts an acquisition stamp in ``sensor_msgs/Image``'s header, librealsense
hangs ``get_timestamp()`` off the frame object.  The one stack that does not is
OpenCV's ``VideoCapture``, where the time is a separate query
(``CAP_PROP_POS_MSEC``) that is documented to return -1 on a live camera.  A
bare array is that design, and it is the one with the known dead end.

Why the estimator still takes an array.  ROS makes the same split: the message
carries the header, and ``cv_bridge`` hands the algorithm a plain array.  The
row estimator needs pixels and has no use for a clock, so asking it for a
``Frame`` would put a field in its signature that it must not read.
"""

import time

import numpy as np
import pytest

from perception.cameras import CameraSet, FakeCameraSet, Frame

SHAPE = (240, 320)  # perception.row_estimator.front_downscale, (height, width)


def an_image(fill: int = 1) -> np.ndarray:
    return np.full((*SHAPE, 3), fill, dtype=np.uint8)


class FakeClock:
    """A clock that only moves when the test moves it."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now


class TestFrame:
    def test_it_carries_the_image_and_when_it_was_taken(self):
        image = an_image()
        frame = Frame(image=image, timestamp_s=12.5)

        assert frame.image is image
        assert frame.timestamp_s == 12.5

    def test_it_is_frozen(self):
        """A frame is a reading, not a variable.  Rewriting the timestamp on one
        would make a stale frame look fresh, which is the exact check the
        timestamp exists for."""
        frame = Frame(image=an_image(), timestamp_s=1.0)
        with pytest.raises(Exception):
            frame.timestamp_s = 2.0  # type: ignore[misc]

    def test_two_frames_can_be_compared_without_raising(self):
        """A dataclass holding an ndarray gets a generated ``__eq__`` that
        compares the arrays elementwise and then asks a whole array for its
        truth value.  That raises, and it raises from ``==``, which is the last
        place anyone looks."""
        a = Frame(image=an_image(), timestamp_s=1.0)
        b = Frame(image=an_image(), timestamp_s=1.0)

        assert (a == b) in (True, False)


class TestCameraSetProtocol:
    def test_the_fake_satisfies_the_protocol(self):
        cameras = FakeCameraSet(front=an_image(), down=an_image())
        assert isinstance(cameras, CameraSet)

    def test_the_accessors_are_named_rather_than_indexed(self):
        """``capture(camera_id)`` would hide that these two cameras differ in
        contract and not in index -- front is uncalibrated and drives at 10 Hz,
        down is homographed to mm and runs at 1-2 Hz.  Hidden, somebody
        eventually puts a front frame through a coordinate transform."""
        cameras = FakeCameraSet(front=an_image(1), down=an_image(2))

        assert cameras.front().image[0, 0, 0] == 1
        assert cameras.down().image[0, 0, 0] == 2


class TestFakeCameraSet:
    def test_a_still_image_is_returned_on_every_call(self):
        cameras = FakeCameraSet(front=an_image(7))

        assert cameras.front().image[0, 0, 0] == 7
        assert cameras.front().image[0, 0, 0] == 7

    def test_a_scripted_sequence_is_consumed_in_order(self):
        cameras = FakeCameraSet(front=[an_image(1), an_image(2), an_image(3)])

        assert [cameras.front().image[0, 0, 0] for _ in range(3)] == [1, 2, 3]

    def test_an_exhausted_script_raises_rather_than_repeating(self):
        """Silently re-serving the last frame would let a test that thinks it
        drove ten frames actually drive two, and pass."""
        cameras = FakeCameraSet(front=[an_image(1)])
        cameras.front()

        with pytest.raises(IndexError, match="front"):
            cameras.front()

    def test_asking_for_a_camera_that_was_not_supplied_says_so(self):
        cameras = FakeCameraSet(front=an_image())

        with pytest.raises(LookupError, match="down"):
            cameras.down()

    def test_the_timestamp_comes_from_the_clock(self):
        clock = FakeClock()
        cameras = FakeCameraSet(front=an_image(), clock=clock)

        clock.now = 4.0
        assert cameras.front().timestamp_s == 4.0
        clock.now = 4.1
        assert cameras.front().timestamp_s == 4.1

    def test_the_default_clock_is_monotonic_and_not_the_wall_clock(self):
        """``camera_timeout_ms`` is a latency measurement, and a wall clock can
        step backwards when NTP corrects it -- which would either fire the
        staleness check on a fresh frame or hide a stale one.  Epoch seconds are
        past 1.7e9; a monotonic clock counts from boot and is nowhere near it."""
        cameras = FakeCameraSet(front=an_image())

        assert cameras.front().timestamp_s < 1e9

    def test_successive_frames_do_not_go_backwards_in_time(self):
        cameras = FakeCameraSet(front=[an_image(), an_image()])

        first, second = cameras.front(), cameras.front()
        assert second.timestamp_s >= first.timestamp_s

    def test_the_clock_is_read_when_the_frame_is_taken(self):
        """Not at construction.  A set built once and read for a whole run would
        otherwise hand out one age forever, and every staleness check downstream
        would compare against it."""
        clock = FakeClock(start=1.0)
        cameras = FakeCameraSet(front=[an_image(), an_image()], clock=clock)

        clock.now = 100.0
        assert cameras.front().timestamp_s == 100.0

    def test_the_real_clock_is_the_default(self):
        cameras = FakeCameraSet(front=an_image())
        before = time.monotonic()
        taken = cameras.front().timestamp_s
        after = time.monotonic()

        assert before <= taken <= after
