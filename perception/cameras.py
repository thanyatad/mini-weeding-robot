"""The camera boundary -- two named accessors, and a frame that knows its own age.

    CameraSet
    |
    +-- FakeCameraSet      here, V1: scripted arrays, no hardware, no Isaac
    +-- IsaacCameraSet     Stage 5
    +-- UsbCameraSet       Stage 5

Named accessors rather than ``capture(camera_id)``, because the two cameras
differ in *contract* and not in index (perception/README.md):

======================  =====================  =========================
                        ``front()``            ``down()``
======================  =====================  =========================
drives                  row following          weed detection
calibration             none, deliberately     homography to mm
rate                    10 Hz, in the loop     1-2 Hz, off the loop
======================  =====================  =========================

Behind one indexed call those differences disappear, and sooner or later
somebody puts an uncalibrated front frame through a coordinate transform.

Why a ``Frame`` and not a bare array
------------------------------------
``perception.camera_timeout_ms`` is 300 ms, so something has to be able to ask
how old a frame is.  Every camera stack with a real sensor behind it answers
that from the frame itself -- ROS 2 stamps ``sensor_msgs/Image``'s header with
the acquisition time, librealsense hangs ``get_timestamp()`` off the frame
object.  The exception is OpenCV's ``VideoCapture``, which returns a bare array
and makes the time a separate query, ``CAP_PROP_POS_MSEC``, documented to
return -1 on a live camera.  A bare array here would be that design.

The clock is monotonic, not the wall clock.  The only question asked of the
timestamp is a duration -- "older than 300 ms?" -- and a wall clock steps
backwards when NTP corrects it, which would fire the check on a fresh frame or
hide a stale one.

The estimator still takes an array, not a ``Frame``.  ROS makes the same split:
the message carries the header, ``cv_bridge`` hands the algorithm a plain
array.  ``row_estimator.estimate()`` needs pixels and has no use for a clock,
and a field in its signature that it must not read is an invitation.

Nothing in this module checks the timeout.  ``Event.CAMERA_TIMEOUT`` is in
``row_run.FAULT_EVENTS``, which is the set detected *outside* the control loop;
this module's job is to make the age knowable, not to judge it.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

#: What a fake camera can be fed: one still image, or a script of them.
Source = np.ndarray | Sequence[np.ndarray]


@dataclass(frozen=True, eq=False)
class Frame:
    """One image and the moment it was taken.

    ``image``        ``(h, w, 3)`` RGB, as ``perception.exg`` expects
    ``timestamp_s``  seconds from a **monotonic** clock, not epoch seconds

    ``eq=False`` on purpose.  The generated ``__eq__`` would compare the two
    images elementwise and then ask an array for its truth value, which raises
    -- out of ``==``, which is the last place anyone thinks to look.  Frames are
    readings; identity is the comparison that means anything about them.
    """

    image: np.ndarray
    timestamp_s: float


@runtime_checkable
class CameraSet(Protocol):
    """The two cameras, addressed by what they are for."""

    def front(self) -> Frame:
        """The forward camera, for row following.  Uncalibrated."""
        ...

    def down(self) -> Frame:
        """The downward camera, for weed detection.  Homographed at V2."""
        ...


class _Feed:
    """One camera's supply of images: a still, or a script that runs out."""

    def __init__(self, name: str, source: Source | None) -> None:
        self._name = name
        self._still: np.ndarray | None = None
        self._script: list[np.ndarray] | None = None
        self._taken = 0

        if source is None:
            return
        if isinstance(source, np.ndarray):
            self._still = source
        else:
            self._script = list(source)

    def next_image(self) -> np.ndarray:
        if self._still is not None:
            return self._still

        if self._script is None:
            raise LookupError(
                f"this FakeCameraSet has no {self._name} camera — "
                f"pass {self._name}= to give it one"
            )

        if self._taken >= len(self._script):
            raise IndexError(
                f"the {self._name} script is exhausted after "
                f"{len(self._script)} frame(s)"
            )

        image = self._script[self._taken]
        self._taken += 1
        return image


class FakeCameraSet:
    """A :class:`CameraSet` fed from arrays, for tests and for V1 perception.

    It lives here rather than under ``tests/`` because it is a *backend*, the
    same kind of thing as ``IsaacCameraSet``, and perception/README.md lists it
    beside them.  It reads nothing about where the rover is, so nothing here
    can leak into the control path the way ``FakeRover``'s internals would.

    A still image is served for as long as it is asked for; a script is served
    once through and then **raises**.  Re-serving the last frame forever would
    let a test that believes it drove ten frames drive two, and pass.
    """

    def __init__(
        self,
        front: Source | None = None,
        down: Source | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._front = _Feed("front", front)
        self._down = _Feed("down", down)
        self._clock = clock

    def front(self) -> Frame:
        return self._take(self._front)

    def down(self) -> Frame:
        return self._take(self._down)

    def _take(self, feed: _Feed) -> Frame:
        """Read the clock when the frame is taken, never at construction.

        A set built once and read for a whole run would otherwise hand out one
        age forever, and every staleness check downstream would compare against
        the moment the test started.
        """
        image = feed.next_image()
        return Frame(image=image, timestamp_s=float(self._clock()))
