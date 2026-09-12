"""Closed-loop row following with no graphics, no physics and no images.

    FakeRover      integrate (v, omega) -> internal pose      (kinematic only)
    FakeRowSensor  pose + furrow equation -> RowEstimate
    RowRun         watchdog -> RowFollower -> state machine
                          +--> back into FakeRover

This answers "do these gains converge, and do they swing out of the furrow" in
milliseconds, in CI, before Isaac exists.  Both halves of that question matter:
gains that converge quickly but leave the furrow on the way are gains that
drive into a crop, and a test that reads only the final value passes them.

The harness plays the world and the camera.  Everything downstream of the
estimate -- the row-loss watchdog, the follower, the state machine -- is the
code that will run on the rover, so a run that ends in STOPPED(row_end_
suspected) ended there through the real transitions.

What the harness does NOT model, so that nothing built on it is mistaken for a
simulation result: wheel slip, motor deadband, soil, image noise, valley
detection, or any latency between seeing and steering.  ``green_fraction`` is
declared by the caller rather than counted, because there is nothing here to
count -- see GREEN_FRACTION_VALID below.  Which means the harness can stage
either ending of §5.4, and can prove neither that the configured threshold
separates them: that needs rendered crops, at V1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from controller.config import get, load_config
from controller.motion import RowEstimate, RowFollower
from controller.rover import FakeRover, Pose
from controller.safety import RowLossWatchdog
from controller.workflow import ErrorCode, Event, RoverState, RowRun, StateMachine
from tests.harness.row import Row

#: Front camera geometry, from cad/parameters/README.md.  Height 180 mm at a
#: 45 deg tilt puts the near valley one camera-height ahead.
CAMERA_HEIGHT_MM = 180.0
NEAR_LOOKAHEAD_MM = 180.0

#: The second sample row, further up the image.  heading_err is the difference
#: between the two, which is why it needs no calibration -- only two rows of
#: the same frame.
FAR_LOOKAHEAD_MM = 360.0

#: Horizontal field of view.  Not a config value: the front camera is
#: uncalibrated by design, so there is no measured number to load.  It is a
#: harness assumption, and it sets how many image-space units one mm of offset
#: is worth -- which is what the gains are tuned against.
HFOV_DEG = 60.0

#: What the harness *declares* green_fraction to be.  Not measured: there are
#: no pixels here, and nothing counts crops.  The caller states which of §5.4's
#: two worlds it is staging -- the green ran out (the end of the row) or the
#: green is still there and the line is not (a lost row) -- and the watchdog
#: reads that declaration exactly as it would read a real one.
GREEN_FRACTION_VALID = 0.4
GREEN_FRACTION_INVALID = 0.0


class FakeRowSensor:
    """Turn a rover pose into the RowEstimate a front camera would produce.

    Models one thing honestly: perspective.  The furrow is sampled at two
    forward distances and each sample is divided by the ground half-width the
    camera sees at *that* distance, so a far offset shrinks the way it does in
    a real image.  Getting this wrong in the flattering direction would inflate
    heading_err and make the gains look better than they are.
    """

    def __init__(
        self,
        row: Row,
        *,
        camera_height_mm: float = CAMERA_HEIGHT_MM,
        near_lookahead_mm: float = NEAR_LOOKAHEAD_MM,
        far_lookahead_mm: float = FAR_LOOKAHEAD_MM,
        hfov_deg: float = HFOV_DEG,
        green_fraction_valid: float = GREEN_FRACTION_VALID,
        green_fraction_invalid: float = GREEN_FRACTION_INVALID,
    ) -> None:
        self.row = row
        self.camera_height_mm = camera_height_mm
        self.near_lookahead_mm = near_lookahead_mm
        self.far_lookahead_mm = far_lookahead_mm
        self._tan_half_hfov = math.tan(math.radians(hfov_deg / 2.0))
        self.green_fraction_valid = green_fraction_valid
        self.green_fraction_invalid = green_fraction_invalid

    def estimate(self, pose: Pose) -> RowEstimate:
        near = self._image_offset(pose, self.near_lookahead_mm)
        far = self._image_offset(pose, self.far_lookahead_mm)

        if near is None or far is None or abs(near) > 1.0 or abs(far) > 1.0:
            # The furrow has left the frame -- it ran out, or the rover is
            # pointing off it.  In the real estimator this is one of two very
            # different things, told apart by green_fraction (§5.4).  The
            # harness has no crops to count, so it reports the value the caller
            # declared rather than inventing one from geometry.
            return RowEstimate(
                valid=False,
                lateral_err=0.0,
                heading_err=0.0,
                confidence=0.0,
                green_fraction=self.green_fraction_invalid,
            )

        return RowEstimate(
            valid=True,
            lateral_err=near,
            heading_err=max(-1.0, min(1.0, far - near)),
            # Constants: the harness has no valley to score and no pixels to
            # count.  They are here so the struct is complete, not to be read.
            confidence=1.0,
            green_fraction=self.green_fraction_valid,
        )

    def _image_offset(self, pose: Pose, lookahead_mm: float) -> float | None:
        """Where the furrow sits in the image, at one forward distance.

        Returns a ratio in image space: the furrow's lateral distance from the
        rover's axis, divided by the ground half-width visible at that range.
        Positive means the furrow is to the right, matching RowEstimate.
        """
        lateral_mm = self._furrow_lateral_offset_mm(pose, lookahead_mm)
        if lateral_mm is None:
            return None

        # A pinhole camera: a lateral offset shrinks with range, not with
        # forward distance, so the range includes the camera's height above
        # the plane.
        range_mm = math.hypot(self.camera_height_mm, lookahead_mm)
        half_width_mm = range_mm * self._tan_half_hfov
        return -lateral_mm / half_width_mm

    def _furrow_lateral_offset_mm(self, pose: Pose, lookahead_mm: float) -> float | None:
        """Lateral offset (rover frame, + = left) of the furrow point lying
        ``lookahead_mm`` ahead of the rover.

        Solved numerically so a curved furrow needs no new code: bisect on bed
        X for the point whose forward coordinate in the rover frame is the
        lookahead distance.
        """
        heading_rad = math.radians(pose.heading_deg)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)

        def forward_of(x_mm: float) -> float:
            dx = x_mm - pose.x_mm
            dy = self.row.centre_offset_mm(x_mm) - pose.y_mm
            return dx * cos_h + dy * sin_h

        low = pose.x_mm
        # Never sample past the end of the furrow: beyond it there is nothing
        # to find, and the bracket check below turns that into the invalid
        # estimate the watchdog counts.
        high = self.row.furthest_x_mm(pose.x_mm + 4.0 * lookahead_mm)
        if forward_of(low) > lookahead_mm or forward_of(high) < lookahead_mm:
            # The furrow does not reach that far ahead of where the rover is
            # pointing; there is nothing to sample.
            return None

        for _ in range(60):
            mid = 0.5 * (low + high)
            if forward_of(mid) < lookahead_mm:
                low = mid
            else:
                high = mid

        x_mm = 0.5 * (low + high)
        dx = x_mm - pose.x_mm
        dy = self.row.centre_offset_mm(x_mm) - pose.y_mm
        return -dx * sin_h + dy * cos_h


@dataclass(frozen=True)
class LoopSample:
    """State at ``t_s``, and the command that was applied to reach it.

    Sample 0 is the starting state, with a zero command.
    """

    t_s: float
    lateral_err: float
    heading_err: float
    valid: bool
    v_mm_s: float
    omega_deg_s: float
    offset_mm: float
    heading_deg: float
    clearance_mm: float


class FakeLoop:
    """Run the control loop against FakeRover at the configured rate."""

    def __init__(
        self,
        row: Row,
        start_offset_mm: float = 0.0,
        start_heading_deg: float = 0.0,
        config: dict | None = None,
        backend: str = "fake",
        sensor: FakeRowSensor | None = None,
    ) -> None:
        """``start_offset_mm`` and ``start_heading_deg`` follow the rover frame:
        +Y is left of the furrow and +heading is CCW.  So ``(+60, +15)`` starts
        the rover to the left of the furrow and pointing further left -- it has
        to stop diverging before it can begin converging, which is the case
        worth testing.
        """
        self.config = config if config is not None else load_config()
        self.row = row
        self.sensor = sensor if sensor is not None else FakeRowSensor(row)
        self.follower = RowFollower.from_config(self.config, backend=backend)

        self.rover = FakeRover(
            x_mm=0.0,
            y_mm=row.centre_offset_mm(0.0) + start_offset_mm,
            heading_deg=start_heading_deg,
        )

        # The real workflow, not a stand-in for it: the harness plays the world
        # and the camera, and everything downstream of the estimate is the code
        # that will run on the rover.  A harness that stopped the rover its own
        # way would be a harness testing itself.
        self.machine = StateMachine()
        self.row_run = RowRun(
            machine=self.machine,
            rover=self.rover,
            follower=self.follower,
            watchdog=RowLossWatchdog.from_config(self.rover, self.config),
        )
        self.machine.fire(Event.VALIDATE_OK)
        self.row_run.start()

        self.loop_hz = float(get(self.config, "perception.loop_hz"))
        self._dt_s = 1.0 / self.loop_hz

        # Crop foliage, not row spacing, is what the rover can actually hit.
        row_spacing_mm = float(get(self.config, "bed.row_spacing_mm"))
        foliage_mm = float(get(self.config, "bed.crop_foliage_half_width_mm"))
        self._half_clear_furrow_mm = (row_spacing_mm - 2.0 * foliage_mm) / 2.0
        self._half_body_mm = float(get(self.config, "rover.body_width_mm")) / 2.0

        self._t_s = 0.0
        self.history: list[LoopSample] = []

    def run(self, seconds: float) -> FakeLoop:
        """Advance the loop.  May be called again to continue where it stopped.

        Stops early once the run leaves DRIVING_ROW -- the row ended, or the
        watchdog tripped -- because that is where §5.5's ``while`` ends.
        """
        if not self.history:
            self.history.append(self._sample(self.sensor.estimate(self.rover.pose), 0.0, 0.0))

        for _ in range(round(seconds * self.loop_hz)):
            if self.state is not RoverState.DRIVING_ROW:
                break

            self.row_run.step(self.sensor.estimate(self.rover.pose))

            # Read back what was commanded rather than what was computed: an
            # invalid frame commands nothing at all, and the standing command
            # is what the rover actually carries into the next step.
            commanded = self.rover.get_drive_state()["commanded"]
            self.rover.step(self._dt_s)
            self._t_s += self._dt_s
            self.history.append(
                self._sample(
                    self.sensor.estimate(self.rover.pose),
                    commanded["v_mm_s"],
                    commanded["omega_deg_s"],
                )
            )

        return self

    @property
    def state(self) -> RoverState:
        return self.machine.state

    @property
    def stop_reason(self) -> str | None:
        return self.machine.stop_reason

    @property
    def error_code(self) -> ErrorCode | None:
        return self.machine.error_code

    @property
    def distance_travelled_mm(self) -> float:
        """How far along the bed the rover got.

        Along bed X, not arc length: the question a scenario asks is how much
        of the row was covered, and the rover has no odometry to answer it with
        anyway -- this is the harness reading the world it plays.
        """
        return self.rover.pose.x_mm  # the rover always starts at bed X = 0

    @property
    def min_clearance_mm(self) -> float:
        """Smallest gap between the rover's widest point and the crop foliage.

        Measured the way invariant 5 measures it: body width, not a swept
        rectangle.  A rover at an angle sweeps wider than that, and the margin
        for it is runaway_budget_mm, which invariant 5 already reserves.

        Sampled at loop rate.  At 100 mm/s and 10 Hz the rover moves 10 mm
        between samples, so this is an estimate of the true minimum, not a bound.
        """
        return min(sample.clearance_mm for sample in self.history)

    @property
    def max_lateral_offset_mm(self) -> float:
        return max(abs(sample.offset_mm) for sample in self.history)

    def _sample(self, est: RowEstimate, v_mm_s: float, omega_deg_s: float) -> LoopSample:
        pose = self.rover.pose
        offset_mm = pose.y_mm - self.row.centre_offset_mm(pose.x_mm)
        return LoopSample(
            t_s=round(self._t_s, 6),
            lateral_err=est.lateral_err,
            heading_err=est.heading_err,
            valid=est.valid,
            v_mm_s=v_mm_s,
            omega_deg_s=omega_deg_s,
            offset_mm=offset_mm,
            heading_deg=pose.heading_deg,
            clearance_mm=self._half_clear_furrow_mm - abs(offset_mm) - self._half_body_mm,
        )
