"""Test harness for the V0 closed loop.

Lives under tests/ and not under controller/ on purpose: FakeRowSensor reads
FakeRover's internal pose, and the pose must stay one import away from anything
that will ever run on the real rover.
"""

from tests.harness.fake_loop import FakeLoop, FakeRowSensor, LoopSample
from tests.harness.recording_rover import RecordingRover
from tests.harness.row import Row, straight_row

__all__ = [
    "FakeLoop",
    "FakeRowSensor",
    "LoopSample",
    "RecordingRover",
    "Row",
    "straight_row",
]
