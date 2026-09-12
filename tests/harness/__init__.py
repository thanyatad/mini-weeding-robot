"""Test harness for the V0 closed loop, and for the link that carries it.

Lives under tests/ and not under controller/ on purpose: FakeRowSensor reads
FakeRover's internal pose, and the pose must stay one import away from anything
that will ever run on the real rover.  ``Bench`` is here for the same reason and
one more: two callers need it, the integration tests and the scenario runner.
"""

from tests.harness.bench import BED_LENGTH_MM, Bench, CutWire
from tests.harness.fake_loop import FakeLoop, FakeRowSensor, LoopSample
from tests.harness.recording_rover import RecordingRover
from tests.harness.row import Row, straight_row

__all__ = [
    "BED_LENGTH_MM",
    "Bench",
    "CutWire",
    "FakeLoop",
    "FakeRowSensor",
    "LoopSample",
    "RecordingRover",
    "Row",
    "straight_row",
]
