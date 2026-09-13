"""Perception: two pipelines that share only ExG.

    camera.front -> row_estimator -> RowEstimate    control path
    camera.down  -> weed_detector -> Detection[]    observation path, V2

The two must not mix: a slow or missed detection must never make the rover
steer wrong, which is why the front camera runs at 10 Hz inside the control
loop and the down camera at 1-2 Hz outside it.

Nothing in here knows where the rover is.  The real machine has no
localisation, and the front camera is uncalibrated on purpose, so there is no
position for this layer to produce even if it wanted one —
``tests/unit/test_layering.py`` holds the whole directory to that.
"""

from perception.cameras import CameraSet, FakeCameraSet, Frame
from perception.exg import exg_index, green_mask, otsu_threshold
from perception.row_estimator import RowEstimator, Valley, find_valley

__all__ = [
    "CameraSet",
    "FakeCameraSet",
    "Frame",
    "RowEstimator",
    "Valley",
    "exg_index",
    "find_valley",
    "green_mask",
    "otsu_threshold",
]
