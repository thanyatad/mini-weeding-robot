"""Safety: the controller's own layer of the three in spec §5.6.

Only the middle layer lives here.  The ESP32's command timeout and the latching
E-stop are the other two, and each of the three has to work without the one
above it — a layer that leans on the layer above is not a safety layer, it is a
feature.

Three detectors, one router, one owner of what a fault means.  ``runaway``
counts estimates, ``link_monitor`` watches the link age and ``emergency_stop``
watches the sense line; ``fault_manager`` translates what the board reports.
All four hand the verdict to the workflow, and none of them decides what
happens next.
"""

from controller.safety.emergency_stop import EmergencyStop
from controller.safety.fault_manager import (
    FAULT_CODE_EVENTS,
    FaultManager,
    UnroutableFault,
)
from controller.safety.link_monitor import LinkMonitor
from controller.safety.runaway import RowLossVerdict, RowLossWatchdog

__all__ = [
    "FAULT_CODE_EVENTS",
    "EmergencyStop",
    "FaultManager",
    "LinkMonitor",
    "RowLossVerdict",
    "RowLossWatchdog",
    "UnroutableFault",
]
