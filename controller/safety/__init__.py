"""Safety: the controller's own layer of the three in spec §5.6.

Only the middle layer lives here.  The ESP32's command timeout and the latching
E-stop are the other two, and each of the three has to work without the one
above it — a layer that leans on the layer above is not a safety layer, it is a
feature.
"""

from controller.safety.runaway import RowLossVerdict, RowLossWatchdog

__all__ = ["RowLossVerdict", "RowLossWatchdog"]
