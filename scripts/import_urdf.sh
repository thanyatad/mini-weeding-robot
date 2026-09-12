#!/usr/bin/env bash
#
# cad/urdf/weeding_rover.urdf  ->  sim/isaac/robots/rover/rover_base.usd
#
# rover_base.usd is GENERATED and gitignored.  It is overwritten every run and
# must never be edited: drive gains, damping, friction and contact offsets go
# in rover.usd, the committed layer stacked on top of it, so that regenerating
# from CAD cannot destroy them.  See cad/README.md#two-usd-layers.
#
# Runs under Isaac Sim's own Python, which is 3.12 - not the interpreter the
# controller's unit tests use.  Nothing under tests/unit/ imports anything this
# writes.
#
#   ./scripts/import_urdf.sh
#   ISAAC_SIM_PATH=/path/to/isaac ./scripts/import_urdf.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaac}"

if [ -x "$ISAAC_SIM_PATH/python.sh" ]; then
  ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"
elif [ -f "$ISAAC_SIM_PATH/python.bat" ]; then
  ISAAC_PYTHON="$ISAAC_SIM_PATH/python.bat"
else
  echo "no Isaac Sim Python at $ISAAC_SIM_PATH — set ISAAC_SIM_PATH" >&2
  exit 1
fi

exec "$ISAAC_PYTHON" "$REPO/scripts/import_urdf.py" "$@"
