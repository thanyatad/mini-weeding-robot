"""Stage setup: a flat ground plane and the rover on it.

Stage 2 of V1-ISAAC.md, and no more than that.  No heightfield, no crop rows,
no weeds, no cameras - the only question this scene has to answer is whether
the wheels turn when they are told to.  Stage 4 replaces the ground plane with
``environments/soil_heightfield.py``.

The rover is added by referencing ``robots/rover/rover.usd``, the committed
override layer, never ``rover_base.usd`` directly.  Referencing the base would
load a robot with no drive, no damping and no friction, and it would look like
it had loaded correctly: the wheels simply never turn.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
ROVER_USD = HERE / "robots" / "rover" / "rover.usd"
ROVER_PHYSICS_YAML = HERE / "robots" / "rover" / "config.yaml"

#: Where the rover is placed on the stage.  ``wheels.py`` resolves the
#: articulation from this path.
ROVER_PRIM_PATH = "/World/rover"

#: The rigid body that actually moves.  ROVER_PRIM_PATH is the Xform the
#: reference hangs off and it NEVER moves - reading its world pose reports the
#: spawn point forever, which looks exactly like a rover that will not drive.
#: base_link is the articulation root and the only prim worth measuring.
ROVER_BASE_PRIM_PATH = f"{ROVER_PRIM_PATH}/Geometry/base_link"

GROUND_PRIM_PATH = "/World/ground"

#: Deliberately grippier than either coefficient in robots/rover/config.yaml.
#: That material combines with "min", so as long as the ground is the grippier
#: of the two, the rover's own material is the only friction knob and Stage 3
#: has one number to tune instead of two.
GROUND_FRICTION = 1.0


def rover_physics() -> dict[str, Any]:
    """The values ``rover.usd`` authors, read back for reporting.

    Reading this configures nothing - ``rover.usd`` is what PhysX reads.  This
    is the human-readable mirror, and tests/unit/test_rover_usd_override.py is
    what keeps the two honest.
    """
    return yaml.safe_load(ROVER_PHYSICS_YAML.read_text(encoding="utf-8"))


def build_world(config: dict[str, Any]) -> Any:
    """Create the world, the ground and the rover.  Does not reset or step.

    The rover needs no spawn offset: base_link's origin is the soil reference
    plane, the wheel axles sit at +0.035 m and the wheel radius is 0.035 m, so
    the wheels rest exactly on Z = 0 with base_link at the origin.
    """
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage

    physics_hz = float(config["simulation"]["physics_hz"])
    render_hz = float(config["simulation"]["render_hz"])

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / physics_hz,
        rendering_dt=1.0 / render_hz,
    )
    world.scene.add_default_ground_plane(
        prim_path=GROUND_PRIM_PATH,
        static_friction=GROUND_FRICTION,
        dynamic_friction=GROUND_FRICTION,
        restitution=0.0,
    )
    add_reference_to_stage(usd_path=str(ROVER_USD), prim_path=ROVER_PRIM_PATH)
    return world
