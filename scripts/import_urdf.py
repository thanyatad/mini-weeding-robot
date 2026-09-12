"""Import cad/urdf/weeding_rover.urdf into sim/isaac/robots/rover/rover_base.usd.

Run through ``scripts/import_urdf.sh``, which supplies Isaac's own Python.  This
file is a standalone Isaac Sim script and cannot be imported by pytest.

Everything this writes is disposable.  ``rover_base.usd`` is gitignored and
overwritten on every run; the tuning that must survive a CAD change - drive
gains, damping, friction, contact offsets - lives in ``rover.usd``, which is the
committed layer stacked on top of it.  cad/README.md#two-usd-layers is the rule,
and the reason the import settings below refuse to bake any of it.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "cad" / "urdf" / "weeding_rover.urdf"
OUT_DIR = REPO / "sim" / "isaac" / "robots" / "rover"
OUT_USD = OUT_DIR / "rover_base.usd"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--urdf", default=str(URDF))
parser.add_argument("--out", default=str(OUT_USD))
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402

_manager = omni.kit.app.get_app().get_extension_manager()
_manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
_manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)

from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig  # noqa: E402


def build_config(urdf_path: Path, out_dir: Path) -> URDFImporterConfig:
    """Every one of these is set explicitly, including the ones that already
    match the importer's default.  A default that moves between Isaac versions
    would otherwise change the robot without changing this repository."""
    config = URDFImporterConfig()
    config.urdf_path = str(urdf_path)

    # usd_path is a DIRECTORY.  Left None the importer writes next to the URDF,
    # which would drop generated USD into cad/urdf/.
    config.usd_path = str(out_dir)

    # True collapses camera_front_link and camera_down_link into base_link.
    # Both camera frames disappear, the sensors at Stage 4 have nothing to
    # attach to, and nothing reports an error.
    config.merge_fixed_joints = False

    # It is a mobile robot.  None would leave the source authoring untouched.
    config.fix_base = False

    # base_link is one rigid body - chassis, motors and brackets all live in it,
    # so there is nothing inside it that can collide.
    config.allow_self_collision = False

    # The URDF already declares a box and four cylinders.  Generating collision
    # from visuals replaces the wheel cylinder with a hull of the visual mesh,
    # which is what cad/urdf/README.md#collision exists to forbid: a tread mesh
    # on a heightfield makes a very large number of contact points and buys no
    # accuracy, because grip comes from the friction coefficient.
    config.collision_from_visuals = False

    # The importer CAN bake a velocity drive and damping in here.  It must not:
    # this file is regenerated whenever CAD changes, and friction and damping
    # are the most expensive values in the project to re-tune.  They belong in
    # rover.usd.
    config.joint_target_type = None
    config.joint_drive_type = None
    config.override_joint_stiffness = None
    config.override_joint_damping = None

    # Meshes under cad/urdf/meshes/ are authored in metres with no scale
    # attribute, and the stage is metres.
    config.merge_mesh = False
    config.link_density = None

    return config


#: Everything a run produces, relative to the output directory.  Wiped before
#: each import so that a mesh or a link deleted in CAD cannot survive as a stale
#: file that the stage still loads.
GENERATED = ("rover_base.usd", "payloads", "Textures")


def clean(out_dir: Path, out_usd: Path) -> None:
    for name in GENERATED:
        target = out_dir / name
        if target.is_dir():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()
    if out_usd.exists():
        out_usd.unlink()


def flatten(written: Path, out_usd: Path) -> None:
    """Move the importer's asset folder up into the output directory.

    The importer writes ``<out_dir>/<robot name>/<robot name>.usd`` with its
    payload layers beside it, referenced as ``./payloads/base.usda`` - relative
    to the USD file's own directory.  Renaming the file without moving the
    payloads with it produces a stage that opens clean and contains nothing,
    which is why this is done here rather than by hand.
    """
    asset_dir = written.parent
    if asset_dir == out_usd.parent:
        written.replace(out_usd)
        return

    for child in list(asset_dir.iterdir()):
        if child == written:
            continue
        destination = out_usd.parent / child.name
        if destination.is_dir():
            shutil.rmtree(destination)
        elif destination.exists():
            destination.unlink()
        shutil.move(os.fspath(child), os.fspath(destination))

    written.replace(out_usd)
    asset_dir.rmdir()


def main() -> int:
    urdf_path = Path(args.urdf).resolve()
    out_usd = Path(args.out).resolve()
    out_dir = out_usd.parent

    if not urdf_path.is_file():
        print(f"no URDF at {urdf_path}", file=sys.stderr)
        return 1

    out_dir.mkdir(parents=True, exist_ok=True)
    clean(out_dir, out_usd)

    importer = URDFImporter(build_config(urdf_path, out_dir))
    written = importer.import_urdf()
    if not written:
        print(f"importer returned no output path for {urdf_path}", file=sys.stderr)
        return 1

    flatten(Path(written).resolve(), out_usd)
    print(f"wrote {out_usd.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    status = main()
    simulation_app.close()
    sys.exit(status)
