"""Generate the simplified sim meshes and mass properties from parameters.csv.

cad/README.md splits the exports: `exports/meshes/` is simplified visual and
collision geometry for the simulator, `exports/step|stl` is for manufacturing.
Only the first is generated here.  Fusion still owns everything a shop would
cut, and that work can follow later without blocking Isaac.

This reads a CSV.  It is NOT Fusion-API tooling, which cad/parameters/README.md
warns against on maintenance-cost grounds -- there is nothing here that breaks
when Autodesk ships a new version.

The mass model is an ESTIMATE (hardware/bom/poc-v3.md, +/-20%).  When real
parts are weighed, MASS_ITEMS below is the single place to change.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

MM_PER_M = 1000.0

#: How finely the wheel cylinder is tessellated.  The wheel's COLLISION is a
#: primitive cylinder in the URDF -- this mesh is visual only, so the segment
#: count buys appearance, not physics.  Isaac pays for every triangle.
WHEEL_SEGMENTS = 32

#: The body shell's width above the wheel tops.  NOT derivable from the CSV:
#: body_width is the overall width including the wheels, and the shell is
#: narrower than that.  Design spec section 3.2 draws the body as
#: 500 x 380 x 280, and section 9.2 specifies the URDF's upper collision box
#: as 0.500 x 0.380 x 0.155.  MASS_ITEMS below derives the shell row's width
#: from this constant, so the two cannot drift apart the way the URDF's
#: separately-typed 0.380 literal can.
BODY_SHELL_WIDTH_MM = 380.0


@dataclass(frozen=True)
class LinkInertia:
    """Mass properties in kg and metres, about the link's own centre of mass."""

    mass: float
    com: tuple[float, float, float]
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


@dataclass(frozen=True)
class Box:
    """An axis-aligned box, in metres, positioned in the base_link frame."""

    size: tuple[float, float, float]
    centre: tuple[float, float, float]


@dataclass(frozen=True)
class Mesh:
    """Vertices in metres, faces as 0-based index tuples.

    Faces may have more than three sides; OBJ takes them as they are and STL
    triangulates on the way out.  Keeping one representation means the two
    formats can never describe different geometry.
    """

    vertices: list[tuple[float, float, float]]
    faces: list[tuple[int, ...]]

    def merge(self, other: Mesh) -> Mesh:
        offset = len(self.vertices)
        return Mesh(
            self.vertices + other.vertices,
            self.faces + [tuple(index + offset for index in face) for face in other.faces],
        )


def read_parameters(path: Path) -> dict[str, float]:
    lines = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    return {row["name"]: float(row["value"]) for row in csv.DictReader(lines)}


def read_obj_vertices(path: Path) -> list[tuple[float, float, float]]:
    """Every `v` line of an OBJ, as (x, y, z) in metres."""
    vertices = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            x, y, z = (float(v) for v in line.split()[1:4])
            vertices.append((x, y, z))
    return vertices


# -- geometry ---------------------------------------------------------------


def body_boxes(cad: dict[str, float]) -> list[Box]:
    """The three boxes base_link is built from, in metres.

    The frame necks in below the wheel tops: chassis_plate_width is the inner
    face spacing, and the body above the wheels is free to overhang at
    body_width minus the wheels.  See the design doc section 3.2.
    """
    belly = cad["chassis_clearance_mm"] / MM_PER_M
    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    body_top = (cad["chassis_clearance_mm"] + cad["body_height_mm"]) / MM_PER_M
    length = cad["body_length_mm"] / MM_PER_M
    frame_width = cad["chassis_plate_width_mm"] / MM_PER_M
    shell_width = BODY_SHELL_WIDTH_MM / MM_PER_M

    frame_height = wheel_top - belly
    shell_height = body_top - wheel_top

    return [
        Box((length, frame_width, frame_height), (0.0, 0.0, belly + frame_height / 2)),
        Box((length, shell_width, shell_height), (0.0, 0.0, wheel_top + shell_height / 2)),
    ]


def mast_dimensions(cad: dict[str, float]) -> tuple[float, float, float]:
    """(radius, height, centre_z) of the mast tube, in metres."""
    radius = cad["mast_diameter_mm"] / 2 / MM_PER_M
    height = cad["mast_height_mm"] / MM_PER_M
    body_top = (cad["chassis_clearance_mm"] + cad["body_height_mm"]) / MM_PER_M
    return radius, height, body_top + height / 2


# -- mass model -------------------------------------------------------------

#: (name, mass_kg, Box in millimetres) -- hardware/bom/poc-v3.md section มวล.
#: Millimetres here so the table reads like the BOM; converted below.
MASS_ITEMS: tuple[
    tuple[str, float, tuple[float, float, float], tuple[float, float, float]], ...
] = (
    ("frame", 8.5, (500, 340, 125), (0, 0, 187.5)),
    ("battery", 5.0, (300, 200, 120), (0, 0, 190)),
    ("power_box", 1.5, (200, 150, 80), (-150, 0, 300)),
    ("compute", 1.5, (200, 150, 80), (150, 0, 300)),
    ("shell", 3.0, (500, BODY_SHELL_WIDTH_MM, 155), (0, 0, 327.5)),
    ("motors", 6.0, (400, 300, 100), (0, 0, 125)),
    ("fasteners", 1.1, (500, 340, 125), (0, 0, 187.5)),
)

#: One wheel: tyre, rim and hub together.  Modelled as a solid cylinder, which
#: overstates the inertia of a tyre (mass sits at the rim), and is the right
#: direction to be wrong in for a stability estimate.
WHEEL_MASS_KG = 1.8

#: The mast tube plus the sensor head it carries.
MAST_MASS_KG = 1.2


def _box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    a, b, c = size
    return (
        mass * (b * b + c * c) / 12.0,
        mass * (a * a + c * c) / 12.0,
        mass * (a * a + b * b) / 12.0,
    )


def mass_properties(cad: dict[str, float] | None = None) -> dict[str, LinkInertia]:
    """base_link and one wheel, about each link's own centre of mass."""
    if cad is None:
        cad = read_parameters(
            Path(__file__).resolve().parents[1] / "cad" / "parameters" / "parameters.csv"
        )

    radius, height, mast_centre_z = mast_dimensions(cad)

    parts: list[tuple[float, tuple[float, float, float], tuple[float, float, float]]] = []
    for _name, mass, size_mm, centre_mm in MASS_ITEMS:
        size = tuple(v / MM_PER_M for v in size_mm)
        centre = tuple(v / MM_PER_M for v in centre_mm)
        parts.append((mass, centre, _box_inertia(mass, size)))

    mast_own = (
        MAST_MASS_KG * (3 * radius * radius + height * height) / 12.0,
        MAST_MASS_KG * (3 * radius * radius + height * height) / 12.0,
        MAST_MASS_KG * radius * radius / 2.0,
    )
    parts.append((MAST_MASS_KG, (0.0, 0.0, mast_centre_z), mast_own))

    total = sum(mass for mass, _, _ in parts)
    com = tuple(sum(mass * centre[axis] for mass, centre, _ in parts) / total for axis in range(3))

    ixx = iyy = izz = 0.0
    for mass, centre, own in parts:
        dx, dy, dz = (centre[axis] - com[axis] for axis in range(3))
        ixx += own[0] + mass * (dy * dy + dz * dz)
        iyy += own[1] + mass * (dx * dx + dz * dz)
        izz += own[2] + mass * (dx * dx + dy * dy)

    wheel_radius = cad["wheel_diameter_mm"] / 2 / MM_PER_M
    wheel_width = cad["wheel_width_mm"] / MM_PER_M
    wheel_transverse = WHEEL_MASS_KG * (3 * wheel_radius**2 + wheel_width**2) / 12.0

    return {
        "base_link": LinkInertia(total, com, ixx, iyy, izz),
        "wheel": LinkInertia(
            WHEEL_MASS_KG,
            (0.0, 0.0, 0.0),
            wheel_transverse,
            0.5 * WHEEL_MASS_KG * wheel_radius**2,
            wheel_transverse,
        ),
    }


# -- mesh building ----------------------------------------------------------


def box_mesh(box: Box) -> Mesh:
    (a, b, c), (cx, cy, cz) = box.size, box.centre
    vertices = [
        (cx + sx * a / 2, cy + sy * b / 2, cz + sz * c / 2)
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    faces = [
        (0, 1, 3, 2),
        (4, 6, 7, 5),
        (0, 4, 5, 1),
        (2, 3, 7, 6),
        (0, 2, 6, 4),
        (1, 5, 7, 3),
    ]
    return Mesh(vertices, faces)


def cylinder_mesh(
    radius: float, length: float, axis: str, centre: tuple[float, float, float]
) -> Mesh:
    """A closed cylinder tessellated into WHEEL_SEGMENTS, about `axis`."""
    half = length / 2
    vertices: list[tuple[float, float, float]] = []
    for end in (-half, half):
        for segment in range(WHEEL_SEGMENTS):
            angle = 2 * math.pi * segment / WHEEL_SEGMENTS
            u, v = radius * math.cos(angle), radius * math.sin(angle)
            point = (u, end, v) if axis == "y" else (u, v, end)
            vertices.append(tuple(point[i] + centre[i] for i in range(3)))

    faces: list[tuple[int, ...]] = []
    for segment in range(WHEEL_SEGMENTS):
        nxt = (segment + 1) % WHEEL_SEGMENTS
        faces.append((segment, nxt, WHEEL_SEGMENTS + nxt, WHEEL_SEGMENTS + segment))
    faces.append(tuple(range(WHEEL_SEGMENTS - 1, -1, -1)))
    faces.append(tuple(range(WHEEL_SEGMENTS, 2 * WHEEL_SEGMENTS)))
    return Mesh(vertices, faces)


def build_meshes(cad: dict[str, float]) -> dict[str, Mesh]:
    base = Mesh([], [])
    for box in body_boxes(cad):
        base = base.merge(box_mesh(box))

    radius, height, centre_z = mast_dimensions(cad)
    base = base.merge(cylinder_mesh(radius, height, "z", (0.0, 0.0, centre_z)))

    wheel = cylinder_mesh(
        cad["wheel_diameter_mm"] / 2 / MM_PER_M,
        cad["wheel_width_mm"] / MM_PER_M,
        "y",
        (0.0, 0.0, 0.0),
    )
    return {"base_link": base, "wheel": wheel}


# -- writing ----------------------------------------------------------------

_BANNER = (
    "generated by tools/generate_sim_meshes.py from cad/parameters/parameters.csv. "
    "Do not edit by hand: run the script. Units are metres."
)


def write_obj(path: Path, name: str, mesh: Mesh) -> None:
    lines = [f"# {name} - {_BANNER}", f"o {name}"]
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in mesh.vertices]
    lines += ["f " + " ".join(str(index + 1) for index in face) for face in mesh.faces]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (0.0, 0.0, 0.0) if length == 0.0 else (nx / length, ny / length, nz / length)


def write_stl(path: Path, name: str, mesh: Mesh) -> None:
    """ASCII STL, fan-triangulated from the same Mesh the OBJ is written from.

    ASCII rather than binary on purpose: this geometry is a handful of boxes and
    two cylinders, so the file is small, and a text file is one a reviewer can
    open.  The Fusion exports under exports/stl/ are a different matter and stay
    binary and LFS-tracked.
    """
    lines = [f"solid {name}"]
    for face in mesh.faces:
        for corner in range(1, len(face) - 1):
            triangle = (
                mesh.vertices[face[0]],
                mesh.vertices[face[corner]],
                mesh.vertices[face[corner + 1]],
            )
            nx, ny, nz = _normal(*triangle)
            lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
            lines.append("    outer loop")
            lines += [f"      vertex {x:.6f} {y:.6f} {z:.6f}" for x, y, z in triangle]
            lines.append("    endloop")
            lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(
    repo_root: Path,
    out_dirs: list[Path] | None = None,
    stl_dirs: list[Path] | None = None,
) -> None:
    cad = read_parameters(repo_root / "cad" / "parameters" / "parameters.csv")
    exports = repo_root / "cad" / "exports" / "meshes"
    if out_dirs is None:
        out_dirs = [repo_root / "cad" / "urdf" / "meshes", exports]
    if stl_dirs is None:
        # Only exports/meshes gets STL: the URDF references .obj, so urdf/meshes
        # carries nothing it does not use.  exports/stl/ is manufacturing
        # geometry and belongs to Fusion, not to this script.
        stl_dirs = [exports]

    meshes = build_meshes(cad)
    for name, mesh in meshes.items():
        for directory in out_dirs:
            write_obj(directory / f"{name}.obj", name, mesh)
        for directory in stl_dirs:
            write_stl(directory / f"{name}.stl", name, mesh)


def _print_inertia() -> None:
    for name, properties in mass_properties().items():
        x, y, z = properties.com
        print(f"<!-- {name} -->")
        print("<inertial>")
        print(f'  <origin xyz="{x:.5f} {y:.5f} {z:.5f}" rpy="0 0 0"/>')
        print(f'  <mass value="{properties.mass:.6f}"/>')
        print(
            f'  <inertia ixx="{properties.ixx:.9f}" ixy="0" ixz="0"'
            f' iyy="{properties.iyy:.9f}" iyz="0" izz="{properties.izz:.9f}"/>'
        )
        print("</inertial>")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-inertia", action="store_true")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if arguments.print_inertia:
        _print_inertia()
    else:
        main(root)
        print(f"wrote sim meshes from {root / 'cad' / 'parameters' / 'parameters.csv'}")
