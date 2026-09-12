"""The sim meshes are generated, not exported by hand.

cad/README.md separates exports/meshes/ (simplified visual + collision, sim
only) from exports/step|stl (manufacturing).  Only the first is generated here.
Manufacturing geometry belongs to a CAD assembly, and this script never touches it.

The point is that a dimension change cannot leave a stale mesh behind: run the
script, and the mesh is the CSV.
"""

import math
from pathlib import Path

import pytest

from tools import generate_sim_meshes as gen

REPO = Path(__file__).resolve().parents[2]
MM_PER_M = 1000.0


@pytest.fixture(scope="module")
def cad() -> dict[str, float]:
    return gen.read_parameters(REPO / "cad" / "parameters" / "parameters.csv")


def test_the_wheel_mesh_is_the_cad_wheel(tmp_path, cad):
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "wheel.obj")

    radius = max(math.hypot(x, z) for x, _, z in vertices)
    half_width = max(abs(y) for _, y, _ in vertices)

    assert radius == pytest.approx(cad["wheel_diameter_mm"] / 2 / MM_PER_M, abs=1e-6)
    assert half_width == pytest.approx(cad["wheel_width_mm"] / 2 / MM_PER_M, abs=1e-9)


def test_the_body_mesh_spans_belly_to_mast_head(tmp_path, cad):
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")

    lowest = min(z for _, _, z in vertices)
    highest = max(z for _, _, z in vertices)
    expected_top = (
        cad["chassis_clearance_mm"] + cad["body_height_mm"] + cad["mast_height_mm"]
    ) / MM_PER_M

    assert lowest == pytest.approx(cad["chassis_clearance_mm"] / MM_PER_M, abs=1e-9)
    assert highest == pytest.approx(expected_top, abs=1e-9)


def test_nothing_in_the_body_mesh_hangs_below_the_axle(tmp_path, cad):
    """chassis_clearance equals the wheel radius, so the belly is the axle
    plane.  Anything below it is a part that would drag — this is the layout
    rule hardware/bom/poc-v3.md states, made checkable."""
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")
    axle_z = cad["wheel_diameter_mm"] / 2 / MM_PER_M
    assert min(z for _, _, z in vertices) >= axle_z - 1e-9


def test_the_frame_never_reaches_the_wheels(tmp_path, cad):
    """Below the wheel tops the body must stay inside the wheel inner faces."""
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")
    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    inner_face = (cad["track_width_mm"] - cad["wheel_width_mm"]) / 2 / MM_PER_M

    widest_in_wheel_zone = max(
        (abs(y) for _, y, z in vertices if z < wheel_top - 1e-9), default=0.0
    )
    assert widest_in_wheel_zone <= inner_face + 1e-9


def test_the_stl_and_the_obj_describe_the_same_solid(tmp_path, cad):
    """Both are written from one Mesh, so they cannot disagree.

    The pair that used to live in cad/exports/meshes/ was exported by hand
    twice, which is exactly how one of them goes stale without anyone noticing.
    """
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])
    obj = gen.read_obj_vertices(tmp_path / "base_link.obj")
    stl = [
        tuple(float(value) for value in line.split()[1:4])
        for line in (tmp_path / "base_link.stl").read_text(encoding="utf-8").splitlines()
        if line.strip().startswith("vertex ")
    ]

    assert stl, "the STL has no vertices"
    for axis in range(3):
        assert min(p[axis] for p in stl) == pytest.approx(min(p[axis] for p in obj), abs=1e-9)
        assert max(p[axis] for p in stl) == pytest.approx(max(p[axis] for p in obj), abs=1e-9)


def _normalise(text: str) -> str:
    """Strip CRLF differences so a Windows checkout cannot fail this test on
    line endings alone -- the comparison below is about content, not bytes."""
    return text.replace("\r\n", "\n")


def test_the_committed_meshes_match_the_generator(tmp_path):
    """The module docstring's promise -- "run the script, and the mesh is the
    CSV" -- rests entirely on this test.  Every test above generates into
    tmp_path and inspects the fresh output only; none of them ever reads
    cad/urdf/meshes/ or cad/exports/meshes/.  Change body_length_mm, forget to
    run the script, and the rest of this file stays green while Isaac loads a
    rover of the old length from the committed meshes.

    This regenerates into tmp_path and compares each committed file, text
    normalised, against that fresh output -- all six: base_link.obj and
    wheel.obj under both cad/urdf/meshes/ and cad/exports/meshes/, plus
    base_link.stl and wheel.stl under cad/exports/meshes/.
    """
    gen.main(REPO, out_dirs=[tmp_path], stl_dirs=[tmp_path])

    committed_dirs = {
        REPO / "cad" / "urdf" / "meshes": ("base_link.obj", "wheel.obj"),
        REPO / "cad" / "exports" / "meshes": (
            "base_link.obj",
            "wheel.obj",
            "base_link.stl",
            "wheel.stl",
        ),
    }

    for directory, names in committed_dirs.items():
        for name in names:
            committed = _normalise((directory / name).read_text(encoding="utf-8"))
            fresh = _normalise((tmp_path / name).read_text(encoding="utf-8"))
            assert committed == fresh, (
                f"{directory / name} does not match what "
                "tools/generate_sim_meshes.py generates from the current "
                "cad/parameters/parameters.csv -- run "
                "`python tools/generate_sim_meshes.py` and commit the result"
            )


def test_total_mass_matches_the_bom():
    properties = gen.mass_properties()
    total = properties["base_link"].mass + 4 * properties["wheel"].mass
    assert total == pytest.approx(35.0, abs=0.01)


def test_the_mass_is_under_the_continuous_torque_ceiling():
    """hardware/bom/poc-v3.md: 4 x 5 N.m at r = 0.125 m gives 160 N, and a 15
    degree grade plus rolling costs m x 3.960 N/kg.  40 kg is the ceiling."""
    properties = gen.mass_properties()
    total = properties["base_link"].mass + 4 * properties["wheel"].mass
    assert total <= 40.0


def test_the_wheel_inertia_is_a_cylinder_about_y():
    """The wheel joint axis is +Y, so Iyy is the spin axis: 0.5 m r^2."""
    wheel = gen.mass_properties()["wheel"]
    radius = 0.125
    assert wheel.iyy == pytest.approx(0.5 * wheel.mass * radius**2, rel=1e-9)
    assert wheel.ixx == pytest.approx(wheel.izz, rel=1e-9)


def test_the_centre_of_mass_is_low_enough_to_be_credible(tmp_path):
    """Battery low and central is the whole point of source spec section 10.
    If the CoM lands above half the body height, the mass model has the battery
    in the wrong place and Isaac will roll the rover on the first slope."""
    base = gen.mass_properties()["base_link"]
    assert base.com[2] < 0.265
    assert base.com[0] == pytest.approx(0.0, abs=1e-9)
    assert base.com[1] == pytest.approx(0.0, abs=1e-9)
