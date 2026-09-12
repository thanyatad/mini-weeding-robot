"""The seven config files must load, merge without collisions, and still carry
the numbers config/README.md documents.

These are not startup invariants (those live in test_startup_invariants.py) —
this file guards the plumbing: that every documented file exists, that no two
files claim the same top-level key, and that the values a reader of the README
expects are actually the values the code will load.
"""

import pytest

from controller import config as cfg


def test_every_documented_file_is_loaded():
    assert set(cfg.CONFIG_FILES) == {
        "rover.yaml",
        "safety.yaml",
        "perception.yaml",
        "control.yaml",
        "simulation.yaml",
        "camera.yaml",
        "development.yaml",
    }


def test_load_merges_all_top_level_sections():
    merged = cfg.load_config()
    assert set(merged) == {
        "rover",
        "bed",
        "safety",
        "perception",
        "row_follower",
        "simulation",
        "cameras",
        "backend",
        "logging",
    }


@pytest.mark.parametrize(
    "dotted, expected",
    [
        ("rover.track_width_mm", 430),
        ("rover.body_width_mm", 520),
        ("rover.wheel_diameter_mm", 250),
        ("rover.chassis_clearance_mm", 125),
        ("rover.drive.v_max_mm_s", 160),
        ("rover.drive.omega_max_deg_s", 25),
        ("rover.drive.wheel_v_max_mm_s", 327),
        ("rover.drive.wheel_v_min_mm_s", 51),
        ("bed.soil_variation_mm", 15),
        ("bed.row_spacing_mm", 750),
        ("bed.crop_foliage_half_width_mm", 30),
        ("safety.runaway_budget_mm", 60),
        ("safety.row_loss_frames", 3),
        ("safety.command_timeout_ms", 300),
        ("safety.link_lost_ms", 500),
        ("safety.ack_timeout_ms", 200),
        ("perception.loop_hz", 10),
        ("perception.exg.exg_floor", 12),
        ("row_follower.v_mm_s", 160),
        ("row_follower.omega_max_deg_s", 25),
        ("row_follower.gains.fake.k_lat", 45.0),
        ("row_follower.gains.fake.k_head", 25.0),
        ("simulation.soil.variation_mm", 15),
    ],
)
def test_documented_values(dotted, expected):
    assert cfg.get(cfg.load_config(), dotted) == expected


def test_esp32_gains_are_deliberately_null():
    """Not a forgotten value — the guard that makes invariant 8 fire."""
    merged = cfg.load_config()
    assert cfg.get(merged, "row_follower.gains.esp32.k_lat") is None
    assert cfg.get(merged, "row_follower.gains.esp32.k_head") is None


def test_derived_from_cad_values_keep_their_marker():
    """`# derived: cad ...` is how a reader knows the source of truth is CAD,
    not this file.  Losing the comment loses the only warning."""
    text = (cfg.CONFIG_DIR / "rover.yaml").read_text(encoding="utf-8")
    for parameter in (
        "track_width",
        "wheelbase",
        "body_width",
        "wheel_diameter",
        "chassis_clearance",
    ):
        assert f"# derived: cad {parameter}" in text


def test_with_overrides_does_not_mutate_the_original():
    base = cfg.load_config()
    patched = cfg.with_overrides(base, {"bed.row_spacing_mm": 250})
    assert cfg.get(patched, "bed.row_spacing_mm") == 250
    assert cfg.get(base, "bed.row_spacing_mm") == 750


def test_with_overrides_rejects_an_unknown_key():
    """A typo in an override must fail loudly, not silently add a new key that
    nothing reads."""
    with pytest.raises(KeyError):
        cfg.with_overrides(cfg.load_config(), {"bed.row_spacng_mm": 250})
