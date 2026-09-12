"""The nine startup invariants, one contradictory config each.

Testing that the shipped config passes would prove almost nothing — it passes
today by construction.  What has to be tested is that each invariant *fires*,
and that when it does the message names the numbers in conflict.  The whole
point of an invariant here is to tell whoever is reading which value to change;
one that only names itself sends them back to the spreadsheet.

Four of the overrides below are values that were genuinely set while the design
was being drafted and thrown out because the arithmetic contradicted itself:
row_spacing 250, omega_max 60, wheel_diameter 40, track_width 140.  They are
kept as regression cases.
"""

import pytest

from controller import startup_checks
from controller.config import load_config, with_overrides
from controller.startup_checks import ConfigInvalid


def config_with(overrides):
    return with_overrides(load_config(), overrides)


def message_for(overrides) -> str:
    with pytest.raises(ConfigInvalid) as exc:
        startup_checks.run(config_with(overrides))
    return str(exc.value)


def test_the_shipped_config_starts():
    """Every number in config/ agrees with every other one."""
    startup_checks.run(load_config())


def test_the_shipped_config_starts_on_every_backend_with_tuned_gains():
    for backend in ("fake", "isaac"):
        startup_checks.run(config_with({"backend": backend}))


def test_all_nine_invariants_are_checked():
    names = [check.name for check in startup_checks.INVARIANTS]
    assert len(names) == 9
    assert len(set(names)) == 9


def test_a_failure_is_refused_at_startup_not_warned_about():
    """ConfigInvalid, not a log line.  This stands in for the bumper switch the
    MVP does not have."""
    assert issubclass(ConfigInvalid, Exception)
    with pytest.raises(ConfigInvalid):
        startup_checks.run(config_with({"safety.runaway_budget_mm": 20}))


class TestSafetyInvariants:
    def test_1_runaway_on_command_timeout(self):
        """v_max x command_timeout/1000 <= runaway_budget      30 <= 60"""
        message = message_for({"safety.runaway_budget_mm": 20})
        assert "runaway_budget" in message
        assert "command_timeout" in message
        assert "30" in message and "20" in message

    def test_1_fires_on_the_timeout_side_too(self):
        message = message_for({"safety.command_timeout_ms": 1000})
        assert "command_timeout" in message
        assert "1000" in message and "100" in message and "60" in message

    def test_2_runaway_on_row_loss(self):
        """v_max x row_loss_frames/loop_hz <= runaway_budget    30 <= 60"""
        message = message_for({"safety.row_loss_frames": 10})
        assert "row_loss_frames" in message
        assert "runaway_budget" in message
        assert "10" in message and "100" in message and "60" in message

    def test_2_fires_when_the_loop_slows_down(self):
        """A slower control loop spends longer blind on the same frame count."""
        message = message_for({"perception.loop_hz": 2})
        assert "loop_hz" in message
        assert "150" in message and "60" in message


class TestGeometryInvariants:
    def test_3_wheel_diameter_against_soil(self):
        """wheel_diameter >= 4 x soil_variation                 65 >= 60

        Regression: wheel_diameter was 40 while drafting."""
        message = message_for({"rover.wheel_diameter_mm": 40})
        assert "wheel_diameter" in message
        assert "soil_variation" in message
        assert "40" in message and "60" in message and "15" in message

    def test_4_chassis_clearance_against_soil(self):
        """chassis_clearance > soil_variation                   35 > 15"""
        message = message_for({"rover.chassis_clearance_mm": 10})
        assert "chassis_clearance" in message
        assert "soil_variation" in message
        assert "10" in message and "15" in message

    def test_4_equal_is_not_enough(self):
        """Strictly greater: a clearance exactly equal to the soil variation is
        a chassis dragging on every high spot."""
        message = message_for({"rover.chassis_clearance_mm": 15})
        assert "chassis_clearance" in message

    def test_5_clear_furrow_against_body_and_runaway(self):
        """clear_furrow > body_width + 2 x runaway_budget      290 > 265

        Regression: row_spacing was 250 while drafting, which is what this
        invariant caught."""
        message = message_for({"bed.row_spacing_mm": 250})
        assert "clear_furrow" in message
        assert "body_width" in message
        assert "190" in message and "265" in message

    def test_5_uses_clear_furrow_and_not_row_spacing(self):
        """Leaves take 30 mm off each side.  An invariant written against
        row_spacing sees 60 mm of room that is full of foliage."""
        message = message_for({"bed.crop_foliage_half_width_mm": 60})
        assert "clear_furrow" in message
        assert "crop_foliage_half_width" in message
        assert "230" in message

    def test_5_uses_the_width_over_the_wheels(self):
        """body_width is the widest point of the rover, 145 mm, not the 140 mm
        chassis plate.  What hits the leaves is the wheels."""
        message = message_for({"bed.row_spacing_mm": 250})
        assert "145" in message


class TestDriveInvariants:
    def test_6_outer_wheel_against_the_motor_ceiling(self):
        """v + omega_max_rad x track/2 <= wheel_v_max       141.9 <= 202"""
        message = message_for({"rover.drive.wheel_v_max_mm_s": 100})
        assert "wheel_v_max" in message
        assert "141.9" in message and "100" in message

    def test_7_inner_wheel_against_the_motor_deadband(self):
        """v - omega_max_rad x track/2 >= wheel_v_min        58.1 >= 51

        Regression: omega_max was 60 while drafting.  At 60 the inner wheel
        lands at 37 mm/s, inside the deadband, where it stops turning while the
        ESP32 still believes it is driving -- the rover turns harder than
        commanded with nothing to signal it."""
        message = message_for({"rover.drive.omega_max_deg_s": 60})
        assert "wheel_v_min" in message
        assert "37.2" in message and "51" in message

    def test_7_says_not_to_relax_the_deadband_to_pass(self):
        """wheel_v_min is a property of the motor, to be measured at V3.  A
        message that does not say so invites exactly the wrong fix."""
        message = message_for({"rover.drive.omega_max_deg_s": 60})
        assert "omega_max" in message
        assert "motor" in message.lower()

    def test_6_and_7_use_the_configured_track_width(self):
        message = message_for({"rover.drive.omega_max_deg_s": 60})
        assert "120" in message


class TestConfigInvariants:
    def test_8_gains_must_not_be_null_for_the_active_backend(self):
        """Regression-proof by construction: esp32 ships null on purpose."""
        message = message_for({"backend": "esp32"})
        assert "esp32" in message
        assert "k_lat" in message and "k_head" in message

    def test_8_names_the_file_to_edit(self):
        message = message_for({"backend": "esp32"})
        assert "control.yaml" in message

    def test_8_also_fires_when_only_one_gain_is_missing(self):
        message = message_for({"row_follower.gains.fake.k_head": None, "backend": "fake"})
        assert "k_head" in message

    def test_8_fires_for_a_backend_with_no_gains_at_all(self):
        message = message_for({"backend": "nonesuch"})
        assert "nonesuch" in message

    def test_9_simulation_soil_must_match_bed_soil(self):
        """Otherwise the simulator builds soil that contradicts the geometry
        invariants while those invariants still report ok."""
        message = message_for({"simulation.soil.variation_mm": 25})
        assert "soil_variation" in message or "variation_mm" in message
        assert "25" in message and "15" in message


class TestRegressionValues:
    """The numbers that were actually set during design, kept so a future edit
    that reintroduces one is caught the same way it was the first time."""

    @pytest.mark.parametrize(
        "override, expect_in_message",
        [
            ({"safety.runaway_budget_mm": 20}, "runaway_budget"),
            ({"bed.row_spacing_mm": 250}, "clear_furrow"),
            ({"rover.drive.omega_max_deg_s": 60}, "wheel_v_min"),
            ({"rover.wheel_diameter_mm": 40}, "soil_variation"),
        ],
    )
    def test_invariant_rejects_bad_config(self, override, expect_in_message):
        assert expect_in_message in message_for(override)

    def test_track_width_140_passes_but_with_almost_no_margin(self):
        """The fourth drafted value, and the odd one out: track_width 140 does
        not break an invariant, it nearly does.

        With body_width following the track (140 + 26 mm of tyre = 166), the
        clear-furrow margin drops from 24 mm to 4 mm, and the inner wheel lands
        0.1 mm/s above the deadband instead of 7.1.  Both still pass, which is
        why the reason for going to 120 is recorded in cad/parameters/README.md
        rather than enforced here -- and why this test asserts the margins
        rather than a failure that does not happen.
        """
        config = config_with({"rover.track_width_mm": 140, "rover.body_width_mm": 166})
        startup_checks.run(config)

        margins = startup_checks.margins(config)
        assert margins["clear_furrow"] == pytest.approx(4.0)
        assert margins["wheel_v_min"] == pytest.approx(0.13, abs=0.01)

    def test_the_shipped_track_width_keeps_a_real_margin(self):
        margins = startup_checks.margins(load_config())
        assert margins["clear_furrow"] == pytest.approx(25.0)
        assert margins["wheel_v_min"] == pytest.approx(7.11, abs=0.01)


class TestMessageQuality:
    def test_every_violation_is_reported_not_just_the_first(self):
        """One startup run should show everything that is wrong, so a config is
        fixed in one pass rather than one invariant at a time."""
        message = message_for({"rover.wheel_diameter_mm": 40, "rover.chassis_clearance_mm": 10})
        assert "wheel_diameter" in message
        assert "chassis_clearance" in message

    def test_the_error_is_tagged_config_invalid(self):
        assert message_for({"backend": "esp32"}).startswith("config_invalid:")

    @pytest.mark.parametrize(
        "override",
        [
            {"safety.runaway_budget_mm": 20},
            {"safety.row_loss_frames": 10},
            {"rover.wheel_diameter_mm": 40},
            {"rover.chassis_clearance_mm": 10},
            {"bed.row_spacing_mm": 250},
            {"rover.drive.wheel_v_max_mm_s": 100},
            {"rover.drive.omega_max_deg_s": 60},
            {"backend": "esp32"},
            {"simulation.soil.variation_mm": 25},
        ],
    )
    def test_every_message_carries_numbers_and_a_way_out(self, override):
        message = message_for(override)
        assert any(character.isdigit() for character in message)
        assert "->" in message, "the message must say which value to change"
