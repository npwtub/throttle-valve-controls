"""Desktop checks: python -m unittest discover -s test -p test_*.py -v"""
import csv
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools.decode_log import COLUMNS, RECORD, RECORD_V4, convert, decode_record, load_engine_tables
from tools.run_test import synchronize

ANGLE_COLUMNS = (
    "measured_motor_angle_deg", "commanded_motor_angle_deg",
    "measured_valve_angle_deg", "commanded_valve_angle_deg",
)


def sample(presence=15, version=3, commanded_thrust_lbf=0.0,
           position_deg=90.0, commanded_position_deg=90.0):
    timestamp = 1_792_540_800_123 if presence & 8 else 0
    return RECORD.pack(version, 13, presence, 15, timestamp,
                       1.25, 2.5, position_deg, 30.5, -50.0, 1200.0, 24.0, 3.25,
                       commanded_position_deg,
                       100, 200, 7, commanded_thrust_lbf)


def sample_v4(presence=111, flags=15, pt3_psi=25.0, position_deg=45.0,
              commanded_position_deg=40.0, motor_temp_c=31, output_percent=-50,
              current_a=3, reserved=0, commanded_thrust_lbf=0.0,
              pid_terms=(1.25, -0.5, 0.25)):
    timestamp = 1_792_540_800_123 if presence & 8 else 0
    return RECORD_V4.pack(4, 15, presence, flags, timestamp,
                          1.25, 2.5, pt3_psi, position_deg,
                          motor_temp_c, output_percent, current_a, reserved,
                          1200.0, 24.0, commanded_position_deg, 7,
                          commanded_thrust_lbf, *pid_terms)


class DecoderTests(unittest.TestCase):
    def test_feedforward_states_and_unknown_fallback(self):
        for state_id, expected in ((15, "feedforward"), (16, "failed"),
                                   (255, "unknown_255")):
            with self.subTest(state_id=state_id):
                data = bytearray(sample())
                data[1] = state_id
                self.assertEqual(decode_record(data)["state"], expected)

    def test_cda_characterization_states(self):
        for state_id, expected in ((17, "moving_up"), (18, "moving_down"),
                                   (19, "at_setpoint_up"), (20, "at_setpoint_down"),
                                   (21, "finished"), (22, "travel_timeout")):
            with self.subTest(state_id=state_id):
                data = bytearray(sample())
                data[1] = state_id
                self.assertEqual(decode_record(data)["state"], expected)

    def test_layout_and_values(self):
        self.assertEqual(RECORD.size, 64)
        row = decode_record(sample())
        self.assertEqual(row["timestamp_unix_ms"], 1_792_540_800_123)
        self.assertEqual(row["state"], "done")
        self.assertEqual(row["pt1_psi"], 1.25)
        self.assertEqual(row["measured_motor_angle_deg"], 90)
        self.assertEqual(row["output_percent"], -50)
        self.assertEqual(row["current_a"], 3.25)
        self.assertEqual(row["commanded_valve_angle_deg"], 90)
        self.assertEqual(row["dropped_records"], 7)
        self.assertEqual(row["format_version"], 3)
        self.assertEqual(row["commanded_thrust_lbf"], 0.0)
        self.assertEqual(row["commanded_mdot_lbm_per_sec"], 0.0)
        self.assertEqual(row["commanded_dp_psi"], 0.0)
        self.assertEqual(row["commanded_chamber_pressure_psi"], 0.0)

    def test_commanded_targets_match_engine_reference(self):
        # Independent 400 lbf reference from the two surrounding engine points.
        row = decode_record(sample(commanded_thrust_lbf=400.0))
        self.assertAlmostEqual(row["commanded_mdot_lbm_per_sec"], 1.2481048568835416, places=6)
        self.assertAlmostEqual(row["commanded_dp_psi"], 106.9387207525376, places=4)
        expected_pressure = 276.1 + (400.0 - 389.8479) / (409.8469 - 389.8479) * (287.98 - 276.1)
        self.assertAlmostEqual(row["commanded_chamber_pressure_psi"], expected_pressure, places=4)

    def test_engine_breakpoints_include_float32_endpoints(self):
        thrust, flow, dp, chamber_pressure, pounds_per_kg = load_engine_tables()
        for index, setpoint in enumerate(thrust):
            with self.subTest(thrust=setpoint):
                row = decode_record(sample(commanded_thrust_lbf=setpoint))
                self.assertAlmostEqual(row["commanded_mdot_lbm_per_sec"],
                                       flow[index] * pounds_per_kg, places=6)
                self.assertAlmostEqual(row["commanded_dp_psi"], dp[index], places=4)
                self.assertAlmostEqual(row["commanded_chamber_pressure_psi"],
                                       chamber_pressure[index], places=4)

    def test_selected_model_headers_supply_tables_and_conversion(self):
        with tempfile.TemporaryDirectory() as folder:
            model_dir = Path(folder)
            (model_dir / "config.h").write_text("""
                // kThrustLbf[] = {1.0, 2.0};
                constexpr double kGearRatio = 40.0;
                constexpr double kBacklashCompensation = 0.5;
                namespace Engine {
                    constexpr double kThrustLbf[] = {100.0, 200.0, 400.0};
                    constexpr double kLoxMdotKg[] = {0.5, 1.0, 2.0};
                    constexpr double kValveDpPsi[] = {400.0, 300.0, 150.0};
                    constexpr double kChamberPressurePsi[] = {10.0, 40.0, 100.0};
                }
                """, encoding="utf-8")
            (model_dir / "feedforward.h").write_text(
                "constexpr double kPoundsMassPerKilogram = 2.0;", encoding="utf-8")
            for thrust, mdot, dp, pressure in (
                    (100.0, 1.0, 400.0, 10.0), (200.0, 2.0, 300.0, 40.0),
                    (300.0, 3.0, 225.0, 70.0), (400.0, 4.0, 150.0, 100.0)):
                with self.subTest(thrust=thrust):
                    row = decode_record(sample(presence=0, commanded_thrust_lbf=thrust), model_dir)
                    self.assertEqual(row["commanded_mdot_lbm_per_sec"], mdot)
                    self.assertEqual(row["commanded_dp_psi"], dp)
                    self.assertEqual(row["commanded_chamber_pressure_psi"], pressure)

            source = model_dir / "test.bin"
            destination = model_dir / "test.csv"
            source.write_bytes(sample(commanded_thrust_lbf=300.0))
            self.assertEqual(convert(source, destination, model_dir), 1)
            with destination.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["commanded_mdot_lbm_per_sec"], "3.0")
            self.assertEqual(row["commanded_dp_psi"], "225.0")
            self.assertEqual(row["commanded_chamber_pressure_psi"], "70.0")

    def test_chamber_pressure_bounds_match_firmware(self):
        for thrust in (-1.0, 169.0, float("nan"), float("inf"), -float("inf")):
            with self.subTest(thrust=thrust):
                row = decode_record(sample(commanded_thrust_lbf=thrust))
                self.assertEqual(row["commanded_chamber_pressure_psi"], "")
        for thrust in (472.0, 1000.0):
            with self.subTest(thrust=thrust):
                row = decode_record(sample(commanded_thrust_lbf=thrust))
                self.assertAlmostEqual(row["commanded_chamber_pressure_psi"], 322.53, places=5)
                self.assertEqual(row["commanded_dp_psi"], "")
                self.assertEqual(row["commanded_mdot_lbm_per_sec"], "")

    def test_invalid_chamber_pressure_table_does_not_create_csv(self):
        for pressure_values in (None, "10.0", "10.0, 0.0", "10.0, -1.0",
                                "10.0, nan", "10.0, inf", "10.0, invalid"):
            with self.subTest(pressure_values=pressure_values):
                with tempfile.TemporaryDirectory() as folder:
                    model_dir = Path(folder)
                    config = """
                        constexpr double kThrustLbf[] = {100.0, 200.0};
                        constexpr double kLoxMdotKg[] = {0.5, 1.0};
                        constexpr double kValveDpPsi[] = {400.0, 300.0};
                    """
                    if pressure_values is not None:
                        config += f"constexpr double kChamberPressurePsi[] = {{{pressure_values}}};"
                    (model_dir / "config.h").write_text(config, encoding="utf-8")
                    source = model_dir / "test.bin"
                    destination = source.with_suffix(".csv")
                    source.write_bytes(sample(commanded_thrust_lbf=150.0))
                    with self.assertRaises(ValueError):
                        convert(source, destination, model_dir)
                    self.assertFalse(destination.exists())

    def test_motor_and_valve_degrees_with_backlash_on_and_off(self):
        # Independent mechanism: 40 motor turns per valve turn, half-turn lash.
        # The half-turn offset is 4.5 valve-side degrees. Every output is degrees.
        with tempfile.TemporaryDirectory() as folder:
            model_dir = Path(folder)
            (model_dir / "config.h").write_text("""
                // constexpr double kGearRatio = 999.0;
                constexpr double kGearRatio = 40.0;
                constexpr double kBacklashCompensation = 0.5;
                """, encoding="utf-8")
            for compensated, expected_command, expected_valve in (
                    (False, 18.0, 22.5), (True, 22.5, 18.0)):
                with self.subTest(compensated=compensated):
                    row = decode_record(sample(
                        presence=15 | (16 if compensated else 0),
                        position_deg=22.5, commanded_position_deg=18.0), model_dir)
                    self.assertEqual(row["backlash_compensated"], int(compensated))
                    self.assertEqual(row["commanded_motor_angle_deg"], expected_command)
                    self.assertEqual(row["measured_motor_angle_deg"], 22.5)
                    self.assertEqual(row["commanded_valve_angle_deg"], 18.0)
                    self.assertEqual(row["measured_valve_angle_deg"], expected_valve)

            # Verify the same units and selected constants survive CSV export.
            source = model_dir / "test.bin"
            source.write_bytes(sample(presence=31, position_deg=22.5,
                                      commanded_position_deg=18.0))
            destination = source.with_suffix(".csv")
            self.assertEqual(convert(source, destination, model_dir), 1)
            with destination.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["commanded_motor_angle_deg"], "22.5")
            self.assertEqual(row["measured_motor_angle_deg"], "22.5")
            self.assertEqual(row["commanded_valve_angle_deg"], "18.0")
            self.assertEqual(row["measured_valve_angle_deg"], "18.0")

            # Taking up slack can move the motor while the valve stays closed.
            row = decode_record(sample(presence=31, position_deg=4.5,
                                       commanded_position_deg=0.0), model_dir)
            self.assertEqual(row["measured_motor_angle_deg"], 4.5)
            self.assertEqual(row["commanded_motor_angle_deg"], 4.5)
            self.assertEqual(row["measured_valve_angle_deg"], 0.0)
            self.assertEqual(row["commanded_valve_angle_deg"], 0.0)

    def test_angle_constants_are_required_and_validated_before_csv_creation(self):
        for gear, backlash, expected_error in (
                (None, "0.5", "kGearRatio"), ("40.0", None, "kBacklashCompensation"),
                ("0.0", "0.5", "kGearRatio"), ("-40.0", "0.5", "kGearRatio"),
                ("nan", "0.5", "kGearRatio"), ("inf", "0.5", "kGearRatio"),
                ("20.0 * 2", "0.5", "kGearRatio"),
                ("40.0", "-0.5", "kBacklashCompensation"),
                ("40.0", "nan", "kBacklashCompensation"),
                ("40.0", "inf", "kBacklashCompensation")):
            with self.subTest(gear=gear, backlash=backlash):
                with tempfile.TemporaryDirectory() as folder:
                    model_dir = Path(folder)
                    constants = ""
                    if gear is not None:
                        constants += f"constexpr double kGearRatio = {gear};\n"
                    if backlash is not None:
                        constants += f"constexpr double kBacklashCompensation = {backlash};\n"
                    (model_dir / "config.h").write_text(constants, encoding="utf-8")
                    source = model_dir / "test.bin"
                    destination = source.with_suffix(".csv")
                    source.write_bytes(sample())
                    with self.assertRaisesRegex(ValueError, expected_error):
                        convert(source, destination, model_dir)
                    self.assertFalse(destination.exists())

    def test_zero_backlash_offset_is_allowed(self):
        with tempfile.TemporaryDirectory() as folder:
            model_dir = Path(folder)
            (model_dir / "config.h").write_text(
                "constexpr double kGearRatio = 40.0;\n"
                "constexpr double kBacklashCompensation = 0.0;", encoding="utf-8")
            row = decode_record(sample(presence=31, position_deg=18.0,
                                       commanded_position_deg=18.0), model_dir)
            for column in ANGLE_COLUMNS:
                self.assertEqual(row[column], 18.0)

    def test_nonfinite_angles_leave_all_angle_columns_blank(self):
        for nonfinite in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=nonfinite):
                row = decode_record(sample(presence=31, position_deg=nonfinite,
                                           commanded_position_deg=nonfinite))
                for column in ANGLE_COLUMNS:
                    self.assertEqual(row[column], "")

    def test_motor_feedback_is_blank_when_motor_absent(self):
        row = decode_record(sample(presence=16))
        self.assertEqual(row["measured_motor_angle_deg"], "")
        self.assertEqual(row["measured_valve_angle_deg"], "")
        self.assertNotEqual(row["commanded_motor_angle_deg"], "")
        self.assertEqual(row["commanded_valve_angle_deg"], 90.0)

    def test_measured_angles_require_status_2_but_preserve_stale_feedback(self):
        for flags, feedback_available in ((0, False), (1, False), (2, True), (3, True)):
            with self.subTest(flags=flags):
                data = bytearray(sample(presence=31))
                data[3] = flags
                row = decode_record(data)
                self.assertEqual(row["is_fresh"], 0)
                self.assertEqual(row["measured_motor_angle_deg"] != "", feedback_available)
                self.assertEqual(row["measured_valve_angle_deg"] != "", feedback_available)
                if feedback_available:
                    self.assertEqual(row["measured_motor_angle_deg"], 90.0)
                self.assertNotEqual(row["commanded_motor_angle_deg"], "")

    def test_invalid_commanded_thrust_leaves_targets_blank(self):
        for thrust in (-1.0, 169.0, 472.0, float("nan"), float("inf"), -float("inf")):
            with self.subTest(thrust=thrust):
                row = decode_record(sample(commanded_thrust_lbf=thrust))
                self.assertEqual(row["commanded_mdot_lbm_per_sec"], "")
                self.assertEqual(row["commanded_dp_psi"], "")

    def test_invalid_model_does_not_create_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            model_dir = Path(folder)
            (model_dir / "config.h").write_text("""
                constexpr double kThrustLbf[] = {100.0, 100.0};
                constexpr double kLoxMdotKg[] = {0.5, 1.0};
                constexpr double kValveDpPsi[] = {400.0, 300.0};
                constexpr double kChamberPressurePsi[] = {10.0, 40.0};
                """, encoding="utf-8")
            source = model_dir / "test.bin"
            destination = model_dir / "test.csv"
            source.write_bytes(sample(commanded_thrust_lbf=100.0))
            with self.assertRaisesRegex(ValueError, "Engine tables"):
                convert(source, destination, model_dir)
            self.assertFalse(destination.exists())

    def test_commanded_thrust_uses_last_four_bytes(self):
        data = sample(commanded_thrust_lbf=430.25)
        self.assertEqual(data[60:64], struct.pack("<f", 430.25))
        row = decode_record(data)
        self.assertEqual(row["commanded_thrust_lbf"], 430.25)
        self.assertEqual(row["commanded_valve_angle_deg"], 90.0)
        self.assertEqual(row["dropped_records"], 7)

    def test_version_2_has_no_commanded_thrust(self):
        # Build the old layout independently to check backward compatibility.
        old_record = struct.Struct("<4BQ9f3I4s")
        data = old_record.pack(2, 13, 15, 15, 1_792_540_800_123,
                               1.25, 2.5, 90.0, 30.5, -50.0, 1200.0, 24.0, 3.25, 90.0,
                               100, 200, 7, b"\0" * 4)
        row = decode_record(data)
        self.assertEqual(row["format_version"], 2)
        self.assertEqual(row["commanded_thrust_lbf"], "")
        self.assertEqual(row["commanded_mdot_lbm_per_sec"], "")
        self.assertEqual(row["commanded_dp_psi"], "")
        self.assertEqual(row["commanded_chamber_pressure_psi"], "")
        self.assertEqual(row["dropped_records"], 7)
        self.assertEqual(row["backlash_compensated"], 0)
        for column in ANGLE_COLUMNS:
            self.assertEqual(row[column], 90.0)

    def test_absent_fields_and_unset_clock(self):
        row = decode_record(sample(presence=0))
        self.assertEqual(row["time_valid"], 0)
        self.assertEqual(row["commanded_thrust_lbf"], 0.0)
        self.assertEqual(decode_record(sample(presence=0, commanded_thrust_lbf=430.0))[
            "commanded_thrust_lbf"], 430.0)
        for field in ("timestamp_unix_ms", "pt1_psi", "measured_motor_angle_deg",
                      "has_status_0"):
            self.assertEqual(row[field], "")

    def test_invalid_format_and_reserved_bytes(self):
        for data in (sample(version=1), sample(version=5), sample()[:-1], b"\0" * 64,
                     sample(version=2)[:-1] + b"x"):
            with self.assertRaises(ValueError):
                decode_record(data)

    def test_invalid_flags_in_both_versions(self):
        for version in (2, 3):
            for offset, invalid_bit in ((2, 16 if version == 2 else 32), (3, 16)):
                with self.subTest(version=version, offset=offset):
                    data = bytearray(sample(version=version))
                    data[offset] |= invalid_bit
                    with self.assertRaisesRegex(ValueError, "Invalid flags"):
                        decode_record(data)

    def test_csv_export_preserves_input_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "test.bin"
            dest = source.with_suffix(".csv")
            original = (sample(commanded_thrust_lbf=430.25) + sample(presence=8) +
                        sample(version=2))
            source.write_bytes(original)
            self.assertEqual(convert(source, dest), 3)
            with dest.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["state"], "done")
            self.assertEqual(next(iter(rows[0])), "timestamp_unix_ms")
            self.assertNotIn("state_id", rows[0])
            self.assertNotIn("timestamp_utc", rows[0])
            self.assertEqual(rows[0]["measured_motor_angle_deg"], "90.0")
            self.assertEqual(rows[0]["has_status_0"], "1")
            self.assertEqual(rows[0]["commanded_thrust_lbf"], "430.25")
            self.assertGreater(float(rows[0]["commanded_mdot_lbm_per_sec"]), 0.0)
            self.assertGreater(float(rows[0]["commanded_dp_psi"]), 0.0)
            self.assertGreater(float(rows[0]["commanded_chamber_pressure_psi"]), 0.0)
            self.assertEqual(rows[1]["motor_present"], "0")
            self.assertEqual(rows[1]["commanded_thrust_lbf"], "0.0")
            self.assertEqual(rows[1]["commanded_mdot_lbm_per_sec"], "0.0")
            self.assertEqual(rows[1]["commanded_dp_psi"], "0.0")
            self.assertEqual(rows[1]["commanded_chamber_pressure_psi"], "0.0")
            self.assertEqual(rows[2]["commanded_thrust_lbf"], "")
            self.assertEqual(rows[2]["commanded_mdot_lbm_per_sec"], "")
            self.assertEqual(rows[2]["commanded_dp_psi"], "")
            self.assertEqual(rows[2]["commanded_chamber_pressure_psi"], "")
            self.assertEqual(tuple(rows[0]), COLUMNS)
            self.assertEqual(list(rows[0])[-16:], ["commanded_thrust_lbf", "commanded_mdot_lbm_per_sec",
                "commanded_dp_psi", "backlash_compensated", *ANGLE_COLUMNS,
                "commanded_chamber_pressure_psi", "pid_p_output_deg", "pid_i_output_deg",
                "pid_d_output_deg", "pid_output_clamped", "pid_correction_deg",
                "feedforward_output_deg", "pid_error_psi"])
            self.assertEqual(tuple(column for column in rows[0]
                                   if "angle" in column or "position" in column), ANGLE_COLUMNS)
            for legacy_column in ("position_deg", "commanded_position_deg",
                                  "commanded_motor_angle", "actual_motor_angle",
                                  "commanded_valve_angle", "actual_valve_angle",
                                  "last_status_0_ms", "last_status_2_ms"):
                self.assertNotIn(legacy_column, rows[0])
                self.assertNotIn(legacy_column, decode_record(sample()))
            self.assertEqual(source.read_bytes(), original)
            with self.assertRaises(FileExistsError):
                convert(source, dest)
            with self.assertRaises(ValueError):
                convert(source, source)

    def test_partial_or_empty_file_does_not_create_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "test.bin"
            dest = source.with_suffix(".csv")
            for data in (b"", sample() + b"bad", sample() + b"\0" * 64):
                source.write_bytes(data)
                with self.assertRaises(ValueError):
                    convert(source, dest)
                self.assertFalse(dest.exists())

    def test_version_4_layout_and_pid_diagnostics(self):
        data = sample_v4()
        self.assertEqual(RECORD_V4.size, 64)
        self.assertEqual(data[12:24], struct.pack("<3f", 1.25, 2.5, 25.0))
        self.assertEqual(data[28:32], bytes((31, 206, 3, 0)))
        self.assertEqual(data[52:64], struct.pack("<3f", 1.25, -0.5, 0.25))
        row = decode_record(data)
        self.assertEqual(row["format_version"], 4)
        self.assertEqual(row["pt3_psi"], 25.0)
        self.assertEqual(row["motor_temp_c"], 31)
        self.assertEqual(row["output_percent"], -50)
        self.assertEqual(row["current_a"], 3)
        self.assertEqual(row["rpm"], 1200.0)
        self.assertEqual(row["voltage_v"], 24.0)
        self.assertEqual(row["measured_motor_angle_deg"], 45.0)
        self.assertEqual(row["commanded_valve_angle_deg"], 40.0)
        self.assertEqual(row["dropped_records"], 7)
        self.assertEqual(row["pid_p_output_deg"], 1.25)
        self.assertEqual(row["pid_i_output_deg"], -0.5)
        self.assertEqual(row["pid_d_output_deg"], 0.25)
        self.assertEqual(row["pid_output_clamped"], 0)
        self.assertEqual(row["pid_correction_deg"], 1.0)
        self.assertEqual(row["feedforward_output_deg"], 39.0)
        self.assertEqual(row["pid_error_psi"], -25.0)

    def test_pid_error_uses_pt3_and_commanded_chamber_pressure(self):
        row = decode_record(sample_v4(pt3_psi=280.0, commanded_thrust_lbf=400.0))
        expected_goal = 276.1 + (400.0 - 389.8479) / (409.8469 - 389.8479) * (287.98 - 276.1)
        self.assertAlmostEqual(row["pid_error_psi"], expected_goal - 280.0, places=4)
        for kwargs in ({"presence": 111 & ~32}, {"pt3_psi": float("nan")},
                       {"commanded_thrust_lbf": -1.0}):
            with self.subTest(kwargs=kwargs):
                self.assertEqual(decode_record(sample_v4(**kwargs))["pid_error_psi"], "")

    def test_pid_clipping_preserves_terms_but_hides_unidentifiable_outputs(self):
        row = decode_record(sample_v4(presence=111 | 128, commanded_position_deg=90.0,
                                      pid_terms=(20.0, 0.0, 0.0)))
        self.assertEqual(row["pid_output_clamped"], 1)
        self.assertEqual(row["pid_p_output_deg"], 20.0)
        self.assertEqual(row["pid_correction_deg"], "")
        self.assertEqual(row["feedforward_output_deg"], "")
        # Reaching an endpoint without actual clipping remains reconstructable.
        row = decode_record(sample_v4(commanded_position_deg=90.0,
                                      pid_terms=(20.0, 0.0, 0.0)))
        self.assertEqual(row["pid_correction_deg"], 20.0)
        self.assertEqual(row["feedforward_output_deg"], 70.0)

    def test_absent_or_nonfinite_pid_does_not_reuse_old_terms(self):
        columns = ("pid_p_output_deg", "pid_i_output_deg", "pid_d_output_deg",
                   "pid_output_clamped", "pid_correction_deg", "feedforward_output_deg")
        for data in (sample_v4(presence=111 & ~64), sample(), sample(version=2)):
            with self.subTest(version=data[0]):
                row = decode_record(data)
                for column in columns:
                    self.assertEqual(row[column], "")
        for invalid in (float("nan"), float("inf")):
            row = decode_record(sample_v4(pid_terms=(invalid, 0.0, 0.0)))
            self.assertEqual(row["pid_correction_deg"], "")
            self.assertEqual(row["feedforward_output_deg"], "")

    def test_version_4_motor_sentinels_and_absent_fields(self):
        row = decode_record(sample_v4(motor_temp_c=-128, output_percent=-128, current_a=255))
        for column in ("motor_temp_c", "output_percent", "current_a"):
            self.assertEqual(row[column], "")
        row = decode_record(sample_v4(presence=0))
        for column in ("timestamp_unix_ms", "pt1_psi", "pt2_psi", "pt3_psi",
                       "motor_temp_c", "output_percent", "current_a", "rpm", "voltage_v",
                       "measured_motor_angle_deg", "measured_valve_angle_deg", "pid_error_psi"):
            self.assertEqual(row[column], "")
        for value in (-127, 127):
            self.assertEqual(decode_record(sample_v4(motor_temp_c=value))["motor_temp_c"], value)
        self.assertEqual(decode_record(sample_v4(current_a=254))["current_a"], 254)

    def test_version_4_validation_and_mixed_export(self):
        for data in (sample_v4(reserved=1), sample_v4(output_percent=101),
                     sample_v4(output_percent=-101)):
            with self.assertRaises(ValueError):
                decode_record(data)
        data = bytearray(sample_v4())
        data[3] |= 16
        with self.assertRaisesRegex(ValueError, "Invalid flags"):
            decode_record(data)
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "test.bin"
            destination = source.with_suffix(".csv")
            source.write_bytes(sample_v4() + sample() + sample(version=2))
            self.assertEqual(convert(source, destination), 3)
            with destination.open(newline="") as stream:
                reader = csv.DictReader(stream)
                rows = list(reader)
            self.assertEqual(reader.fieldnames[4:7], ["pt1_psi", "pt2_psi", "pt3_psi"])
            self.assertEqual(rows[0]["pid_correction_deg"], "1.0")
            self.assertEqual(rows[0]["pt3_psi"], "25.0")
            self.assertEqual(rows[1]["pt3_psi"], "")
            self.assertEqual(rows[2]["pid_p_output_deg"], "")


class FakeSerial:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.sent = []

    def readline(self):
        return next(self.lines, b"")

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def flush(self):
        pass


class HandshakeTests(unittest.TestCase):
    def test_waits_for_ready_and_never_starts_motor(self):
        port = FakeSerial([b"SD log: LOG0000.BIN\n", b"READY_FOR_TIME\n",
                           b"READY_FOR_TIME\n", b"TIME_SET\n"])
        with patch("tools.run_test.time.time_ns", return_value=1_792_540_800_123_000_000):
            synchronize(port, lambda line: None)
        self.assertEqual(port.sent, [b"1792540800123\n"])

    def test_sd_error_does_not_send_time(self):
        port = FakeSerial([b"SD_ERROR: setup failed\n"])
        with self.assertRaises(RuntimeError):
            synchronize(port, lambda line: None)
        self.assertEqual(port.sent, [])

    def test_invalid_time_is_reported(self):
        port = FakeSerial([b"READY_FOR_TIME\n", b"INVALID_TIME\n"])
        with self.assertRaises(RuntimeError):
            synchronize(port, lambda line: None)

    def test_handshake_times_out(self):
        with patch("tools.run_test.time.monotonic", side_effect=[0, 31]):
            with self.assertRaises(TimeoutError):
                synchronize(FakeSerial([]), lambda line: None)


if __name__ == "__main__":
    unittest.main()
