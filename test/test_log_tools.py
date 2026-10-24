"""Desktop checks: python -m unittest discover -s test -p test_*.py -v"""
import csv
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools.decode_log import RECORD, convert, decode_record, load_engine_tables
from tools.run_test import synchronize


def sample(presence=15, version=3, commanded_thrust_lbf=0.0):
    timestamp = 1_792_540_800_123 if presence & 8 else 0
    return RECORD.pack(version, 13, presence, 15, timestamp,
                       1.25, 2.5, 90.0, 30.5, -50.0, 1200.0, 24.0, 3.25, 90.0,
                       100, 200, 7, commanded_thrust_lbf)


class DecoderTests(unittest.TestCase):
    def test_feedforward_states_and_unknown_fallback(self):
        for state_id, expected in ((15, "feedforward"), (16, "failed"),
                                   (255, "unknown_255")):
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
        self.assertEqual(row["position_deg"], 90)
        self.assertEqual(row["output_percent"], -50)
        self.assertEqual(row["current_a"], 3.25)
        self.assertEqual(row["commanded_position_deg"], 90)
        self.assertEqual(row["last_status_2_ms"], 200)
        self.assertEqual(row["dropped_records"], 7)
        self.assertEqual(row["format_version"], 3)
        self.assertEqual(row["commanded_thrust_lbf"], 0.0)
        self.assertEqual(row["commanded_mdot_lbm_per_sec"], 0.0)
        self.assertEqual(row["commanded_dp_psi"], 0.0)

    def test_commanded_targets_match_engine_reference(self):
        # Independent 400 lbf reference from the two surrounding engine points.
        row = decode_record(sample(commanded_thrust_lbf=400.0))
        self.assertAlmostEqual(row["commanded_mdot_lbm_per_sec"], 1.2481048568835416, places=6)
        self.assertAlmostEqual(row["commanded_dp_psi"], 106.9387207525376, places=4)

    def test_engine_breakpoints_include_float32_endpoints(self):
        thrust, flow, dp, pounds_per_kg = load_engine_tables()
        for index, setpoint in enumerate(thrust):
            with self.subTest(thrust=setpoint):
                row = decode_record(sample(commanded_thrust_lbf=setpoint))
                self.assertAlmostEqual(row["commanded_mdot_lbm_per_sec"],
                                       flow[index] * pounds_per_kg, places=6)
                self.assertAlmostEqual(row["commanded_dp_psi"], dp[index], places=4)

    def test_selected_model_headers_supply_tables_and_conversion(self):
        with tempfile.TemporaryDirectory() as folder:
            model_dir = Path(folder)
            (model_dir / "config.h").write_text("""
                // kThrustLbf[] = {1.0, 2.0};
                namespace Engine {
                    constexpr double kThrustLbf[] = {100.0, 200.0, 400.0};
                    constexpr double kLoxMdotKg[] = {0.5, 1.0, 2.0};
                    constexpr double kValveDpPsi[] = {400.0, 300.0, 150.0};
                }
                """, encoding="utf-8")
            (model_dir / "feedforward.h").write_text(
                "constexpr double kPoundsMassPerKilogram = 2.0;", encoding="utf-8")
            for thrust, mdot, dp in ((100.0, 1.0, 400.0), (200.0, 2.0, 300.0),
                                    (300.0, 3.0, 225.0), (400.0, 4.0, 150.0)):
                with self.subTest(thrust=thrust):
                    row = decode_record(sample(presence=0, commanded_thrust_lbf=thrust), model_dir)
                    self.assertEqual(row["commanded_mdot_lbm_per_sec"], mdot)
                    self.assertEqual(row["commanded_dp_psi"], dp)

            source = model_dir / "test.bin"
            destination = model_dir / "test.csv"
            source.write_bytes(sample(commanded_thrust_lbf=300.0))
            self.assertEqual(convert(source, destination, model_dir), 1)
            with destination.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["commanded_mdot_lbm_per_sec"], "3.0")
            self.assertEqual(row["commanded_dp_psi"], "225.0")

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
        self.assertEqual(row["commanded_position_deg"], 90.0)
        self.assertEqual(row["last_status_0_ms"], 100)
        self.assertEqual(row["last_status_2_ms"], 200)
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
        self.assertEqual(row["commanded_position_deg"], 90.0)
        self.assertEqual(row["dropped_records"], 7)

    def test_absent_fields_and_unset_clock(self):
        row = decode_record(sample(presence=0))
        self.assertEqual(row["time_valid"], 0)
        self.assertEqual(row["commanded_thrust_lbf"], 0.0)
        self.assertEqual(decode_record(sample(presence=0, commanded_thrust_lbf=430.0))[
            "commanded_thrust_lbf"], 430.0)
        for field in ("timestamp_unix_ms", "pt1_psi", "position_deg",
                      "has_status_0", "last_status_2_ms"):
            self.assertEqual(row[field], "")

    def test_invalid_format_and_reserved_bytes(self):
        for data in (sample(version=1), sample(version=4), sample()[:-1], b"\0" * 64,
                     sample(version=2)[:-1] + b"x"):
            with self.assertRaises(ValueError):
                decode_record(data)

    def test_invalid_flags_in_both_versions(self):
        for version in (2, 3):
            for offset in (2, 3):
                with self.subTest(version=version, offset=offset):
                    data = bytearray(sample(version=version))
                    data[offset] |= 16
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
            self.assertEqual(rows[0]["position_deg"], "90.0")
            self.assertEqual(rows[0]["has_status_0"], "1")
            self.assertEqual(rows[0]["commanded_thrust_lbf"], "430.25")
            self.assertGreater(float(rows[0]["commanded_mdot_lbm_per_sec"]), 0.0)
            self.assertGreater(float(rows[0]["commanded_dp_psi"]), 0.0)
            self.assertEqual(rows[1]["motor_present"], "0")
            self.assertEqual(rows[1]["commanded_thrust_lbf"], "0.0")
            self.assertEqual(rows[1]["commanded_mdot_lbm_per_sec"], "0.0")
            self.assertEqual(rows[1]["commanded_dp_psi"], "0.0")
            self.assertEqual(rows[2]["commanded_thrust_lbf"], "")
            self.assertEqual(rows[2]["commanded_mdot_lbm_per_sec"], "")
            self.assertEqual(rows[2]["commanded_dp_psi"], "")
            self.assertEqual(list(rows[0])[-3:], ["commanded_thrust_lbf",
                "commanded_mdot_lbm_per_sec", "commanded_dp_psi"])
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
