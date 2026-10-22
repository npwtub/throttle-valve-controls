"""Desktop checks: python -m unittest discover -s test -p test_*.py -v"""
import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.decode_log import RECORD, convert, decode_record
from tools.run_test import synchronize


def sample(presence=15, version=2):
    timestamp = 1_792_540_800_123 if presence & 8 else 0
    return RECORD.pack(version, 13, presence, 15, timestamp,
                       1.25, 2.5, 90.0, 30.5, -50.0, 1200.0, 24.0, 3.25, 90.0,
                       100, 200, 7, b"\0" * 4)


class DecoderTests(unittest.TestCase):
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

    def test_absent_fields_and_unset_clock(self):
        row = decode_record(sample(presence=0))
        self.assertEqual(row["time_valid"], 0)
        for field in ("timestamp_unix_ms", "pt1_psi", "position_deg",
                      "has_status_0", "last_status_2_ms"):
            self.assertEqual(row[field], "")

    def test_invalid_format_and_reserved_bytes(self):
        for data in (sample(version=1), sample()[:-1], b"\0" * 64,
                     sample()[:-1] + b"x"):
            with self.assertRaises(ValueError):
                decode_record(data)

    def test_csv_export_preserves_input_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "test.bin"
            dest = source.with_suffix(".csv")
            original = sample() + sample(presence=8)
            source.write_bytes(original)
            self.assertEqual(convert(source, dest), 2)
            with dest.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["state"], "done")
            self.assertEqual(next(iter(rows[0])), "timestamp_unix_ms")
            self.assertNotIn("state_id", rows[0])
            self.assertNotIn("timestamp_utc", rows[0])
            self.assertEqual(rows[0]["position_deg"], "90.0")
            self.assertEqual(rows[0]["has_status_0"], "1")
            self.assertEqual(rows[1]["motor_present"], "0")
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
