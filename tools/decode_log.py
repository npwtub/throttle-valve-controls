"""Convert a finalized version-2 binary SD log to CSV. Never changes the BIN file."""
import argparse
import csv
from pathlib import Path
import struct
import sys

RECORD = struct.Struct("<4BQ9f3I4s")
STATES = (
    "waiting", "closed", "opening", "open", "closing", "motor froze",
    "initial_opening", "initial_open", "soaking", "cycle_opening",
    "cycle_open", "cycle_closing", "cycle_closed", "done", "timeout",
)
COLUMNS = (
    "timestamp_unix_ms", "state", "time_valid", "motor_present",
    "pt1_psi", "pt2_psi", "position_deg", "motor_temp_c", "output_percent",
    "rpm", "voltage_v", "current_a", "commanded_position_deg",
    "has_status_0", "has_status_2", "has_telemetry", "is_fresh",
    "last_status_0_ms", "last_status_2_ms", "dropped_records",
)


def decode_record(data):
    if len(data) != RECORD.size:
        raise ValueError(f"Incomplete record: expected 64 bytes, got {len(data)}")
    version, state, presence, flags, timestamp, *fields = RECORD.unpack(data)
    if version != 2:
        raise ValueError(f"Unsupported format version {version}; expected 2")
    if presence & ~15 or flags & ~15 or fields[-1] != b"\0" * 4:
        raise ValueError("Invalid flags or reserved bytes")
    valid_time = bool(presence & 8)
    if valid_time and not 1_000_000_000_000 <= timestamp < 4_102_444_800_000:
        raise ValueError("Timestamp marked valid but outside supported range")
    if not valid_time and timestamp != 0:
        raise ValueError("Unset timestamp must be zero")
    row = dict.fromkeys(COLUMNS, "")
    row.update(format_version=version,
               state=STATES[state] if state < len(STATES) else f"unknown_{state}",
               time_valid=int(valid_time), pt1_present=int(bool(presence & 1)),
               pt2_present=int(bool(presence & 2)), motor_present=int(bool(presence & 4)),
               commanded_position_deg=fields[8], dropped_records=fields[11])
    if valid_time:
        row["timestamp_unix_ms"] = timestamp
    if presence & 1:
        row["pt1_psi"] = fields[0]
    if presence & 2:
        row["pt2_psi"] = fields[1]
    if presence & 4:
        for column, value in zip(("position_deg", "motor_temp_c", "output_percent",
                                  "rpm", "voltage_v", "current_a"), fields[2:8]):
            row[column] = value
        for bit, column in enumerate(("has_status_0", "has_status_2",
                                      "has_telemetry", "is_fresh")):
            row[column] = int(bool(flags & (1 << bit)))
        row["last_status_0_ms"], row["last_status_2_ms"] = fields[9:11]
    return row


def convert(source, destination):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError("Input and output must be different files")
    # Validate first so malformed/unfinalized files do not produce plausible CSVs.
    with source.open("rb") as stream:
        count = 0
        while data := stream.read(RECORD.size):
            try:
                decode_record(data)
            except ValueError as error:
                raise ValueError(f"Record {count}, byte offset {count * RECORD.size}: {error}") from error
            count += 1
        if count == 0:
            raise ValueError("Log contains no records")
        stream.seek(0)
        with destination.open("x", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=COLUMNS, extrasaction = "ignore")
            writer.writeheader()
            while data := stream.read(RECORD.size):
                writer.writerow(decode_record(data))
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    args = parser.parse_args()
    output = args.output or args.input.with_suffix(".csv")
    try:
        count = convert(args.input, output)
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    print(f"Decoded {count} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
