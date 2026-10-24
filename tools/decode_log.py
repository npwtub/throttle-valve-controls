"""Convert a finalized version-2 or version-3 binary SD log to CSV. Never changes the BIN file."""
import argparse
from bisect import bisect_left
import csv
from functools import lru_cache
import math
from pathlib import Path
import re
import struct
import sys

RECORD = struct.Struct("<4BQ9f3If")
FLOAT32 = struct.Struct("<f")
DEFAULT_MODEL_DIR = Path(__file__).resolve().parents[1] / "include"
STATES = (
    "waiting", "closed", "opening", "open", "closing", "motor froze",
    "initial_opening", "initial_open", "soaking", "cycle_opening",
    "cycle_open", "cycle_closing", "cycle_closed", "done", "timeout",
    "feedforward", "failed",
)
COLUMNS = (
    "timestamp_unix_ms", "state", "time_valid", "motor_present",
    "pt1_psi", "pt2_psi", "position_deg", "motor_temp_c", "output_percent",
    "rpm", "voltage_v", "current_a", "commanded_position_deg",
    "has_status_0", "has_status_2", "has_telemetry", "is_fresh",
    "last_status_0_ms", "last_status_2_ms", "dropped_records", "commanded_thrust_lbf",
    "commanded_mdot_lbm_per_sec", "commanded_dp_psi",
)


def float32(value):
    """Round each operation like double on the Arduino Mega (binary32)."""
    return FLOAT32.unpack(FLOAT32.pack(value))[0]


@lru_cache(maxsize=8)
def load_engine_tables(model_dir=DEFAULT_MODEL_DIR):
    """Read the numeric firmware constants once, without a second table copy."""
    model_dir = Path(model_dir)

    def read_header(name):
        source = (model_dir / name).read_text(encoding="utf-8")
        return re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.DOTALL)

    config = read_header("config.h")

    def read_array(name):
        match = re.search(rf"\b{re.escape(name)}\s*\[\s*\]\s*=\s*\{{([^}}]*)\}}\s*;", config)
        if not match:
            raise ValueError(f"Missing numeric array {name} in {model_dir / 'config.h'}")
        try:
            return tuple(float32(float(value.strip())) for value in match[1].split(",")
                         if value.strip())
        except (ValueError, OverflowError) as error:
            raise ValueError(f"Invalid numeric array {name} in {model_dir / 'config.h'}") from error

    thrust = read_array("kThrustLbf")
    flow = read_array("kLoxMdotKg")
    dp = read_array("kValveDpPsi")
    if (len(thrust) < 2 or len(thrust) != len(flow) or len(thrust) != len(dp) or
            any(not math.isfinite(value) or value <= 0.0 for table in (thrust, flow, dp)
                for value in table) or
            any(high <= low for low, high in zip(thrust, thrust[1:]))):
        raise ValueError(f"Engine tables must have matching lengths, positive finite values, "
                         f"and increasing thrust in {model_dir / 'config.h'}")

    match = re.search(r"\bkPoundsMassPerKilogram\s*=\s*([^;]+);", read_header("feedforward.h"))
    if not match:
        raise ValueError(f"Missing kPoundsMassPerKilogram in {model_dir / 'feedforward.h'}")
    try:
        pounds_per_kg = float32(float(match[1].strip()))
    except (ValueError, OverflowError) as error:
        raise ValueError("kPoundsMassPerKilogram must be a numeric constant") from error
    if not math.isfinite(pounds_per_kg) or pounds_per_kg <= 0.0:
        raise ValueError("kPoundsMassPerKilogram must be positive and finite")
    return thrust, flow, dp, pounds_per_kg


def commanded_targets(thrust_lbf, model_dir=DEFAULT_MODEL_DIR):
    """Reconstruct nominal LOX flow and valve drop, without runtime PT1 validation."""
    if not math.isfinite(thrust_lbf) or thrust_lbf < 0.0:
        return "", ""
    if thrust_lbf == 0.0:
        return 0.0, 0.0

    thrust, flow, dp, pounds_per_kg = load_engine_tables(model_dir)
    if thrust_lbf < thrust[0] or thrust_lbf > thrust[-1]:
        return "", ""

    high = max(1, bisect_left(thrust, thrust_lbf))
    low = high - 1
    fraction = float32(float32(thrust_lbf - thrust[low]) /
                       float32(thrust[high] - thrust[low]))

    def interpolate(table):
        return float32(table[low] + float32(fraction * float32(table[high] - table[low])))

    try:
        mdot = float32(interpolate(flow) * pounds_per_kg)
        valve_dp = interpolate(dp)
    except OverflowError:
        return "", ""
    if not math.isfinite(mdot) or not math.isfinite(valve_dp) or valve_dp <= 0.0:
        return "", ""
    return mdot, valve_dp


def decode_record(data, model_dir=DEFAULT_MODEL_DIR):
    if len(data) != RECORD.size:
        raise ValueError(f"Incomplete record: expected 64 bytes, got {len(data)}")
    version, state, presence, flags, timestamp, *fields = RECORD.unpack(data)
    if version not in (2, 3):
        raise ValueError(f"Unsupported format version {version}; expected 2 or 3")
    if presence & ~15 or flags & ~15:
        raise ValueError("Invalid flags")
    if version == 2 and data[60:64] != b"\0" * 4:
        raise ValueError("Invalid version-2 reserved bytes")
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
               commanded_position_deg=fields[8], dropped_records=fields[11],
               commanded_thrust_lbf=fields[12] if version == 3 else "")
    if version == 3:
        row["commanded_mdot_lbm_per_sec"], row["commanded_dp_psi"] = commanded_targets(
            fields[12], model_dir)
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


def convert(source, destination, model_dir=DEFAULT_MODEL_DIR):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError("Input and output must be different files")
    # Validate first so malformed/unfinalized files do not produce plausible CSVs.
    with source.open("rb") as stream:
        count = 0
        while data := stream.read(RECORD.size):
            try:
                decode_record(data, model_dir)
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
                writer.writerow(decode_record(data, model_dir))
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR,
                        help="Directory containing the test's config.h and feedforward.h "
                             "(default: this project's include directory)")
    args = parser.parse_args()
    output = args.output or args.input.with_suffix(".csv")
    try:
        count = convert(args.input, output, args.model_dir)
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    print(f"Decoded {count} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
