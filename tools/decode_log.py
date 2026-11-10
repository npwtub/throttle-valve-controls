"""Convert a finalized version-2, version-3, or version-4 SD log to CSV. Never changes the BIN file."""
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
RECORD_V4 = struct.Struct("<4BQ4fbbBB3fI4f")
FLOAT32 = struct.Struct("<f")
DEFAULT_MODEL_DIR = Path(__file__).resolve().parents[1] / "include"
STATES = (
    "waiting", "closed", "opening", "open", "closing", "motor froze",
    "initial_opening", "initial_open", "soaking", "cycle_opening",
    "cycle_open", "cycle_closing", "cycle_closed", "done", "timeout",
    "feedforward", "failed",
    "moving_up", "moving_down", "at_setpoint_up", "at_setpoint_down",
    "finished", "travel_timeout",
)
COLUMNS = (
    "timestamp_unix_ms", "state", "time_valid", "motor_present",
    "pt1_psi", "pt2_psi", "pt3_psi", "motor_temp_c", "output_percent",
    "rpm", "voltage_v", "current_a",
    "has_status_0", "has_status_2", "has_telemetry", "is_fresh",
    "dropped_records", "commanded_thrust_lbf",
    "commanded_mdot_lbm_per_sec", "commanded_dp_psi",
    "backlash_compensated",
    "measured_motor_angle_deg", "commanded_motor_angle_deg",
    "measured_valve_angle_deg", "commanded_valve_angle_deg",
    "commanded_chamber_pressure_psi",
    "pid_p_output_deg", "pid_i_output_deg", "pid_d_output_deg", "pid_output_clamped",
    "pid_correction_deg", "feedforward_output_deg", "pid_error_psi",
)


def float32(value):
    """Round each operation like double on the Arduino Mega (binary32)."""
    return FLOAT32.unpack(FLOAT32.pack(value))[0]


def read_model_header(model_dir, name):
    source = (Path(model_dir) / name).read_text(encoding="utf-8")
    return re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.DOTALL)


@lru_cache(maxsize=8)
def load_angle_constants(model_dir=DEFAULT_MODEL_DIR):
    """Use the gear ratio and backlash offset from the recorded test's firmware."""
    config = read_model_header(model_dir, "config.h")

    def read_constant(name):
        match = re.search(rf"\b{re.escape(name)}\s*=\s*([^;]+);", config)
        if not match:
            raise ValueError(f"Missing numeric constant {name} in {Path(model_dir) / 'config.h'}")
        try:
            value = float32(float(match[1].strip()))
        except (ValueError, OverflowError) as error:
            raise ValueError(f"{name} must be a numeric constant") from error
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    gear_ratio = read_constant("kGearRatio")
    backlash_motor_rotations = read_constant("kBacklashCompensation")
    if gear_ratio <= 0.0:
        raise ValueError("kGearRatio must be positive")
    if backlash_motor_rotations < 0.0:
        raise ValueError("kBacklashCompensation must be nonnegative")
    return gear_ratio, backlash_motor_rotations


@lru_cache(maxsize=8)
def load_engine_tables(model_dir=DEFAULT_MODEL_DIR):
    """Read the numeric firmware constants once, without a second table copy."""
    model_dir = Path(model_dir)

    config = read_model_header(model_dir, "config.h")

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
    chamber_pressure = read_array("kChamberPressurePsi")
    if (len(thrust) < 2 or len(thrust) != len(flow) or len(thrust) != len(dp) or
            len(thrust) != len(chamber_pressure) or
            any(not math.isfinite(value) or value <= 0.0
                for table in (thrust, flow, dp, chamber_pressure)
                for value in table) or
            any(high <= low for low, high in zip(thrust, thrust[1:]))):
        raise ValueError(f"Engine tables must have matching lengths, positive finite values, "
                         f"and increasing thrust in {model_dir / 'config.h'}")

    match = re.search(r"\bkPoundsMassPerKilogram\s*=\s*([^;]+);",
                      read_model_header(model_dir, "feedforward.h"))
    if not match:
        raise ValueError(f"Missing kPoundsMassPerKilogram in {model_dir / 'feedforward.h'}")
    try:
        pounds_per_kg = float32(float(match[1].strip()))
    except (ValueError, OverflowError) as error:
        raise ValueError("kPoundsMassPerKilogram must be a numeric constant") from error
    if not math.isfinite(pounds_per_kg) or pounds_per_kg <= 0.0:
        raise ValueError("kPoundsMassPerKilogram must be positive and finite")
    return thrust, flow, dp, chamber_pressure, pounds_per_kg


def commanded_targets(thrust_lbf, model_dir=DEFAULT_MODEL_DIR):
    """Reconstruct nominal LOX flow and valve drop, without runtime PT1 validation."""
    if not math.isfinite(thrust_lbf) or thrust_lbf < 0.0:
        return "", ""
    if thrust_lbf == 0.0:
        return 0.0, 0.0

    thrust, flow, dp, _, pounds_per_kg = load_engine_tables(model_dir)
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


def commanded_chamber_pressure(thrust_lbf, model_dir=DEFAULT_MODEL_DIR):
    """Match utils::chamber_pressure_for_thrust(), including its upper clamp."""
    if not math.isfinite(thrust_lbf) or thrust_lbf < 0.0:
        return ""
    if thrust_lbf == 0.0:
        return 0.0

    thrust, _, _, pressure, _ = load_engine_tables(model_dir)
    if thrust_lbf < thrust[0]:
        return ""
    if thrust_lbf >= thrust[-1]:
        return pressure[-1]

    high = max(1, bisect_left(thrust, thrust_lbf))
    low = high - 1
    fraction = float32(float32(thrust_lbf - thrust[low]) /
                       float32(thrust[high] - thrust[low]))
    return float32(pressure[low] +
                   float32(fraction * float32(pressure[high] - pressure[low])))


def decode_record(data, model_dir=DEFAULT_MODEL_DIR):
    if len(data) != RECORD.size:
        raise ValueError(f"Incomplete record: expected 64 bytes, got {len(data)}")
    version = data[0]
    if version not in (2, 3, 4):
        raise ValueError(f"Unsupported format version {version}; expected 2, 3, or 4")
    layout = RECORD_V4 if version == 4 else RECORD
    version, state, presence, flags, timestamp, *fields = layout.unpack(data)
    allowed_presence = {2: 15, 3: 31, 4: 255}[version]
    if presence & ~allowed_presence or flags & ~15:
        raise ValueError("Invalid flags")
    if version == 2 and data[60:64] != b"\0" * 4:
        raise ValueError("Invalid version-2 reserved bytes")
    if version == 4:
        if fields[7] != 0:
            raise ValueError("Invalid version-4 reserved byte")
        if fields[5] != -128 and not -100 <= fields[5] <= 100:
            raise ValueError("Invalid version-4 applied output")
        encoder_valve_degrees = fields[3]
        commanded_valve_angle = fields[10]
        dropped_records = fields[11]
        thrust_lbf = fields[12]
        motor_values = (fields[4] if fields[4] != -128 else "",
                        fields[5] if fields[5] != -128 else "",
                        fields[8], fields[9], fields[6] if fields[6] != 255 else "")
    else:
        encoder_valve_degrees = fields[2]
        commanded_valve_angle = fields[8]
        dropped_records = fields[11]
        thrust_lbf = fields[12] if version == 3 else ""
        motor_values = fields[3:8]
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
               pt3_present=int(version == 4 and bool(presence & 32)),
               backlash_compensated=int(bool(presence & 16)),
               dropped_records=dropped_records,
               commanded_thrust_lbf=thrust_lbf)
    if version >= 3:
        row["commanded_mdot_lbm_per_sec"], row["commanded_dp_psi"] = commanded_targets(
            thrust_lbf, model_dir)
        row["commanded_chamber_pressure_psi"] = commanded_chamber_pressure(
            thrust_lbf, model_dir)
    gear_ratio, backlash_motor_rotations = load_angle_constants(model_dir)
    backlash_degrees = (backlash_motor_rotations * 360.0 / gear_ratio
                        if row["backlash_compensated"] else 0.0)
    # The logged valve target is BEFORE applying backlash, in valve degrees.
    if math.isfinite(commanded_valve_angle):
        row["commanded_valve_angle_deg"] = commanded_valve_angle
        row["commanded_motor_angle_deg"] = commanded_valve_angle + backlash_degrees
    if valid_time:
        row["timestamp_unix_ms"] = timestamp
    if presence & 1:
        row["pt1_psi"] = fields[0]
    if presence & 2:
        row["pt2_psi"] = fields[1]
    if version == 4 and presence & 32:
        row["pt3_psi"] = fields[2]
        goal_pressure = row["commanded_chamber_pressure_psi"]
        if goal_pressure != "" and math.isfinite(goal_pressure) and math.isfinite(fields[2]):
            try:
                error = float32(goal_pressure - fields[2])
            except OverflowError:
                pass
            else:
                if math.isfinite(error):
                    row["pid_error_psi"] = error
    if version == 4 and presence & 64:
        terms = fields[13:16]
        row.update(zip(("pid_p_output_deg", "pid_i_output_deg", "pid_d_output_deg"), terms))
        row["pid_output_clamped"] = int(bool(presence & 128))
        # Only an unclipped combined valve command identifies the applied PID
        # correction and original feedforward from these three contributions.
        if not presence & 128 and math.isfinite(commanded_valve_angle) and all(
                math.isfinite(value) for value in terms):
            try:
                correction = float32(float32(terms[0] + terms[1]) + terms[2])
                feedforward = float32(commanded_valve_angle - correction)
            except OverflowError:
                pass
            else:
                if math.isfinite(correction) and math.isfinite(feedforward):
                    row["pid_correction_deg"] = correction
                    row["feedforward_output_deg"] = feedforward
    if presence & 4:
        # All four angles use valve-side degrees. Motor feedback already includes
        # the mechanical offset; subtract it only for the valve estimate.
        if flags & 2 and math.isfinite(encoder_valve_degrees):
            row["measured_motor_angle_deg"] = encoder_valve_degrees
            row["measured_valve_angle_deg"] = encoder_valve_degrees - backlash_degrees
        for column, value in zip(("motor_temp_c", "output_percent",
                                  "rpm", "voltage_v", "current_a"), motor_values):
            row[column] = value
        for bit, column in enumerate(("has_status_0", "has_status_2",
                                      "has_telemetry", "is_fresh")):
            row[column] = int(bool(flags & (1 << bit)))
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
